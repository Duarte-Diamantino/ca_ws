"""Ligacao da GUI ao ROS e ao Gazebo (tudo no thread do Qt; o ROS e rodado por um QTimer)."""
import math
import os
import re
import signal
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray

os.environ.setdefault('IGN_IP', '127.0.0.1')   # como o resto da simulacao (ver README, Rede)

CONTROLLERS = {
    'mpc': ['ros2', 'run', 'mpc_python', 'mpc_controller', '--ros-args', '-p', 'use_sim_time:=true'],
    'dwa': ['ros2', 'run', 'dwa_follower', 'dwa_global_follower', '--ros-args', '-p', 'use_sim_time:=true'],
}
USER_MPC_PARAMS = os.environ.get('CA_MPC_PARAMS', os.path.expanduser('~/ca_ws/mpc_params.yaml'))
USER_DWA_PARAMS = os.environ.get('CA_DWA_PARAMS', os.path.expanduser('~/ca_ws/dwa_params.yaml'))


def dwa_param_files():
    files = []
    try:
        files.append(os.path.join(get_package_share_directory('dwa_follower'), 'config', 'dwa.yaml'))
    except PackageNotFoundError:
        pass
    files.append(USER_DWA_PARAMS)
    return [f for f in files if os.path.isfile(f)]


def mpc_param_files():
    files = []
    try:
        files.append(os.path.join(get_package_share_directory('mpc_python'), 'config', 'mpc.yaml'))
    except PackageNotFoundError:
        pass
    files.append(USER_MPC_PARAMS)
    return [f for f in files if os.path.isfile(f)]


def read_param_files(files):
    out = {}
    for f in files:
        with open(f) as fh:
            data = yaml.safe_load(fh) or {}
        out.update(data.get('mpc_controller', {}).get('ros__parameters', {}))
    return out


def write_user_params(values, path=USER_MPC_PARAMS):
    lines = ['# Parametros do MPC gravados na GUI (ou no terminal do MPC, tecla g).',
             '# Carregados depois de mpc_python/config/mpc.yaml. Apagar = voltar aos de origem.',
             'mpc_controller:', '  ros__parameters:']
    for n, v in values.items():
        val = ('true' if v else 'false') if isinstance(v, bool) else str(v) if isinstance(v, int) else repr(float(v))
        lines.append(f'    {n}: {val}')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return path


GZ_PREFIXES = ('ca_obs_', 'ca_obstaculo_')   # ca_obstaculo_N = nomes das versoes anteriores da GUI
GZ_RESYNC_MS = 15000                          # confirma o Gazebo de 15 em 15 s


def obstacle_name(x, y, r):
    """Nome fixo de cada obstaculo no Gazebo (posicao e raio em mm). O mesmo obstaculo tem sempre o
    mesmo nome (numa GUI reaberta tambem), por isso nunca fica em duplicado e os que sobram de uma
    sessao anterior reconhecem-se e tiram-se."""
    def mm(v):
        return str(int(round(float(v) * 1000))).replace('-', 'm')
    return f'ca_obs_{mm(x)}_{mm(y)}_{mm(r)}'


def parse_gz_models(scene_text):
    """Nomes dos obstaculos da GUI na resposta do /world/<w>/scene/info (ignition.msgs.Scene)."""
    names = re.findall(r'name: "([^"]+)"', scene_text)
    return {n for n in names if n.startswith(GZ_PREFIXES)}


def obstacle_sdf(name, r, h=0.3):
    geo = f'<geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>'
    return (f'<sdf version="1.8"><model name="{name}"><static>true</static><link name="l">'
            f'<collision name="c"><pose>0 0 {h / 2:.3f} 0 0 0</pose>{geo}</collision>'
            f'<visual name="v"><pose>0 0 {h / 2:.3f} 0 0 0</pose>{geo}<material>'
            '<ambient>0.85 0.12 0.10 1</ambient><diffuse>0.85 0.12 0.10 1</diffuse></material></visual>'
            '</link></model></sdf>')


class RosBridge(QObject):
    odom = pyqtSignal(float, float, float, float)      # x, y, yaw, v
    commands = pyqtSignal(float, float)                # rpm, servo
    prediction = pyqtSignal(list)                      # [(x, y)] do MPC
    driver = pyqtSignal(str)                           # quem conduz (/ca_sim/driver)
    log = pyqtSignal(str)
    controller_ended = pyqtSignal(str, int)            # tipo, codigo

    def __init__(self, world='pista'):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('ca_gui')
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)
        self.world = world
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        n = self.node
        self.path_pub = n.create_publisher(Path, '/reference_path', latched)
        self.obst_pub = n.create_publisher(PoseArray, '/obstacles', 10)
        self.obst_marker_pub = n.create_publisher(MarkerArray, '/obstacles_marker', 10)
        self.speed_pub = n.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = n.create_publisher(Float64, '/commands/servo/position', 10)
        self.driver_pub = n.create_publisher(String, '/ca_sim/driver', latched)
        n.create_subscription(String, '/ca_sim/driver', lambda m: self.driver.emit(m.data), latched)
        n.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 5)
        n.create_subscription(MarkerArray, '/predicted_trajectories', self._on_markers, 2)
        self._rpm, self._servo = None, None
        n.create_subscription(Float64, '/commands/motor/speed', lambda m: self._cmd(rpm=m.data), 10)
        n.create_subscription(Float64, '/commands/servo/position', lambda m: self._cmd(servo=m.data), 10)
        self.vesc_param = n.create_client(SetParameters, '/vesc_to_gz/set_parameters')
        self.mpc_get = n.create_client(GetParameters, '/mpc_controller/get_parameters')
        self.mpc_set = n.create_client(SetParameters, '/mpc_controller/set_parameters')
        self.path_msg = None
        self.obstacles = []
        self.proc = None
        self.proc_kind = None
        self.my_id = None
        self._gz = ThreadPoolExecutor(max_workers=1)
        self._gz_pending = False                      # ja ha uma sincronizacao na fila
        self._gz_closed = False
        self._gz_warned = set()
        self._create_exe = None
        try:
            self._create_exe = os.path.join(get_package_prefix('ros_gz_sim'), 'lib', 'ros_gz_sim', 'create')
        except PackageNotFoundError:
            pass
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin)
        self.spin_timer.start(20)
        self.pub_timer = QTimer(self)
        self.pub_timer.timeout.connect(self.republish)
        self.pub_timer.start(500)
        # O Gazebo e a referencia: no arranque tira obstaculos que ficaram de uma GUI anterior, e de
        # tempos a tempos volta a por/tirar o que nao bate certo (pedido perdido, Gazebo reaberto...)
        self.gz_timer = QTimer(self)
        self.gz_timer.timeout.connect(self.request_gz_sync)
        self.gz_timer.start(GZ_RESYNC_MS)
        QTimer.singleShot(1500, self.request_gz_sync)

    # ------------------------------------------------------------------ ROS
    def spin(self):
        # Ctrl+C no terminal: o rclpy fecha o contexto -> fechar a GUI (antes ficava a dar erros)
        if not rclpy.ok():
            self.closed_by_ros()
            return
        try:
            for _ in range(30):
                self.exec.spin_once(timeout_sec=0.0)
        except KeyboardInterrupt:
            self.closed_by_ros()
        except Exception:
            if not rclpy.ok():
                self.closed_by_ros()
            else:
                raise

    def closed_by_ros(self):
        from PyQt5.QtWidgets import QApplication
        self.spin_timer.stop()
        self.pub_timer.stop()
        self.stop_controller(brake=False)
        QApplication.quit()

    def _on_odom(self, m):
        p = m.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.odom.emit(p.position.x, p.position.y, yaw, m.twist.twist.linear.x)

    def _cmd(self, rpm=None, servo=None):
        if rpm is not None:
            self._rpm = rpm
        if servo is not None:
            self._servo = servo
        if self._rpm is not None and self._servo is not None:
            self.commands.emit(self._rpm, self._servo)

    def _on_markers(self, ma):
        pts = [(m.pose.position.x, m.pose.position.y) for m in ma.markers
               if m.type == Marker.SPHERE and 20000 <= m.id < 30000]
        if pts or any(m.action == Marker.DELETEALL for m in ma.markers):
            self.prediction.emit(pts)

    def republish(self):
        if not rclpy.ok():
            return
        if self.path_msg is not None:
            self.path_pub.publish(self.path_msg)
        self.publish_obstacles()

    # ------------------------------------------------------------------ caminho
    def publish_path(self, pts):
        msg = Path()
        msg.header.frame_id = 'map'
        now = self.node.get_clock().now()
        msg.header.stamp = now.to_msg()
        for i, (x, y) in enumerate(pts):
            j = min(i + 1, len(pts) - 1)
            k = max(j - 1, 0)
            yaw = math.atan2(pts[j][1] - pts[k][1], pts[j][0] - pts[k][0])
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            # stamps crescentes: o dwa_global_follower ordena o caminho pelo stamp
            ps.header.stamp = (now + rclpy.duration.Duration(nanoseconds=i * 1000)).to_msg()
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(ps)
        self.path_msg = msg
        self.path_pub.publish(msg)

    # ------------------------------------------------------------------ obstaculos
    def set_obstacles(self, obstacles):
        self.obstacles = list(obstacles)
        self.publish_obstacles()
        self.request_gz_sync()

    def request_gz_sync(self):
        """Sincroniza o Gazebo com self.obstacles (num thread; varios pedidos seguidos = um so)."""
        if self._gz_pending or self._gz_closed:
            return
        self._gz_pending = True
        try:
            self._gz.submit(self._gz_job)
        except RuntimeError:  # executor ja fechado
            self._gz_pending = False

    def _gz_job(self):
        self._gz_pending = False
        self._sync_gazebo(list(self.obstacles))

    def publish_obstacles(self):
        pa = PoseArray()
        pa.header.frame_id = 'map'
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.ns = 'obstacles'
        ma.markers.append(clear)
        for i, (x, y, r) in enumerate(self.obstacles):
            p = Pose()
            # z = raio do obstaculo: o MPC usa-o com OBSTACLE_SIZE_FROM_Z (ligado no config/mpc.yaml)
            p.position.x, p.position.y, p.position.z = float(x), float(y), float(r)
            pa.poses.append(p)
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = 'obstacles', i, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * float(r)
            m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.15, 0.1, 0.9
            ma.markers.append(m)
        self.obst_pub.publish(pa)
        self.obst_marker_pub.publish(ma)

    def _ign(self, *args, timeout=8, full=False):
        """ign service ... -> (ok, texto). full=True: ok = respondeu, texto inteiro."""
        ign = shutil_which('ign')
        if ign is None:
            return False, 'sem ign'
        res = subprocess.run(['ruby', ign] + list(args), capture_output=True, text=True, timeout=timeout)
        if full:
            return res.returncode == 0 and bool(res.stdout.strip()), res.stdout
        return res.returncode == 0 and 'true' in res.stdout, (res.stdout + res.stderr).strip()[:200]

    def gz_models(self, timeout_ms=4000):
        """Obstaculos da GUI que estao mesmo no Gazebo (None se o Gazebo nao responder)."""
        try:
            ok, out = self._ign('service', '-s', f'/world/{self.world}/scene/info',
                                '--reqtype', 'ignition.msgs.Empty', '--reptype', 'ignition.msgs.Scene',
                                '--timeout', str(timeout_ms), '--req', '',
                                timeout=timeout_ms / 1000 + 6, full=True)
        except subprocess.TimeoutExpired:
            return None
        return parse_gz_models(out) if ok else None

    def _sync_gazebo(self, obstacles, timeout_ms=4000):
        """Acerta os obstaculos do Gazebo com os da GUI (corre num thread a parte).

        [ca_ws] Antes a GUI guardava so em memoria o que tinha criado, com nomes ca_obstaculo_1, 2...
        Um pedido que falhava (Gazebo lento), uma GUI reaberta ou um Gazebo antigo ainda aberto
        deixavam obstaculos no Gazebo que a GUI ja nao via nem tirava. Agora pergunta ao Gazebo o que
        la esta e cada obstaculo tem um nome fixo (obstacle_name)."""
        if shutil_which('ign') is None:
            return
        try:
            want = {obstacle_name(x, y, r): (x, y, r) for x, y, r in obstacles}
            have = self.gz_models(timeout_ms)
            if have is None:
                return  # Gazebo fechado ou ainda a abrir: o gz_timer tenta outra vez
            for name in sorted(have - set(want)):
                ok, out = self._ign('service', '-s', f'/world/{self.world}/remove',
                                    '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                                    '--timeout', str(timeout_ms), '--req', f'name: "{name}" type: MODEL')
                if not ok:
                    self._warn_once(f'rm {name}', f'Gazebo: nao consegui tirar {name} (tento outra vez)')
            for name in sorted(set(want) - have):
                if self._create_exe is None:
                    return
                x, y, r = want[name]
                try:
                    res = subprocess.run([self._create_exe, '-world', self.world, '-name', name,
                                          '-string', obstacle_sdf(name, r), '-x', str(x), '-y', str(y), '-z', '0'],
                                         capture_output=True, text=True, timeout=20)
                    ok = res.returncode == 0
                except subprocess.TimeoutExpired:
                    ok = False
                if not ok:
                    self._warn_once(f'add {name}', f'Gazebo: nao consegui por o obstaculo em ({x:.2f}, {y:.2f}) '
                                    '(tento outra vez)')
        except Exception as exc:  # o Gazebo pode nao estar aberto
            self._warn_once(f'exc {exc}', f'Gazebo: {exc}')

    def _warn_once(self, key, text):
        if key not in self._gz_warned:
            self._gz_warned.add(key)
            self.log.emit(text)

    # ------------------------------------------------------------------ carro
    def move_car(self, x, y, yaw):
        def job():
            from ca_sim.gz_pose import set_car_pose
            ok, out = set_car_pose(x, y, yaw, world=self.world)
            self.log.emit('carro no inicio do caminho' if ok else f'nao consegui mover o carro: {out}')
        self._gz.submit(job)

    def claim(self, kind, profile):
        self.my_id = f'gui-{kind}:{os.getpid()}'
        self.driver_pub.publish(String(data=self.my_id))
        if self.vesc_param.wait_for_service(timeout_sec=0.3):
            req = SetParameters.Request()
            req.parameters = [Parameter(name='profile', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=profile))]
            self.vesc_param.call_async(req)

    def send_command(self, rpm, servo):
        self.speed_pub.publish(Float64(data=float(rpm)))
        self.steer_pub.publish(Float64(data=float(servo)))

    # ------------------------------------------------------------------ controladores
    def start_controller(self, kind):
        self.stop_controller()
        cmd = list(CONTROLLERS[kind])
        for f in (mpc_param_files() if kind == 'mpc' else dwa_param_files()):
            cmd += ['--params-file', f]
        env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1')
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     env=env, start_new_session=True)
        self.proc_kind = kind
        threading.Thread(target=self._pump, args=(self.proc, kind), daemon=True).start()
        self.claim(kind, 'dwa' if kind == 'dwa' else 'mpc')

    def _pump(self, proc, kind):
        for line in proc.stdout:
            if getattr(proc, 'ca_stopping', False):
                continue
            line = line.rstrip()
            if line and 'UserWarning' not in line and 'warnings.warn' not in line:
                self.log.emit(f'[{kind}] {line}')
        code = proc.wait()
        if not getattr(proc, 'ca_stopping', False):
            self.controller_ended.emit(kind, code)

    def stop_controller(self, brake=True):
        p = self.proc
        if p is not None and p.poll() is None:
            p.ca_stopping = True
            try:
                os.killpg(p.pid, signal.SIGINT)
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.proc = None
        self.proc_kind = None
        if brake:
            for _ in range(3):
                self.send_command(0.0, 0.5751)

    # ------------------------------------------------------------------ parametros do MPC
    def _call(self, cli, req, timeout=2.0):
        if not cli.wait_for_service(timeout_sec=0.5):
            return None
        fut = cli.call_async(req)
        self.exec.spin_until_future_complete(fut, timeout_sec=timeout)
        return fut.result()

    def mpc_get_params(self, names):
        req = GetParameters.Request()
        req.names = list(names)
        res = self._call(self.mpc_get, req)
        if res is None:
            return None
        out = {}
        for n, v in zip(names, res.values):
            if v.type == ParameterType.PARAMETER_BOOL:
                out[n] = v.bool_value
            elif v.type == ParameterType.PARAMETER_INTEGER:
                out[n] = v.integer_value
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                out[n] = v.double_value
        return out

    def mpc_set_params(self, values):
        req = SetParameters.Request()
        for n, v in values.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, v
            elif isinstance(v, int):
                pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, v
            else:
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(v)
            req.parameters.append(Parameter(name=n, value=pv))
        res = self._call(self.mpc_set, req)
        if res is None:
            return None
        return [(p.name, r.successful, r.reason) for p, r in zip(req.parameters, res.results)]

    def shutdown(self):
        self.spin_timer.stop()
        self.pub_timer.stop()
        self.gz_timer.stop()
        self.stop_controller(brake=rclpy.ok())
        self._gz_closed = True
        # tira os obstaculos da GUI do Gazebo (pouco tempo: com Ctrl+C no launch o Gazebo tambem fecha)
        self._gz.submit(self._sync_gazebo, [], 1500)
        self._gz.shutdown(wait=True)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def shutil_which(name):
    import shutil
    return shutil.which(name)
