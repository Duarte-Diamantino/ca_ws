#!/usr/bin/env python3
"""Conduzir o carro simulado a partir de um terminal (um terminal por modo).

  ros2 run ca_sim drive mpc      espaco = iniciar / parar o MPC  (mpc_python/mpc_controller)
  ros2 run ca_sim drive dwa      espaco = iniciar / parar o DWA  (dwa_follower/dwa_global_follower)
  ros2 run ca_sim drive remote   como um comando de RC: enquanto carregas em W (ou seta para cima)
                                 o carro acelera, S trava e depois faz marcha atras, A/D viram;
                                 ao largar as teclas a velocidade desce ate 0 e a direcao volta ao
                                 centro. espaco = travar ja. m = modo "manter" (W/S/A/D mudam
                                 0.1 m/s / 2 graus por toque e o valor fica; C = direcao ao centro)
  Em todos:  r = parar e por o carro no inicio do caminho (Gazebo)   q ou Ctrl+C = sair

Precisa da simulacao a correr: ros2 launch ca_sim gazebo_sim.launch.py

A ultima linha do terminal mostra sempre o que esta a ser mandado ao carro (/commands/motor/speed
e /commands/servo/position, convertidos para m/s e graus) e o que o carro faz (/odometry/filtered).

Quem comeca a conduzir anuncia-se em /ca_sim/driver e os outros terminais largam o carro
(o controlador deles para), por isso nunca ha dois a mandar no carro ao mesmo tempo.
Ao comecar a conduzir, poe o carro simulado (/vesc_to_gz) com a calibracao desse
controlador (MPC e DWA assumem calibracoes diferentes, ver README). O remote usa a do MPC
(ou --profile dwa).
"""
import argparse
import math
import os
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty

import rclpy
from nav_msgs.msg import Odometry, Path
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

from ca_sim.gz_pose import set_car_pose
from ca_sim.vesc import PROFILES, VescCalibration, start_pose_from_path

CONTROLLERS = {
    'mpc': ['ros2', 'run', 'mpc_python', 'mpc_controller', '--ros-args', '-p', 'use_sim_time:=true'],
    # os outros parametros (v_max, goal_tol, ...) vem de dwa_follower/config/dwa.yaml (controller_cmd)
    'dwa': ['ros2', 'run', 'dwa_follower', 'dwa_global_follower', '--ros-args', '-p', 'use_sim_time:=true'],
}
HELP = {
    'mpc': ('espaco = iniciar/parar o MPC | p = ver parametros | t = mudar um | g = guardar | '
            'r = carro ao inicio | q = sair'),
    'dwa': 'espaco = iniciar/parar o DWA | r = carro ao inicio | q = sair',
    'remote': ('W/S ou setas = acelerar/travar (larga -> volta a 0) | A/D = virar (larga -> centro) | '
               'espaco = travar | m = modo manter | r = carro ao inicio | q = sair'),
}
V_MIN, V_MAX = -1.0, 3.0
# modo normal (como um comando de RC)
ACCEL = 0.8                         # m/s^2 enquanto W esta carregado
BRAKE = 2.5                         # m/s^2 com S a andar para a frente
REVERSE_ACCEL = 0.6                 # m/s^2 com S parado -> marcha atras (ate V_MIN)
COAST = 0.8                         # m/s^2 sem W/S: a velocidade desce ate 0
STEER_RATE = math.radians(80.0)     # rad/s com A/D
STEER_RETURN = math.radians(120.0)  # rad/s sem A/D: a direcao volta ao centro
# Um terminal nao avisa quando se larga uma tecla: so recebe a letra repetida pelo auto-repeat do
# teclado (~30 por segundo, depois de ~0.5 s). A tecla conta como carregada se chegou ha menos de
# HOLD_FIRST (primeiro toque, antes do auto-repeat comecar) ou HOLD_REPEAT (ja a repetir).
HOLD_FIRST = 0.55
HOLD_REPEAT = 0.12
# modo "manter" (o comportamento antigo)
V_STEP = 0.1
STEER_STEP = math.radians(2.0)
ODOM_TIMEOUT = 2.0   # s sem /odometry/filtered a conduzir -> avisa (o MPC/DWA nao mexe o carro sem pose)
# parametros do MPC: os do pacote (mpc_python/config/mpc.yaml) e depois os teus (gravados com "g")
USER_MPC_PARAMS = os.environ.get('CA_MPC_PARAMS', os.path.expanduser('~/ca_ws/mpc_params.yaml'))
# idem para o DWA: dwa_follower/config/dwa.yaml (simulador) e depois os teus
USER_DWA_PARAMS = os.environ.get('CA_DWA_PARAMS', os.path.expanduser('~/ca_ws/dwa_params.yaml'))
STATUS_PERIOD = 0.2  # s entre atualizacoes da linha de estado
ARROWS = {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}   # ESC [ A..D = setas

_out_lock = threading.Lock()
_status = {'shown': False, 'tty': sys.stdout.isatty()}


def say(text):
    """Escreve uma linha por cima da linha de estado (que volta a aparecer no proximo tick)."""
    with _out_lock:
        if _status['shown']:
            sys.stdout.write('\r\033[K')
            _status['shown'] = False
        sys.stdout.write(text + '\n')
        sys.stdout.flush()


def show_status(text):
    with _out_lock:
        if _status['tty']:
            sys.stdout.write('\r\033[K' + text)
            _status['shown'] = True
        else:                         # pipe/ficheiro (testes): uma linha normal
            sys.stdout.write('ESTADO ' + text + '\n')
        sys.stdout.flush()


class KeyReader:
    """Le teclas sem esperar pelo Enter (cbreak). Tambem funciona com stdin num pipe (testes)."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = None
        self.eof = False
        if sys.stdin.isatty():
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def read(self):
        """Devolve as teclas que chegaram (lista, setas ja convertidas em w/a/s/d)."""
        if self.eof or not select.select([self.fd], [], [], 0)[0]:
            return []
        data = os.read(self.fd, 64)
        if not data:
            self.eof = True
            return []
        text = data.decode(errors='ignore')
        keys, i = [], 0
        while i < len(text):
            if text[i] == '\x1b' and text[i + 1:i + 2] == '[' and i + 2 < len(text):
                if text[i + 2] in ARROWS:
                    keys.append(ARROWS[text[i + 2]])
                i += 3
                continue
            keys.append(text[i].lower())
            i += 1
        return keys

    def restore(self):
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class Drive(Node):
    def __init__(self, mode, profile):
        super().__init__(f'drive_{mode}')
        self.mode = mode
        self.profile = profile
        self.calib = VescCalibration.from_profile(profile)
        self.my_id = f'{mode}:{os.getpid()}'
        self.driving = False
        self.proc = None
        self.path_pts = None
        # comando (remote)
        self.v = 0.0
        self.delta = 0.0
        self.hold_mode = False
        self.key_t = {}               # tecla -> (instante do ultimo evento, ja a repetir?)
        self.latch = {}               # tecla presa -> tecla do outro grupo que a mantem (W+A)
        self.last_tick = time.monotonic()
        # o que esta a ser mandado ao carro e o que o carro faz (linha de estado)
        self.cmd_rpm = None
        self.cmd_servo = None
        self.odom = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.driver_pub = self.create_publisher(String, '/ca_sim/driver', latched)
        self.create_subscription(String, '/ca_sim/driver', self.on_driver, latched)
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.create_subscription(Float64, '/commands/motor/speed',
                                 lambda m: setattr(self, 'cmd_rpm', m.data), 10)
        self.create_subscription(Float64, '/commands/servo/position',
                                 lambda m: setattr(self, 'cmd_servo', m.data), 10)
        self.create_subscription(Path, '/reference_path', self.on_path, 1)
        # pose do carro: linha de estado e aviso quando nao chega (ligacao ao Gazebo partida)
        self.last_odom = None
        self.drive_since = None
        self.odom_warned = False
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 1)
        self.param_cli = self.create_client(SetParameters, '/vesc_to_gz/set_parameters')
        self.keys = None              # KeyReader (para pedir uma linha de texto no "t")
        if mode == 'mpc':
            self.mpc_list = self.create_client(ListParameters, '/mpc_controller/list_parameters')
            self.mpc_get = self.create_client(GetParameters, '/mpc_controller/get_parameters')
            self.mpc_set = self.create_client(SetParameters, '/mpc_controller/set_parameters')
        if mode == 'remote':
            self.create_timer(0.05, self.remote_tick)  # 20 Hz enquanto conduz (o sim trava apos 0.5 s)
        self.create_timer(STATUS_PERIOD, self.print_status)

    # ---------- quem conduz ----------
    def claim(self):
        self.driver_pub.publish(String(data=self.my_id))
        self.set_car_profile()
        self.driving = True
        self.drive_since = time.monotonic()
        self.odom_warned = False

    def on_driver(self, msg):
        if self.driving and msg.data != self.my_id:
            who = msg.data.split(':')[0].upper()
            self.release(f'{who} assumiu o controlo do carro')

    def release(self, reason, brake=False):
        """Larga o carro. brake=False quando outro terminal o assumiu (nao mandar zeros por cima)."""
        say(f'>>> {reason}')
        if self.mode == 'remote':
            self.v = 0.0
            self.delta = 0.0
            self.key_t.clear()
            self.latch.clear()
            self.driving = False
            if brake:
                self.publish_stop()
        else:
            self.stop_controller(brake=brake)

    def set_car_profile(self):
        if not self.param_cli.wait_for_service(timeout_sec=0.5):
            say('    (sem /vesc_to_gz: a calibracao do carro simulado nao foi mudada)')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name='profile', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=self.profile))]
        self.param_cli.call_async(req)
        say(f'    carro simulado com a calibracao do {self.profile.upper()}')

    def publish_stop(self):
        for _ in range(3):
            self.speed_pub.publish(Float64(data=0.0))
            self.steer_pub.publish(Float64(data=self.calib.servo_center))

    # ---------- controlador (mpc / dwa) ----------
    def controller_cmd(self):
        cmd = list(CONTROLLERS[self.mode])
        pkg, cfg, user = {'mpc': ('mpc_python', 'mpc.yaml', USER_MPC_PARAMS),
                          'dwa': ('dwa_follower', 'dwa.yaml', USER_DWA_PARAMS)}[self.mode]
        files = [user]
        try:
            files.insert(0, os.path.join(get_package_share_directory(pkg), 'config', cfg))
        except PackageNotFoundError:
            pass
        for f in files:
            if os.path.isfile(f):
                cmd += ['--params-file', f]
                if f == user:
                    say(f'    parametros do {self.mode.upper()}: os teus, de {f}')
        return cmd

    def start_controller(self):
        self.claim()
        env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1')
        self.proc = subprocess.Popen(self.controller_cmd(), stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, env=env,
                                     start_new_session=True)
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        say(f'>>> {self.mode.upper()} a conduzir (espaco para parar)')

    def _pump(self, proc):
        tag = f'[{self.mode}] '
        for line in proc.stdout:
            # depois do SIGINT o rclpy (Humble) as vezes rebenta a meio de ler uma mensagem e o
            # MPC escreve 2 tracebacks ao sair: e so ruido de saida, nao mostrar
            if getattr(proc, 'ca_stopping', False):
                continue
            line = line.rstrip()
            if line and 'UserWarning' not in line and 'warnings.warn' not in line:
                say(tag + line)

    def stop_controller(self, brake=True):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.ca_stopping = True
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
            except ProcessLookupError:
                pass
        self.proc = None
        if self.driving and brake:
            self.publish_stop()
            say(f'>>> {self.mode.upper()} parado, carro travado')
        elif self.driving:
            say(f'>>> {self.mode.upper()} parado')
        self.driving = False

    def check_controller(self):
        if self.proc is not None and self.proc.poll() is not None:
            say(f'>>> o {self.mode.upper()} terminou sozinho (codigo {self.proc.returncode})')
            self.proc = None
            self.driving = False
            self.publish_stop()

    # ---------- parametros do MPC (p / t / g) ----------
    def _call(self, cli, req, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=1.0):
            say('>>> o MPC nao esta a correr (espaco para o arrancar)')
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def mpc_params(self):
        """{nome: valor} dos parametros ajustaveis do MPC (None se o MPC nao responder)."""
        res = self._call(self.mpc_list, ListParameters.Request())
        if res is None:
            return None
        names = sorted(n for n in res.result.names if n != 'use_sim_time' and not n.startswith('qos_'))
        req = GetParameters.Request()
        req.names = names
        got = self._call(self.mpc_get, req)
        if got is None:
            return None
        out = {}
        for n, v in zip(names, got.values):
            if v.type == ParameterType.PARAMETER_BOOL:
                out[n] = v.bool_value
            elif v.type == ParameterType.PARAMETER_INTEGER:
                out[n] = v.integer_value
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                out[n] = v.double_value
        return out

    def show_mpc_params(self):
        params = self.mpc_params()
        if params is None:
            return
        cells = [f'{n} = {v:g}' if isinstance(v, float) else f'{n} = {v}' for n, v in params.items()]
        width = max(len(c) for c in cells) + 2
        lines = ['    ' + ''.join(c.ljust(width) for c in cells[i:i + 3]) for i in range(0, len(cells), 3)]
        say('>>> parametros do MPC (t = mudar, g = guardar):\n' + '\n'.join(lines))

    def ask_line(self, prompt):
        with _out_lock:
            if _status['shown']:
                sys.stdout.write('\r\033[K')
                _status['shown'] = False
        if self.keys is not None:
            self.keys.restore()
        try:
            return input(prompt)
        except EOFError:
            return ''
        finally:
            if self.keys is not None and self.keys.old is not None:
                tty.setcbreak(self.keys.fd)

    def tune_mpc_param(self):
        params = self.mpc_params()
        if params is None:
            return
        line = self.ask_line('    parametro e valor (ex.: V_MAX 1.5), Enter = cancelar: ').strip()
        if not line:
            return
        parts = line.replace('=', ' ').split()
        if len(parts) != 2:
            say('>>> escreve o nome e o valor, ex.: V_MAX 1.5')
            return
        byname = {n.lower(): n for n in params}
        name = byname.get(parts[0].lower())
        if name is None:
            say(f'>>> nao ha o parametro "{parts[0]}" (p mostra a lista)')
            return
        old = params[name]
        pv = ParameterValue()
        try:
            if isinstance(old, bool):
                pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, parts[1].lower() in ('1', 'true', 'sim', 'yes')
            elif isinstance(old, int):
                pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, int(float(parts[1]))
            else:
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(parts[1])
        except ValueError:
            say(f'>>> valor invalido: {parts[1]}')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=pv)]
        res = self._call(self.mpc_set, req)
        if res is not None:
            r = res.results[0]
            say(f'>>> {name}: {old} -> {parts[1]}' if r.successful else f'>>> recusado: {r.reason}')

    def save_mpc_params(self):
        params = self.mpc_params()
        if params is None:
            return
        lines = [f'# Parametros do MPC gravados no terminal do MPC (g) em {time.strftime("%Y-%m-%d %H:%M")}.',
                 '# O tools/sim.sh mpc carrega-os depois de mpc_python/config/mpc.yaml. Apagar = voltar aos de origem.',
                 'mpc_controller:', '  ros__parameters:']
        for n, v in params.items():
            val = ('true' if v else 'false') if isinstance(v, bool) else str(v) if isinstance(v, int) else repr(float(v))
            lines.append(f'    {n}: {val}')
        with open(USER_MPC_PARAMS, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        say(f'>>> {len(params)} parametros gravados em {USER_MPC_PARAMS} (usados no proximo arranque do MPC)')

    # ---------- remote (teclado) ----------
    def remote_key(self, ch):
        if ch == 'm':
            self.hold_mode = not self.hold_mode
            say('>>> modo MANTER: W/S = +/-0.1 m/s e A/D = 2 graus por toque, o valor fica; C = centro'
                if self.hold_mode else
                '>>> modo NORMAL: enquanto carregas acelera/vira; ao largar volta a 0 e ao centro')
            return
        if ch not in 'wsadc ':
            return
        if not self.driving:
            self.claim()
            say('>>> comando a conduzir')
        if ch == ' ':
            self.v = 0.0
        elif ch == 'c':
            self.delta = 0.0
        elif self.hold_mode:
            if ch == 'w':
                self.v = min(V_MAX, self.v + V_STEP)
            elif ch == 's':
                self.v = max(V_MIN, self.v - V_STEP)
            elif ch == 'a':
                self.delta = min(self.calib.steer_max, self.delta + STEER_STEP)
            elif ch == 'd':
                self.delta = max(-self.calib.steer_max, self.delta - STEER_STEP)
        else:
            now = time.monotonic()
            # duas teclas ao mesmo tempo (W+A, W+D, S+A, S+D): o terminal so repete a ultima. Se uma
            # tecla do outro grupo estava carregada quando esta chegou, fica "presa" a esta enquanto
            # esta repetir (ver held_eff).
            for k in ('ws' if ch in 'ad' else 'ad'):
                if self.held_eff(k, now):
                    self.latch[k] = ch
            self.latch.pop(ch, None)
            for k in ('ad' if ch in 'ad' else 'ws'):
                if k != ch:
                    self.latch.pop(k, None)    # a tecla oposta do mesmo grupo foi largada
            last = self.key_t.get(ch)
            self.key_t[ch] = (now, last is not None and now - last[0] < 0.25)

    def held(self, ch, now):
        k = self.key_t.get(ch)
        return k is not None and now - k[0] < (HOLD_REPEAT if k[1] else HOLD_FIRST)

    def held_eff(self, ch, now):
        """Carregada a repetir, ou presa a uma tecla do outro grupo que esta a repetir."""
        if self.held(ch, now):
            return True
        src = self.latch.get(ch)
        if src is not None and self.held(src, now):
            return True
        self.latch.pop(ch, None)
        return False

    def remote_tick(self):
        now = time.monotonic()
        dt = min(0.2, now - self.last_tick)
        self.last_tick = now
        if not self.driving:
            return
        if not self.hold_mode:
            w, s = self.held_eff('w', now), self.held_eff('s', now)
            a, d = self.held_eff('a', now), self.held_eff('d', now)
            # acelerador preso a A/D (W+A): mantem a velocidade enquanto vira, nao acelera nem desce
            # (nao da para saber se o W continua carregado ou ja foi largado)
            cruise = (w and 'w' in self.latch) or (s and 's' in self.latch)
            if cruise:
                pass
            elif w and not s:
                self.v = min(V_MAX, self.v + ACCEL * dt)
            elif s and not w:
                if self.v > 0.0:
                    self.v = max(0.0, self.v - BRAKE * dt)       # trava ate 0 antes da marcha atras
                else:
                    self.v = max(V_MIN, self.v - REVERSE_ACCEL * dt)
            else:                                                # largou: desce ate 0
                self.v = math.copysign(max(0.0, abs(self.v) - COAST * dt), self.v)
            target = self.calib.steer_max * ((1.0 if a else 0.0) - (1.0 if d else 0.0))
            rate = STEER_RATE if target != 0.0 else STEER_RETURN
            step = max(-rate * dt, min(rate * dt, target - self.delta))
            self.delta += step
        self.speed_pub.publish(Float64(data=self.calib.motor_cmd(self.v)))
        self.steer_pub.publish(Float64(data=self.calib.servo_pos(self.delta)))

    # ---------- linha de estado ----------
    def print_status(self):
        if self.mode == 'remote':
            who = ('comando MANTER' if self.hold_mode else 'comando') + (' a conduzir' if self.driving else ' parado')
        else:
            who = self.mode.upper() + (' a conduzir' if self.proc is not None else ' parado')
        if self.cmd_rpm is not None and self.cmd_servo is not None:
            v_cmd = self.calib.speed(self.cmd_rpm)
            ang = math.degrees(self.calib.steer(self.cmd_servo))
            lado = 'esq' if ang > 0.5 else 'dir' if ang < -0.5 else '   '
            cmd = (f'cmd {v_cmd:+5.2f} m/s ({self.cmd_rpm:5.0f} rpm)  dir {abs(ang):4.1f} graus {lado} '
                   f'(servo {self.cmd_servo:.3f})')
        else:
            cmd = 'cmd --'
        if self.odom is not None:
            p = self.odom.pose.pose
            yaw = math.degrees(2.0 * math.atan2(p.orientation.z, p.orientation.w))
            v = self.odom.twist.twist.linear.x
            car = f'carro {v:+5.2f} m/s  x {p.position.x:5.2f} y {p.position.y:5.2f} yaw {yaw:4.0f}'
        else:
            car = 'carro: sem pose'
        show_status(f'[{who}] {cmd} | {car}')

    # ---------- pose do carro ----------
    def on_odom(self, msg):
        self.odom = msg
        self.last_odom = time.monotonic()
        if self.odom_warned:
            self.odom_warned = False
            say('>>> a pose do carro (/odometry/filtered) voltou a chegar')

    def check_odom(self):
        if not self.driving or self.odom_warned:
            return
        now = time.monotonic()
        since = self.last_odom if self.last_odom is not None else self.drive_since
        if since is not None and now - max(since, self.drive_since) > ODOM_TIMEOUT:
            self.odom_warned = True
            nunca = 'nunca chegou' if self.last_odom is None else f'nao chega ha {now - self.last_odom:.0f} s'
            dom = os.environ.get('ROS_DOMAIN_ID', '0')
            loc = os.environ.get('ROS_LOCALHOST_ONLY', '0')
            say(f'>>> AVISO: a pose do carro (/odometry/filtered) {nunca}: sem ela o MPC/DWA nao '
                'mexe o carro (e o RViz nao o mostra a andar).\n'
                f'    Este terminal esta no ROS_DOMAIN_ID={dom} (ROS_LOCALHOST_ONLY={loc}). A simulacao '
                'esta aberta e foi aberta com tools/sim.sh, como este terminal?\n'
                '    Se sim, a ligacao ao Gazebo caiu: fecha a simulacao (Ctrl+C no terminal 1) e '
                'volta a abri-la.')

    # ---------- carro ao inicio ----------
    def on_path(self, msg):
        if len(msg.poses) >= 2:
            self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def reset_car(self):
        if self.mode == 'remote':
            if self.driving:
                self.release('comando parado', brake=True)
        else:
            self.stop_controller()
        x, y, yaw = start_pose_from_path(self.path_pts) if self.path_pts else (-0.4, 0.0, 0.0)
        ok, out = set_car_pose(x, y, yaw)
        if ok:
            say(f'>>> carro no inicio do caminho (x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} graus)')
        else:
            say(f'>>> nao consegui mover o carro: {out}')

    # ---------- teclas ----------
    def handle_key(self, ch):
        if ch == 'q':
            return False
        if ch == 'r':
            self.reset_car()
        elif self.mode == 'remote':
            self.remote_key(ch)
        elif self.mode == 'mpc' and ch in 'ptg':
            {'p': self.show_mpc_params, 't': self.tune_mpc_param, 'g': self.save_mpc_params}[ch]()
        elif ch in (' ', '\n', '\r'):
            if self.proc is None:
                self.start_controller()
            else:
                self.stop_controller()
        return True

    def shutdown(self):
        if self.mode == 'remote':
            if self.driving:
                self.publish_stop()
        else:
            self.stop_controller()


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if args is None else args)[1:]
    ap = argparse.ArgumentParser(prog='ros2 run ca_sim drive', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['mpc', 'dwa', 'remote'])
    ap.add_argument('--profile', choices=sorted(PROFILES),
                    help='calibracao a usar (defeito: a do controlador; remote: mpc)')
    a = ap.parse_args(argv)
    profile = a.profile or ('dwa' if a.mode == 'dwa' else 'mpc')

    rclpy.init(args=args)
    node = Drive(a.mode, profile)
    keys = KeyReader()
    node.keys = keys
    say(f'=== {a.mode.upper()} === {HELP[a.mode]}')
    running = True
    try:
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.02)
            for ch in keys.read():
                if not node.handle_key(ch):
                    running = False
                    break
            node.check_controller()
            node.check_odom()
    except KeyboardInterrupt:
        pass
    finally:
        keys.restore()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        say('=== fim ===')


if __name__ == '__main__':
    main()
