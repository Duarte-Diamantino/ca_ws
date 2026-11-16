#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange, ParameterDescriptor,  # [ca_ws]
                                SetParametersResult)
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy
import numpy as np
import math, time, hashlib, sys
from scipy.optimize import minimize
from scipy.spatial import KDTree

# ===== Timing / Horizonte =====
dt = 0.3
H_SHORT = 6
H_LONG  = 16
WINDOW_AHEAD = 50

# ===== Veículo / limites =====
V_MAX = 1.2
V_MIN = 0.2
GAMMA_MAX = math.pi / 8.0
WHEEL_BASE = 0.335
ALIGN_HEADING_THRESH_DEG = 8.0
ALIGN_CTE_THRESH = 0.08
V_MAX_NOT_ALIGNED = 0.8
ACCEL_LIMIT = 0.5
DECEL_LIMIT = 1.8
OMEGA_MAX = 0.7

# ===== Servo =====
SERVO_CENTER = 0.5751
SERVO_MAX = 0.8760
SERVO_MIN = 0.2639

# ===== Pesos MPC =====
w_cross_track = 15.0
w_heading = 5.0
w_control_effort = 0.1
w_control_smooth = 2.0
w_terminal = 15.0

# ===== Pista / segurança =====
TRACK_HALF_WIDTH = 0.50
MAX_LAT_ACC = 1.2
V_SAFETY_MARGIN = 0.15
PATH_TUBE_LIMIT = 0.55   # <<< novo: “tubo” máximo à volta da trajetória para rejeitar cortes

# ===== Obstáculos =====
REQUIRE_OBSTACLES_TO_START = True
OBSTACLE_RADIUS = 0.5
PATH_FILTER_MARGIN = 0.10
OBS_APPEARS_THRESH_FOR_SLOW = 0.15
LOCAL_SCAN_MULT = 6
# [ca_ws] Custo de obstaculo suave: W*(R_SOFT - d)^2 quando d < R_SOFT = OBSTACLE_RADIUS + margem.
# Antes cada plano com um estado a < OBSTACLE_RADIUS devolvia um custo fixo (1e9): patamar sem
# gradiente, o L-BFGS-B nao conseguia afastar o plano do obstaculo e o carro parava.
# A validacao dura (states_hit_obstacle com OBSTACLE_RADIUS) continua igual, por isso o custo
# suave comeca antes (margem) e cresce depressa: passar a 0.5 m custa 1e5 * 0.2^2 = 4000.
OBSTACLE_SOFT_MARGIN = 0.20
W_OBSTACLE_SOFT = 1.0e5
# [ca_ws] Tolerancia do L-BFGS-B (era 0.1 = parar quando uma iteracao melhora < 10%). Com 0.1 o
# plano ficava as vezes 1 cm dentro do raio duro e o carro parava. Custo: update() ~6 -> ~18 ms
# (max ~55 ms) num PC; medir na Jetson (o ciclo e de dt = 300 ms).
MPC_FTOL = 1e-3
MPC_MAXITER = 32          # [ca_ws] iteracoes maximas do L-BFGS-B (era fixo no codigo)
# [ca_ws] Modo "contornar" (rejoin sequencial) desligado: comecava a referencia DEPOIS do bloco
# inseguro com o carro ainda ~2 m antes; todos os planos cortavam a curva 1.4-2 m e eram
# rejeitados pelo tubo (PATH_TUBE_LIMIT) -> "STOP: sem trajetoria livre apos obstaculo".
# Com o custo suave o plano normal contorna sozinho. Ver tools/mpc_offline.py e o README.
USE_REJOIN = False
# [ca_ws] Obstaculo EM CIMA do caminho: a referencia e desviada para o lado (desvio lateral suave,
# perpendicular ao caminho) ate ficar a raio duro + AVOID_MARGIN do obstaculo, e o resto do MPC (custo, tubo
# PATH_TUBE_LIMIT, progresso) usa esse caminho desviado. Antes os pontos perto do obstaculo eram
# apagados (filter_ref_points_near_obstacles) e todos os planos que desviavam mais de 0.55 m do
# caminho original eram rejeitados pelo tubo -> o carro parava ~1.4 m antes do obstaculo.
# O MPC nao conhece a pista: o desvio pode sair da faixa se o obstaculo estiver junto a berma.
USE_REF_SHIFT = True
AVOID_MARGIN = 0.10       # o caminho desviado passa a (raio duro do obstaculo + isto) do obstaculo
AVOID_RAMP = 2.0          # m de caminho para entrar/sair do desvio (raio minimo do carro ~0.8 m)
AVOID_DEFAULT_SIDE = 1.0  # obstaculo mesmo em cima do caminho: +1 contorna pela esquerda, -1 pela direita
AVOID_CENTER_TOL = 0.05   # |desvio lateral do obstaculo| abaixo disto = "em cima do caminho"
# Obstaculos a menos de OBSTACLE_RADIUS + margem + isto da referencia entram no custo e na validacao
# (com o caminho desviado a referencia ja nao passa a < OBSTACLE_RADIUS do obstaculo).
OBSTACLE_LOOKAROUND = 0.50
# [ca_ws] Raio de cada obstaculo. Com OBSTACLE_SIZE_FROM_Z, o /obstacles traz o tamanho de cada um em
# pose.position.z (raio do obstaculo, > 0) e o raio duro (distancia minima ao centro do carro) e
# esse + CAR_HALF_WIDTH + OBSTACLE_MARGIN: o obstaculo e o que e, mais o carro, mais a folga.
# Obstaculos sem tamanho (z = 0, ou OBSTACLE_SIZE_FROM_Z desligado) usam o OBSTACLE_RADIUS fixo.
# Desligado por defeito (no carro real nao se sabe o que a percecao poe em z); o simulador
# (config/mpc.yaml, GUI ca_gui) liga-o.
OBSTACLE_SIZE_FROM_Z = False
CAR_HALF_WIDTH = 0.10      # meia largura do carro (0.193 m de largura) [m]
OBSTACLE_MARGIN = 0.10     # folga entre a parte lateral do carro e o obstaculo [m]
# [ca_ws] Referencia local espacada pela velocidade: o ponto k fica a (k+1)*v*dt do carro ao longo
# do caminho, com v = ultima velocidade comandada (>= REF_SPACING_V_MIN). Antes eram sempre 3 pontos
# por passo (0.45 m / 0.3 s = 1.5 m/s > V_MAX): a referencia fugia a frente do carro, o custo
# (distancia ao ponto k, penalizacao de TRACK_HALF_WIDTH) puxava o plano para dentro das curvas.
REF_SPEED_SPACING = True    # False = como antes (3 pontos por passo)
REF_SPACING_V_MIN = 0.5
REF_SPACING_LEAD = 0.15     # m/s a mais que a ultima velocidade (deixa o MPC acelerar)

# Retângulo branco (apenas visual)
OBSTACLE_RECT_SIZE = (0.5, 0.5, 0.1)

# ===== Loop / fim =====
LOOP_CLOSE_TOL = 1.0
WRAP_NEAR_START_DIST = 0.9
END_SLOWDOWN_REMAIN = 2.0
END_STOP_REMAIN = 0.80
END_FORCE_STOP_RADIUS = 0.50

# ===== Progresso exigido =====
PROGRESS_SLOWDOWN_MIN = 0.50
PROGRESS_STOP_MIN     = 0.80

# ===== Resampling =====
RESAMPLE_STEP = 0.15

# ===== Velocidade→RPM (hard min 3000) =====
RPM_CMD_MIN = 3000.0
RPM_MAX = 9000.0
GAIN_RPM_PER_MS = 7000.0
def v_to_rpm(v_mps: float) -> float:
    if v_mps <= 0.0: return 0.0
    rpm = max(RPM_CMD_MIN, GAIN_RPM_PER_MS * v_mps)
    return float(np.clip(rpm, RPM_CMD_MIN, RPM_MAX))

# ===== Rejoin tuning =====
POST_OBS_BIAS_STEPS = 18
REJOIN_GAIN = 1.2                 # ligeiramente mais baixo para não “cortar esquina”
FILTER_MARGIN_AFTER_FACTOR = 0.5

# ===== Rejoin sequencial =====
REJOIN_CANDIDATES = 25
REJOIN_STEP = 2

# ===== [ca_ws] Parametros ajustaveis =====
# Cada constante desta lista e tambem um parametro ROS com o mesmo nome. Com o MPC a correr:
#   ros2 param set /mpc_controller V_MAX 1.5            (usar 1.5 / 2.0: o tipo tem de bater)
#   ros2 run rqt_reconfigure rqt_reconfigure             (barras deslizantes)
#   terminal do MPC (tools/sim.sh mpc): p = ver, t = mudar, g = guardar
# Ao arrancar: --ros-args --params-file mpc_python/config/mpc.yaml (+ ~/ca_ws/mpc_params.yaml).
# As alteracoes valem no ciclo seguinte; os marcados com "ref" voltam a calcular o caminho desviado
# dos obstaculos (sem perder o progresso). dt nao e ajustavel (periodo do timer).
# (nome, descricao, minimo, maximo, recalcula_referencia)
TUNABLE = [
    ('V_MAX', 'velocidade maxima [m/s]', 0.2, 4.0, False),
    ('V_MIN', 'velocidade minima dos planos [m/s]', 0.0, 1.0, False),
    ('V_MAX_NOT_ALIGNED', 'velocidade maxima com rumo/erro acima dos limiares [m/s]', 0.1, 4.0, False),
    ('ALIGN_HEADING_THRESH_DEG', 'erro de rumo acima do qual limita a V_MAX_NOT_ALIGNED [graus]', 1.0, 45.0, False),
    ('ALIGN_CTE_THRESH', 'erro lateral acima do qual limita a V_MAX_NOT_ALIGNED [m]', 0.01, 1.0, False),
    ('ACCEL_LIMIT', 'aceleracao maxima do comando [m/s^2]', 0.05, 5.0, False),
    ('DECEL_LIMIT', 'travagem maxima do comando [m/s^2]', 0.1, 10.0, False),
    ('MAX_LAT_ACC', 'aceleracao lateral maxima nas curvas (velocidade de referencia) [m/s^2]', 0.1, 10.0, False),
    ('OMEGA_MAX', 'velocidade angular maxima [rad/s]', 0.1, 5.0, False),
    ('w_cross_track', 'peso: distancia ao caminho', 0.0, 1000.0, False),
    ('w_heading', 'peso: erro de rumo', 0.0, 1000.0, False),
    ('w_control_effort', 'peso: tamanho dos comandos', 0.0, 100.0, False),
    ('w_control_smooth', 'peso: variacao dos comandos', 0.0, 100.0, False),
    ('w_terminal', 'peso: distancia ao caminho no fim do horizonte', 0.0, 1000.0, False),
    ('H_SHORT', 'horizonte normal [passos de dt = 0.3 s]', 2, 30, False),
    ('H_LONG', 'horizonte longo (quando o curto colide) [passos]', 4, 40, False),
    ('MPC_FTOL', 'tolerancia do L-BFGS-B', 1e-6, 0.5, False),
    ('MPC_MAXITER', 'iteracoes maximas do L-BFGS-B', 4, 200, False),
    ('TRACK_HALF_WIDTH', 'distancia ao ponto de referencia acima da qual penaliza 1e9 [m]', 0.1, 3.0, False),
    ('PATH_TUBE_LIMIT', 'desvio maximo ao caminho aceite num plano [m]', 0.1, 3.0, False),
    ('OBSTACLE_SIZE_FROM_Z', 'usar o tamanho de cada obstaculo (pose.position.z = raio) de /obstacles', None, None, True),
    ('CAR_HALF_WIDTH', 'meia largura do carro: raio duro = raio do obstaculo + isto + OBSTACLE_MARGIN [m]', 0.0, 0.5, True),
    ('OBSTACLE_MARGIN', 'folga entre o carro e o obstaculo [m]', 0.0, 1.0, True),
    ('OBSTACLE_RADIUS', 'raio duro dos obstaculos SEM tamanho (distancia ao centro do carro) [m]', 0.05, 2.0, True),
    ('OBSTACLE_SOFT_MARGIN', 'margem do custo suave alem do raio [m]', 0.0, 1.0, False),
    ('W_OBSTACLE_SOFT', 'peso do custo suave dos obstaculos', 0.0, 1e7, False),
    ('OBS_APPEARS_THRESH_FOR_SLOW', 'abranda (0.35 m/s) se o plano passa a raio + isto [m]', 0.0, 1.0, False),
    ('USE_REF_SHIFT', 'desviar a referencia a volta dos obstaculos', None, None, True),
    ('AVOID_MARGIN', 'o caminho desviado passa a raio duro + isto do obstaculo [m]', 0.0, 1.0, True),
    ('AVOID_RAMP', 'comprimento das rampas do desvio [m]', 0.2, 6.0, True),
    ('AVOID_DEFAULT_SIDE', 'obstaculo em cima do caminho: +1 esquerda, -1 direita', -1.0, 1.0, True),
    ('REF_SPEED_SPACING', 'referencia espacada pela velocidade (False = 3 pontos por passo)', None, None, False),
    ('REF_SPACING_LEAD', 'm/s a mais que a ultima velocidade no espacamento da referencia', 0.0, 1.0, False),
    ('END_SLOWDOWN_REMAIN', 'comeca a abrandar a esta distancia do fim [m]', 0.0, 10.0, False),
    ('END_STOP_REMAIN', 'para a esta distancia do fim [m]', 0.0, 5.0, False),
    ('REQUIRE_OBSTACLES_TO_START', 'so arranca depois de receber /obstacles', None, None, False),
]

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.declare_tunables()  # [ca_ws] constantes ajustaveis como parametros ROS

        # Estado
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0; self.velocity = 0.0
        self.have_odom = False

        # Trajetória
        # [ca_ws] nominal_path = caminho recebido (reamostrado); ref_path = o mesmo desviado dos
        # obstaculos (USE_REF_SHIFT), e o que o MPC segue
        self.nominal_path = []; self._obs_key = None
        self.base_path = []; self.ref_path = []
        self.path_headings = []; self.path_curvatures = []; self.path_cumdist = []
        self.path_kdtree = None; self.current_segment = 0
        self._last_path_hash = None
        self.closed_loop = False

        # Progresso
        self.max_index_reached = 0
        self.progress_ratio = 0.0

        # Obstáculos
        self.obstacles = []; self.have_obstacles = False
        self.obstacle_sizes = {}  # [ca_ws] (x, y) -> raio do obstaculo (pose.position.z), se vier
        self.active_obstacles = []

        # Joystick
        self.joy_override = False

        # Horizonte/warm start
        self.H = H_SHORT
        self.control_sequence_v = [0.8] * self.H
        self.control_sequence_gamma = [0.0] * self.H
        self.prev_v_cmd = 0.0

        # Override & rejoin bias
        self.ref_start_override = None
        self.post_obs_bias_count = 0

        # Log throttle
        self._last_log_time = 0.0
        self._log_period = 1.0

        # ROS I/O
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/predicted_trajectories', 10)

        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(Path, '/reference_path', self.path_cb, 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_subscription(PoseArray, '/obstacles', self.obstacles_cb, 10)

        self.create_timer(dt, self.update)

    # ---------- [ca_ws] Parametros ajustaveis ----------
    def declare_tunables(self):
        mod = sys.modules[__name__]
        for name, desc, lo, hi, _ in TUNABLE:
            val = getattr(mod, name)
            d = ParameterDescriptor(description=desc)
            if isinstance(val, bool):
                pass
            elif isinstance(val, int):
                d.integer_range = [IntegerRange(from_value=int(lo), to_value=int(hi), step=1)]
            else:
                val = float(val)
                d.floating_point_range = [FloatingPointRange(from_value=float(lo), to_value=float(hi), step=0.0)]
            # o valor declarado vem do --params-file se existir, senao fica o do codigo
            setattr(mod, name, self.declare_parameter(name, val, d).value)
        self.add_on_set_parameters_callback(self.on_set_tunables)

    def on_set_tunables(self, params):
        mod = sys.modules[__name__]
        spec = {t[0]: t for t in TUNABLE}
        new = {p.name: p.value for p in params if p.name in spec}
        if not new:
            return SetParametersResult(successful=True)
        cur = lambda n: new.get(n, getattr(mod, n))  # noqa: E731
        if cur('V_MIN') >= cur('V_MAX'):
            return SetParametersResult(successful=False, reason='V_MIN tem de ser < V_MAX')
        if cur('H_LONG') < cur('H_SHORT'):
            return SetParametersResult(successful=False, reason='H_LONG tem de ser >= H_SHORT')
        refresh = False
        for name, value in new.items():
            old = getattr(mod, name)
            setattr(mod, name, value)
            refresh |= spec[name][4] and old != value
            self.get_logger().info(f'parametro {name}: {old} -> {value}')
        if refresh and self.nominal_path:
            self.update_reference_geometry()
        return SetParametersResult(successful=True)

    # ---------- Callbacks ----------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.velocity = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.have_odom = True

    def joy_cb(self, msg: Joy):
        self.joy_override = any(button > 0 for button in msg.buttons)

    def obstacles_cb(self, msg: PoseArray):
        first = not self.have_obstacles
        self.obstacles = [(p.position.x, p.position.y) for p in msg.poses]
        self.obstacle_sizes = {(p.position.x, p.position.y): p.position.z for p in msg.poses
                               if p.position.z > 0.0}  # [ca_ws] tamanho (ver OBSTACLE_SIZE_FROM_Z)
        self.have_obstacles = True
        # [ca_ws] Antes reconstruia a cada mensagem (10 Hz). rebuild_path_structures() nao usa os
        # obstaculos, mas repoe max_index_reached/post_obs_bias_count a 0 e relocaliza o carro no
        # ponto mais proximo de TODO o caminho -> num caminho cujo fim passa perto do inicio (Q)
        # o indice saltava para o fim e o MPC parava com "fim de trajetoria" a 9% do percurso.
        # Agora so relocaliza na primeira mensagem (arranque: o MPC espera pelos obstaculos).
        if first and self.base_path:
            self.rebuild_path_structures(self.base_path)
        elif USE_REF_SHIFT and self.base_path and self._obstacles_key() != self._obs_key:
            # [ca_ws] obstaculos mudaram: volta a desviar a referencia (sem mexer no progresso)
            self.update_reference_geometry()

    def _obstacles_key(self):
        return tuple(round(v, 2) for p in self.obstacles for v in (*p, self.obstacle_sizes.get(p, 0.0)))

    def obs_radius(self, o):
        """[ca_ws] Raio duro do obstaculo o = (x, y): distancia minima ao centro do carro."""
        r = self.obstacle_sizes.get(o, 0.0) if OBSTACLE_SIZE_FROM_Z else 0.0
        return r + CAR_HALF_WIDTH + OBSTACLE_MARGIN if r > 0.0 else OBSTACLE_RADIUS

    def _hash_points(self, pts):
        if not pts: return None
        s = ';'.join(f'{x:.3f},{y:.3f}' for (x,y) in pts)
        return hashlib.md5(s.encode('utf-8')).hexdigest()

    def path_cb(self, msg: Path):
        raw = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        new_base = self.resample_path(raw) if len(raw) >= 2 else raw
        new_hash = self._hash_points(new_base)
        if new_hash == self._last_path_hash:
            return
        self._last_path_hash = new_hash
        self.base_path = new_base
        self.rebuild_path_structures(self.base_path)
        self.get_logger().info(f"Trajetória recebida com {len(self.ref_path)} pontos (base {len(self.base_path)})")

    # ---------- Trajetória ----------
    def resample_path(self, points, step=None):
        step = RESAMPLE_STEP if step is None else step  # [ca_ws] valor atual (parametro)
        if len(points) < 2: return points
        points_arr = np.array(points)
        seg = np.diff(points_arr, axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
        total = float(np.sum(seg_len))
        if total <= 0.0: return points
        num_points = max(2, int(total / step))
        cum = np.insert(np.cumsum(seg_len), 0, 0.0)
        targets = np.linspace(0.0, total, num_points)
        out = []
        for d in targets:
            i = int(np.searchsorted(cum, d) - 1)
            i = max(0, min(i, len(seg_len) - 1))
            s = points_arr[i]; e = points_arr[i+1]
            t = (d - cum[i]) / max(seg_len[i], 1e-9)
            p = s + t * (e - s)
            out.append((float(p[0]), float(p[1])))
        return out

    def rebuild_path_structures(self, pts):
        self.nominal_path = list(pts)
        self.closed_loop = False
        if len(self.nominal_path) >= 3:
            d_cl = math.hypot(self.nominal_path[0][0]-self.nominal_path[-1][0],
                              self.nominal_path[0][1]-self.nominal_path[-1][1])
            if d_cl <= LOOP_CLOSE_TOL:
                self.closed_loop = True

        self.update_reference_geometry()

        # KD
        if self.have_odom and self.path_kdtree:
            _, idx = self.path_kdtree.query([self.x, self.y])
            self.current_segment = int(idx)
        else:
            self.current_segment = 0

        # reset progresso / rejoin
        self.max_index_reached = 0
        self.progress_ratio = 0.0
        self.post_obs_bias_count = 0

    def update_reference_geometry(self):
        """[ca_ws] ref_path = nominal_path (desviado dos obstaculos) + rumos, curvaturas, distancia
        acumulada e KD-tree. Nao mexe no progresso (os indices sao os mesmos do caminho nominal)."""
        pts = self.nominal_path
        self._obs_key = self._obstacles_key()
        if USE_REF_SHIFT and self.obstacles and len(pts) >= 3:
            pts = self.shift_path_around_obstacles(pts, self.obstacles)
        self.ref_path = list(pts)

        # headings
        self.path_headings = []
        for i in range(len(self.ref_path) - 1):
            dx = self.ref_path[i+1][0] - self.ref_path[i][0]
            dy = self.ref_path[i+1][1] - self.ref_path[i][1]
            self.path_headings.append(math.atan2(dy, dx))
        self.path_headings.append(self.path_headings[-1] if self.path_headings else 0.0)

        # curvaturas
        self.path_curvatures = [0.0] * len(self.ref_path)
        for i in range(1, len(self.ref_path) - 1):
            x1,y1 = self.ref_path[i-1]; x2,y2 = self.ref_path[i]; x3,y3 = self.ref_path[i+1]
            a = math.hypot(x2-x1, y2-y1); b = math.hypot(x3-x2, y3-y2); c = math.hypot(x3-x1, y3-y1)
            s = 0.5*(a+b+c); area_sq = max(s*(s-a)*(s-b)*(s-c), 0.0)
            if a*b*c > 1e-6 and area_sq > 0.0:
                R = (a*b*c)/(4.0*math.sqrt(area_sq))
                self.path_curvatures[i] = 1.0/max(R,1e-6)

        # cumdist
        self.path_cumdist = [0.0]
        for i in range(1, len(self.ref_path)):
            d = math.hypot(self.ref_path[i][0]-self.ref_path[i-1][0], self.ref_path[i][1]-self.ref_path[i-1][1])
            self.path_cumdist.append(self.path_cumdist[-1] + d)

        # KD
        self.path_kdtree = KDTree(self.ref_path) if self.ref_path else None

    def shift_path_around_obstacles(self, pts, obstacles):
        """[ca_ws] Desvia o caminho para o lado em cada passagem perto de um obstaculo.

        Em cada passagem (o caminho pode passar 2x perto do mesmo obstaculo, ex. o Q), no ponto mais
        proximo mede-se o desvio lateral do obstaculo (lat > 0: a esquerda). O caminho e empurrado
        para o lado contrario ate ficar a C = raio duro + AVOID_MARGIN do obstaculo: deslocamento total
        enquanto o caminho original estaria a < C, e rampa em cosseno de AVOID_RAMP m antes e
        depois. Com varios obstaculos fica, em cada ponto, o maior deslocamento.
        """
        P = np.asarray(pts, dtype=float)
        n = len(P)
        seg_len = np.hypot(*np.diff(P, axis=0).T)
        s = np.concatenate(([0.0], np.cumsum(seg_len)))
        T = np.gradient(P, axis=0)
        T /= np.maximum(np.hypot(T[:, 0], T[:, 1]), 1e-9)[:, None]
        left = np.stack([-T[:, 1], T[:, 0]], axis=1)
        offset = np.zeros(n)
        for ox, oy in obstacles:
            C = self.obs_radius((ox, oy)) + AVOID_MARGIN
            d = np.hypot(P[:, 0] - ox, P[:, 1] - oy)
            near = np.flatnonzero(d < C)
            if near.size == 0:
                continue
            for group in np.split(near, np.flatnonzero(np.diff(near) > 1) + 1):
                ic = int(group[np.argmin(d[group])])
                lat = float((ox - P[ic, 0]) * left[ic, 0] + (oy - P[ic, 1]) * left[ic, 1])
                if abs(lat) > AVOID_CENTER_TOL:
                    side = -math.copysign(1.0, lat)
                else:
                    side = self._free_side(P, s, ic, left[ic], C, (ox, oy), obstacles)
                amount = lat + side * C          # o ponto fica a C do obstaculo, do lado "side"
                hold = math.sqrt(max(C * C - lat * lat, 0.0))
                ds = np.abs(s - s[ic])
                ramp = 0.5 * (1.0 + np.cos(math.pi * np.clip((ds - hold) / AVOID_RAMP, 0.0, 1.0)))
                c = amount * np.where(ds <= hold, 1.0, ramp)
                offset = np.where(np.abs(c) > np.abs(offset), c, offset)
        Q = P + offset[:, None] * left
        return [(float(x), float(y)) for x, y in Q]

    @staticmethod
    def _free_side(P, s, ic, left_ic, C, obstacle, obstacles):
        """[ca_ws] Obstaculo mesmo em cima do caminho: escolhe o lado com mais espaco, isto e, o
        mais longe de outras partes do caminho (ex. o fim do Q que volta a reta inicial) e dos
        outros obstaculos. Empate -> AVOID_DEFAULT_SIDE."""
        far = np.abs(s - s[ic]) > 2.0 * (C + AVOID_RAMP)
        others = [o for o in obstacles if o != obstacle]
        room = {}
        for side in (1.0, -1.0):
            cx, cy = P[ic, 0] + side * C * left_ic[0], P[ic, 1] + side * C * left_ic[1]
            r = float(np.min(np.hypot(P[far, 0] - cx, P[far, 1] - cy))) if far.any() else math.inf
            for ox, oy in others:
                r = min(r, math.hypot(ox - cx, oy - cy))
            room[side] = r
        if abs(room[1.0] - room[-1.0]) < 0.05 or min(room.values()) > 2.0:
            return AVOID_DEFAULT_SIDE
        return 1.0 if room[1.0] > room[-1.0] else -1.0

    # ---------- Obstáculo utils ----------
    def obstacles_relevant_for_points(self, pts, extra=None):
        extra = PATH_FILTER_MARGIN if extra is None else extra  # [ca_ws] valor atual (parametro)
        if not pts or not self.obstacles: return []
        relevant = []
        for (ox, oy) in self.obstacles:
            r = self.obs_radius((ox, oy)) + max(0.0, extra)  # [ca_ws] raio de cada obstaculo
            r2 = r * r
            min_d2 = min((ox - x)**2 + (oy - y)**2 for (x, y) in pts)
            if min_d2 <= r2:
                relevant.append((ox, oy))
        return relevant

    def states_hit_obstacle(self, states, obstacles, appear_extra=0.0):
        if not obstacles: return False
        r2s = [(ox, oy, (self.obs_radius((ox, oy)) + max(0.0, appear_extra)) ** 2) for (ox, oy) in obstacles]  # [ca_ws]
        for (x, y, _, _) in states:
            for (ox, oy, r2) in r2s:
                dx = x - ox; dy = y - oy
                if (dx*dx + dy*dy) < r2:
                    return True
        return False

    def is_ref_idx_unsafe(self, i, obstacles, extra=None):
        extra = PATH_FILTER_MARGIN if extra is None else extra  # [ca_ws] valor atual (parametro)
        if not obstacles: return False
        rx, ry = self.ref_path[i]
        # >>> FIX: usar a lista 'obstacles' passada como argumento <<<
        for (ox, oy) in obstacles:
            r2 = (self.obs_radius((ox, oy)) + extra) ** 2  # [ca_ws] raio de cada obstaculo
            dx = rx - ox; dy = ry - oy
            if (dx*dx + dy*dy) <= r2:
                return True
        return False

    # ---------- Distância ao caminho / tubo ----------
    def nearest_path_dist(self, x, y):
        if not self.path_kdtree: return 0.0
        d, idx = self.path_kdtree.query([x, y])
        return float(d)

    def states_outside_tube(self, states, tube=None):
        """True se algum estado se afasta demasiado do caminho (corta a pista)."""
        # [ca_ws] era tube=PATH_TUBE_LIMIT: o valor ficava fixo quando o modulo era carregado
        t2 = PATH_TUBE_LIMIT if tube is None else tube
        for (x, y, _, _) in states:
            if self.nearest_path_dist(x, y) > t2:
                return True
        return False

    # ---------- Ordenação: menor índice válido à frente ----------
    def find_closest_point_ordered(self, x, y):
        if not self.ref_path: return (0.0, 0.0), 0, 0.0
        HEADING_GATE_RAD = math.radians(60.0)
        FORWARD_DOT_MIN = -0.05
        n = len(self.ref_path); i0 = self.current_segment
        i1 = min(n - 1, i0 + WINDOW_AHEAD)
        if i1 <= i0: i1 = min(n - 1, i0 + 1)
        fx, fy = math.cos(self.yaw), math.sin(self.yaw)
        chosen = i0
        for i in range(i0, i1 + 1):
            rx, ry = self.ref_path[i]
            if i < len(self.path_headings):
                if abs(self.normalize_angle(self.path_headings[i] - self.yaw)) > HEADING_GATE_RAD:
                    continue
            vx, vy = (rx - x), (ry - y)
            if (vx*fx + vy*fy) < FORWARD_DOT_MIN:
                continue
            chosen = i
            break
        if chosen > self.current_segment:
            self.current_segment = chosen
        if self.closed_loop and self.current_segment >= len(self.ref_path) - 5:
            d0 = math.hypot(x - self.ref_path[0][0], y - self.ref_path[0][1])
            if d0 < WRAP_NEAR_START_DIST:
                self.current_segment = 0
        idx = self.current_segment
        return self.ref_path[idx], idx, (self.path_headings[idx] if idx < len(self.path_headings) else 0.0)

    # ---------- Bloco proibido à frente ----------
    def find_unsafe_block(self, start_idx, obstacles, max_scan=80):
        if not obstacles: return None
        n = len(self.ref_path)
        jmax = min(n - 1, start_idx + max_scan)
        in_block = False; first = None; last = None
        for j in range(start_idx, jmax + 1):
            unsafe = self.is_ref_idx_unsafe(j, obstacles)
            if unsafe:
                if not in_block: first = j
                in_block = True; last = j
            elif in_block:
                break
        if in_block:
            return first, last
        return None

    def first_safe_after(self, idx, obstacles, max_scan=80):
        n = len(self.ref_path)
        jmax = min(n - 1, idx + max_scan)
        for j in range(idx + 1, jmax + 1):
            if not self.is_ref_idx_unsafe(j, obstacles):
                return j
        return idx

    # ---------- Dinâmica / custo ----------
    def simulate_vehicle_dynamics(self, x0, y0, yaw0, vs, gs, H=None):
        H = H if H is not None else self.H
        states = []
        x, y, yaw = x0, y0, yaw0
        for k in range(H):
            v = float(vs[k]); g = float(gs[k])
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += (v / WHEEL_BASE) * math.tan(g) * dt
            yaw = self.normalize_angle(yaw)
            states.append((x, y, yaw, v))
        return states

    def obstacle_extra(self, margin):
        """[ca_ws] margem para escolher os obstaculos que entram no custo/validacao (ver OBSTACLE_LOOKAROUND)."""
        return margin + (OBSTACLE_LOOKAROUND if USE_REF_SHIFT else 0.0)

    def mpc_cost_function(self, u):
        # [ca_ws] mantido por compatibilidade; o run_optimization usa build_cost (referencia 1x)
        return self.build_cost(self.H)(u)

    def build_cost(self, H):
        """[ca_ws] Devolve a funcao de custo do MPC para o estado atual.

        O custo e exatamente o de antes (mesmos termos e pesos), mas a referencia local (ponto de
        partida, pontos, rumos, curvaturas, obstaculos relevantes) e calculada UMA vez por
        otimizacao em vez de em cada avaliacao (o L-BFGS-B com diferencas finitas avalia o custo
        2H+1 vezes por iteracao), e o rollout e os termos sao feitos em Python simples (para H = 6/16 o numpy e mais lento).
        """
        if self.ref_start_override is not None:
            start_idx = int(self.ref_start_override)
        else:
            _, start_idx, _ = self.find_closest_point_ordered(self.x, self.y)
        base_pts, base_heads, base_curv = self.get_local_reference_base(start_idx, H=H)
        margin = PATH_FILTER_MARGIN * (FILTER_MARGIN_AFTER_FACTOR if self.post_obs_bias_count > 0 else 1.0)
        relevant_obs = self.obstacles_relevant_for_points(base_pts, extra=self.obstacle_extra(margin))
        self.active_obstacles = relevant_obs
        if relevant_obs:
            ref_pts, ref_heads, ref_curv = self.filter_ref_points_near_obstacles(
                base_pts, base_heads, base_curv, relevant_obs, margin=margin
            )
        else:
            ref_pts, ref_heads, ref_curv = base_pts, base_heads, base_curv
        if not ref_pts:
            return lambda u: 1e8
        rejoin_factor = (1.0 + REJOIN_GAIN) if self.post_obs_bias_count > 0 else 1.0
        big = 1e9; eps = 1e-6
        K = min(H, len(ref_pts))                       # zip(states, ref...) do original
        ref = []
        for k in range(K):
            c_k = abs(ref_curv[k])
            v_c = V_MAX if c_k < eps else math.sqrt(max(MAX_LAT_ACC / (c_k + eps), 0.0))
            ref.append((ref_pts[k][0], ref_pts[k][1], ref_heads[k], max(V_MIN, min(V_MAX, v_c)),
                        rejoin_factor * (k / max(1, H)) * w_terminal))
        # [ca_ws] (x, y, raio do custo suave) de cada obstaculo
        obs = [(ox, oy, self.obs_radius((ox, oy)) + OBSTACLE_SOFT_MARGIN) for ox, oy in relevant_obs]
        x0, y0, yaw0 = self.x, self.y, self.yaw
        cos, sin, tan, hypot, pi = math.cos, math.sin, math.tan, math.hypot, math.pi
        two_pi = 2.0 * pi; half_pi = pi / 2.0

        def cost(u):
            u = u.tolist() if hasattr(u, 'tolist') else list(u)
            cv = u[:H]; cg = u[H:]
            if min(cv) < V_MIN or max(cv) > V_MAX: return 1e8
            if max(abs(g) for g in cg) > GAMMA_MAX: return 1e8
            x, y, yaw = x0, y0, yaw0
            total = 0.0
            for k in range(K):
                v = cv[k]; rx, ry, rh, v_c, w_k = ref[k]
                # rollout = simulate_vehicle_dynamics
                x += v * cos(yaw) * dt
                y += v * sin(yaw) * dt
                yaw += (v / WHEEL_BASE) * tan(cg[k]) * dt
                # [ca_ws] obstaculo: custo suave W*(R_SOFT - d)^2 (era: patamar 1e9 sem gradiente)
                for ox, oy, r_soft in obs:
                    d_obs = hypot(x - ox, y - oy)
                    if d_obs < r_soft:
                        total += W_OBSTACLE_SOFT * (r_soft - d_obs) ** 2
                dist = hypot(x - rx, y - ry)
                total += w_cross_track * dist * dist
                if dist > TRACK_HALF_WIDTH: total += big * (dist - TRACK_HALF_WIDTH + 1.0)
                hd = abs((yaw - rh + pi) % two_pi - pi)
                total += w_heading * hd
                total += w_k * dist
                heading_scaler = max(0.2, 1.0 - 0.95 * (hd / half_pi))
                cte_scaler = max(0.25, 1.0 - 1.5 * dist)
                v_ref = max(V_MIN, min(V_MAX, v_c * heading_scaler * cte_scaler))
                total += 3.5 * (v - v_ref) ** 2
                ex = v - v_ref * (1.0 + V_SAFETY_MARGIN)
                if ex > 0.0: total += 300.0 * ex * ex
            total += w_control_effort * (sum(v * v for v in cv) + sum(g * g for g in cg))
            sm = 0.0
            for k in range(H - 1):
                dv = cv[k + 1] - cv[k]; dg = cg[k + 1] - cg[k]
                sm += dv * dv + dg * dg
            total += w_control_smooth * sm
            return total

        return cost

    # ---------- Local ref helpers ----------
    def get_local_reference_base(self, start_idx, H=None):
        if not self.ref_path: return [], [], []
        H = H if H is not None else self.H
        if REF_SPEED_SPACING:
            return self.speed_spaced_reference(start_idx, H)
        end_idx = min(len(self.ref_path), start_idx + H * 3)
        pts = self.ref_path[start_idx:end_idx]
        heads = self.path_headings[start_idx:end_idx]
        curv = self.path_curvatures[start_idx:end_idx] if self.path_curvatures else [0.0] * len(pts)
        step = max(1, len(pts) // H)
        base_pts, base_heads, base_curv = [], [], []
        for i in range(0, len(pts), step):
            base_pts.append(pts[i]); base_heads.append(heads[i]); base_curv.append(curv[i])
            if len(base_pts) >= H: break
        while len(base_pts) < H:
            base_pts.append(self.ref_path[-1])
            base_heads.append(self.path_headings[-1] if self.path_headings else 0.0)
            base_curv.append(self.path_curvatures[-1] if self.path_curvatures else 0.0)
        return base_pts[:H], base_heads[:H], base_curv[:H]

    def speed_spaced_reference(self, start_idx, H):
        """[ca_ws] H pontos ao longo do caminho a partir de start_idx, o ponto k a (k+1)*v*dt
        (interpolados; rumo e curvatura do segmento). Ver REF_SPEED_SPACING."""
        v_sp = max(REF_SPACING_V_MIN, min(V_MAX, float(self.prev_v_cmd) + REF_SPACING_LEAD))
        step = v_sp * dt
        cum = self.path_cumdist; path = self.ref_path; n = len(path)
        start_idx = int(max(0, min(start_idx, n - 1)))
        s0 = cum[start_idx]
        pts, heads, curv = [], [], []
        j = start_idx
        for k in range(H):
            s = s0 + (k + 1) * step
            while j < n - 1 and cum[j + 1] < s:
                j += 1
            if j >= n - 1:
                pts.append(path[-1]); heads.append(self.path_headings[-1])
                curv.append(self.path_curvatures[-1] if self.path_curvatures else 0.0)
                continue
            seg = cum[j + 1] - cum[j]
            t = 0.0 if seg <= 1e-9 else min(1.0, max(0.0, (s - cum[j]) / seg))
            (x1, y1), (x2, y2) = path[j], path[j + 1]
            pts.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
            heads.append(self.path_headings[j])
            c1 = self.path_curvatures[j] if self.path_curvatures else 0.0
            c2 = self.path_curvatures[j + 1] if self.path_curvatures else 0.0
            curv.append(max(c1, c2))
        return pts, heads, curv

    def filter_ref_points_near_obstacles(self, pts, heads, curv, obstacles, margin=None):
        margin = PATH_FILTER_MARGIN if margin is None else margin  # [ca_ws] valor atual (parametro)
        if not obstacles: return pts, heads, curv
        r2s = [(ox, oy, (self.obs_radius((ox, oy)) + margin) ** 2) for (ox, oy) in obstacles]  # [ca_ws]
        f_pts, f_heads, f_curv = [], [], []
        for (p, h, c) in zip(pts, heads, curv):
            rx, ry = p
            close = False
            for (ox, oy, r2) in r2s:
                dx = rx - ox; dy = ry - oy
                if (dx*dx + dy*dy) <= r2:
                    close = True; break
            if not close:
                f_pts.append(p); f_heads.append(h); f_curv.append(c)
        if not f_pts:
            return pts, heads, curv
        while len(f_pts) < len(pts):
            f_pts.append(f_pts[-1]); f_heads.append(f_heads[-1]); f_curv.append(f_curv[-1])
        return f_pts[:len(pts)], f_heads[:len(pts)], f_curv[:len(pts)]

    # ---------- Otimização ----------
    def run_optimization(self, H, warm_v, warm_g):
        self.H = H
        v0 = list(warm_v); g0 = list(warm_g)
        if len(v0) < H: v0 += [v0[-1] if v0 else 0.8] * (H - len(v0)); g0 += [g0[-1] if g0 else 0.0] * (H - len(g0))
        elif len(v0) > H: v0 = v0[:H]; g0 = g0[:H]
        # warm shift + clamp dentro dos bounds
        v0 = v0[1:] + [v0[-1]]; g0 = g0[1:] + [g0[-1]]
        epsb = 1e-9
        v0 = [float(np.clip(v, V_MIN + epsb, V_MAX - epsb)) for v in v0]
        g0 = [float(np.clip(g, -GAMMA_MAX + epsb, GAMMA_MAX - epsb)) for g in g0]
        init = v0 + g0
        bounds = [(V_MIN, V_MAX)] * H + [(-GAMMA_MAX, GAMMA_MAX)] * H
        # [ca_ws] era minimize(self.mpc_cost_function, ...): recalculava a referencia em cada avaliacao
        cost = self.build_cost(H)
        result = minimize(cost, init, method='L-BFGS-B', bounds=bounds,
                          options={'maxiter': int(MPC_MAXITER), 'ftol': MPC_FTOL})
        # [ca_ws] success=False tambem quando chega as 32 iteracoes; o resultado continua a ser
        # melhor do que o warm start (que nao foi otimizado e as vezes vai contra o obstaculo).
        # Antes deitava-se fora e usava-se o warm start -> "STOP: sem trajetoria livre".
        if not result.success and float(result.fun) < cost(init):
            result.success = True
        if result.success:
            v_seq = [float(np.clip(v, V_MIN, V_MAX)) for v in result.x[:H]]
            g_seq = [float(np.clip(g, -GAMMA_MAX, GAMMA_MAX)) for g in result.x[H:]]
        else:
            v_seq, g_seq = v0, g0
        states = self.simulate_vehicle_dynamics(self.x, self.y, self.yaw, v_seq, g_seq, H=H)
        return bool(result.success), v_seq, g_seq, states

    # ---------- Visualização ----------
    def publish_markers(self, predicted_states):
        ma = MarkerArray()
        clear = Marker(); clear.header.frame_id = "map"; clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        # ref laranja→castanho
        ref_down = self.ref_path[::3] if self.ref_path else []
        n = len(ref_down)
        start_rgb = (1.0, 0.55, 0.0); end_rgb = (0.40, 0.26, 0.13)
        for i, (x, y) in enumerate(ref_down):
            t = 0.0 if n <= 1 else i/(n-1)
            r = (1.0-t)*start_rgb[0] + t*end_rgb[0]
            g = (1.0-t)*start_rgb[1] + t*end_rgb[1]
            b = (1.0-t)*start_rgb[2] + t*end_rgb[2]
            m = Marker(); m.header.frame_id = "map"; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = Point(x=float(x), y=float(y), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b
            ma.markers.append(m)
        # janela azul
        _, current_idx, _ = self.find_closest_point_ordered(self.x, self.y)
        ref_points, _, _ = self.get_local_reference_base(current_idx, H=self.H)
        for i, (x, y) in enumerate(ref_points):
            m = Marker(); m.header.frame_id = "map"; m.id = 10000+i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position = Point(x=float(x), y=float(y), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.a = 1.0; m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0
            ma.markers.append(m)
        # previsão
        for i, (x, y, _, _) in enumerate(predicted_states):
            coll = self.states_hit_obstacle([(x, y, 0.0, 0.0)], self.active_obstacles, appear_extra=0.0)
            m = Marker(); m.header.frame_id = "map"; m.id = 20000+i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = Point(x=float(x), y=float(y), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.a = 1.0
            if coll: m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            else:    m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            ma.markers.append(m)
        # obstáculos
        for i, (ox, oy) in enumerate(self.obstacles):
            m = Marker(); m.header.frame_id = "map"; m.id = 40000+i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position = Point(x=float(ox), y=float(oy), z=0.0)
            m.scale.x, m.scale.y, m.scale.z = OBSTACLE_RECT_SIZE
            m.color.a = 0.95; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0
            ma.markers.append(m)
        # pose atual
        m = Marker(); m.header.frame_id = "map"; m.id = 30000
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position = Point(x=float(self.x), y=float(self.y), z=0.0)
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.a = 1.0; m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0
        ma.markers.append(m)
        self.marker_pub.publish(ma)

    # ---------- Planeamento auxiliar ----------
    def plan_from_index(self, start_idx, warm_v, warm_g, H=None):
        H = H_LONG if H is None else H  # [ca_ws] valor atual (parametro)
        """Tenta planear a partir de start_idx (override) e valida: sem colisão e dentro do 'tubo'."""
        self.ref_start_override = int(start_idx)
        ok, v, g, states = self.run_optimization(H, warm_v, warm_g)
        base_pts, _, _ = self.get_local_reference_base(start_idx, H=H)
        margin = PATH_FILTER_MARGIN * (FILTER_MARGIN_AFTER_FACTOR if self.post_obs_bias_count > 0 else 1.0)
        rel = self.obstacles_relevant_for_points(base_pts, extra=self.obstacle_extra(margin))
        self.ref_start_override = None
        # validações
        if self.states_hit_obstacle(states, rel, appear_extra=0.0):
            return False, v, g, states, rel
        if self.states_outside_tube(states, tube=PATH_TUBE_LIMIT):
            return False, v, g, states, rel
        return True, v, g, states, rel

    # ---------- Ciclo ----------
    def update(self):
        if not self.have_odom or not self.ref_path or self.joy_override: return
        if REQUIRE_OBSTACLES_TO_START and not self.have_obstacles: return

        (_, idx_now, _) = self.find_closest_point_ordered(self.x, self.y)

        # progresso
        if idx_now > self.max_index_reached:
            self.max_index_reached = idx_now
        total_idx = max(1, len(self.ref_path) - 1)
        self.progress_ratio = float(self.max_index_reached) / float(total_idx)

        # obstáculos relevantes ao curto
        base_pts_short, _, _ = self.get_local_reference_base(idx_now, H=H_SHORT)
        relevant_obs_now = self.obstacles_relevant_for_points(
            base_pts_short, extra=self.obstacle_extra(PATH_FILTER_MARGIN))

        # plano curto default
        succ_s, v_s, g_s, states_s = self.run_optimization(H_SHORT, self.control_sequence_v, self.control_sequence_gamma)
        v_plan, g_plan, states_plan = v_s, g_s, states_s
        used_override = False

        # bloco proibido mesmo em cima → rejoin sequencial
        unsafe_block = (self.find_unsafe_block(idx_now, relevant_obs_now, max_scan=80)
                        if (USE_REJOIN and relevant_obs_now) else None)
        if unsafe_block is not None:
            _, last = unsafe_block
            first_safe = self.first_safe_after(last, relevant_obs_now, max_scan=80)
            success_any = False
            for k in range(REJOIN_CANDIDATES):
                cand = min(len(self.ref_path) - 1, first_safe + k * REJOIN_STEP)
                ok, v_l, g_l, states_l, rel_cand = self.plan_from_index(cand, self.control_sequence_v, self.control_sequence_gamma, H=H_LONG)
                if ok:
                    v_plan, g_plan, states_plan = v_l, g_l, states_l
                    self.active_obstacles = rel_cand
                    used_override = True
                    success_any = True
                    break
            if not success_any:
                self.publish_stop(); self.publish_markers(states_plan)
                self.throttled_log("STOP: sem trajetória livre após obstáculo (todas as tentativas falharam).")
                return
        else:
            # se o curto colide, tenta longo; caso contrário fica como está
            if self.states_hit_obstacle(states_s, relevant_obs_now, appear_extra=0.0):
                succ_l, v_l, g_l, states_l = self.run_optimization(H_LONG, v_s, g_s)
                if not self.states_hit_obstacle(states_l, relevant_obs_now, appear_extra=0.0) and not self.states_outside_tube(states_l):
                    v_plan, g_plan, states_plan = v_l, g_l, states_l
                else:
                    self.publish_stop(); self.publish_markers(states_l)
                    self.throttled_log("STOP: sem trajetória livre (curto e longo).")
                    return
            self.active_obstacles = relevant_obs_now

        # pós-obstáculo: viés de rejoin
        if used_override:
            self.post_obs_bias_count = POST_OBS_BIAS_STEPS
        else:
            if self.post_obs_bias_count > 0:
                self.post_obs_bias_count -= 1

        # receding horizon curto
        self.H = H_SHORT
        self.control_sequence_v = (v_plan + [v_plan[-1]])[:H_SHORT]
        self.control_sequence_gamma = (g_plan + [g_plan[-1]])[:H_SHORT]

        (ref_x, ref_y), idx, ref_heading = self.find_closest_point_ordered(self.x, self.y)
        cte = math.hypot(self.x - ref_x, self.y - ref_y)
        heading_diff = abs(self.normalize_angle(self.yaw - ref_heading))

        # comando base
        v_cmd = float(np.clip(self.control_sequence_v[0], V_MIN, V_MAX))
        gamma0 = float(self.control_sequence_gamma[0])

        # caps
        aligned_heading = heading_diff <= math.radians(ALIGN_HEADING_THRESH_DEG)
        aligned_cte = cte <= ALIGN_CTE_THRESH
        v_cap_align = V_MAX if (aligned_heading and aligned_cte) else V_MAX_NOT_ALIGNED

        max_gamma = max(abs(float(g)) for g in self.control_sequence_gamma[:3])
        tan_g = max(1e-3, abs(math.tan(max_gamma)))
        v_cap_omega = OMEGA_MAX * WHEEL_BASE / tan_g

        curv_ahead_max = 0.0
        if self.path_curvatures:
            j0 = idx; j1 = min(len(self.path_curvatures), idx + 12)
            curv_ahead_max = max(abs(c) for c in self.path_curvatures[j0:j1]) if j1 > j0 else 0.0
        eps = 1e-6
        v_cap_curv = V_MAX if curv_ahead_max < eps else max(V_MIN, min(V_MAX, math.sqrt(max(MAX_LAT_ACC/(abs(curv_ahead_max)+eps), 0.0))))

        # fim com progresso
        v_cap_end = V_MAX
        if not self.closed_loop:
            remain = self.remaining_distance(idx)
            if self.progress_ratio >= PROGRESS_SLOWDOWN_MIN and remain < END_SLOWDOWN_REMAIN:
                v_cap_end = max(0.0, min(V_MAX_NOT_ALIGNED, (remain / END_SLOWDOWN_REMAIN) * V_MAX_NOT_ALIGNED))
            dist_to_last = math.hypot(self.x - self.ref_path[-1][0], self.y - self.ref_path[-1][1])
            if (self.progress_ratio >= PROGRESS_STOP_MIN) and ((remain < END_STOP_REMAIN) or (dist_to_last < END_FORCE_STOP_RADIUS)):
                self.publish_stop(); self.publish_markers(states_plan)
                self.throttled_log(f"STOP: fim de trajetória (progress={self.progress_ratio:.2f})."); return

        # abrandar se plano passa muito perto dos obstáculos ativos
        danger_pred = self.states_hit_obstacle(states_plan, self.active_obstacles, appear_extra=OBS_APPEARS_THRESH_FOR_SLOW)
        v_cap_danger = 0.35 if danger_pred else V_MAX

        v_cmd = min(v_cmd, v_cap_align, v_cap_omega, v_cap_curv, v_cap_end, v_cap_danger)

        # limitar variação
        dv = v_cmd - self.prev_v_cmd
        max_up = ACCEL_LIMIT * dt; max_dn = DECEL_LIMIT * dt
        if dv > max_up: v_cmd = self.prev_v_cmd + max_up
        elif dv < -max_dn: v_cmd = self.prev_v_cmd - max_dn
        self.prev_v_cmd = v_cmd

        # hard-constraint: >=3000 RPM quando >0
        rpm_cmd = v_to_rpm(v_cmd)
        if rpm_cmd > 0.0 and rpm_cmd < RPM_CMD_MIN:
            rpm_cmd = RPM_CMD_MIN
        self.speed_pub.publish(Float64(data=rpm_cmd))

        gamma = -float(gamma0)
        servo_pos = float(np.clip(SERVO_CENTER + (gamma / GAMMA_MAX) * (SERVO_MAX - SERVO_CENTER),
                                  SERVO_MIN, SERVO_MAX))
        self.steer_pub.publish(Float64(data=servo_pos))

        self.publish_markers(states_plan)

        self.throttled_log(
            f"idx={idx} | prog={self.progress_ratio:.2f} | CTE={cte:.2f}m | head={math.degrees(heading_diff):.1f}° | "
            f"v={v_cmd:.2f} m/s | rpm={rpm_cmd:.0f} | active_obs={len(self.active_obstacles)} | rejoin={self.post_obs_bias_count}"
        )

    # ---------- Helpers ----------
    def normalize_angle(self, a): return (a + math.pi) % (2 * math.pi) - math.pi
    def remaining_distance(self, idx):
        if not self.path_cumdist: return float('inf')
        total = self.path_cumdist[-1]
        idx = int(max(0, min(idx, len(self.path_cumdist) - 1)))
        return max(0.0, total - self.path_cumdist[idx])
    def publish_stop(self):
        self.prev_v_cmd = 0.0
        self.speed_pub.publish(Float64(data=0.0))  # STOP = 0 RPM (única exceção)
        self.steer_pub.publish(Float64(data=SERVO_CENTER))
    def throttled_log(self, s):
        now = time.time()
        if (now - self._last_log_time) >= self._log_period:
            self._last_log_time = now; self.get_logger().info(s)

def main(args=None):
    rclpy.init(args=args)
    controller = MPCController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
