"""Corre o MPC verdadeiro (mpc_python.mpc_controller) em circuito fechado com o modelo
bicicleta, sem ROS a correr: deterministico e independente do CPU.

Precisa do workspace carregado (source install/setup.bash). Exemplos:
  python3 tools/mpc_offline.py ""                        # sem obstaculos
  python3 tools/mpc_offline.py "8.1,2.16"                 # obstaculo 0.4 m por fora da curva grande
  python3 tools/mpc_offline.py "2.5,0.0" --verbose       # obstaculo em cima da reta, passo a passo
  python3 tools/mpc_offline.py "8.1,2.16" --patch USE_REJOIN=1 --gate 8   # modo "contornar" antigo
  python3 tools/mpc_offline.py "8.1,2.16" --patch W_OBSTACLE_SOFT=1e5     # mudar uma constante
Quando o MPC para por falta de trajetoria livre, mostra porque e que cada plano foi rejeitado.
"""
import argparse
import math
import random
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray

import mpc_python.mpc_controller as M
from pickup_description.q_path_publisher import QPathPublisher

ap = argparse.ArgumentParser()
ap.add_argument('obstacles', help='x1,y1,x2,y2,... ou "" ')
ap.add_argument('--patch', action='append', default=[], help='constante do MPC, ex. PATH_TUBE_LIMIT=1.0')
ap.add_argument('--steps', type=int, default=140)
ap.add_argument('--verbose', action='store_true')
ap.add_argument('--gate', type=int, default=0,
                help='>0: so entra no modo contornar se o bloco inseguro comecar a <= N indices do carro')
ap.add_argument('--latency', action='store_true',
                help='o comando novo so e aplicado depois do tempo que o update() demorou (como no ROS)')
ap.add_argument('--seed', type=int, default=-1,
                help='>=0: perturba ao acaso a pose inicial (+-3 cm, +-2 graus) e os obstaculos (+-3 cm)')
ap.add_argument('--ftol', type=float, default=0.0,
                help='>0: troca o ftol do L-BFGS-B (o MPC usa 0.1)')
ap.add_argument('--refstep', type=int, default=0,
                help='>0: pontos do caminho por passo do horizonte (o original usa 3 = 0.45 m por 0.3 s)')
a = ap.parse_args()
for p in a.patch:
    k, v = p.split('=')
    old = getattr(M, k)
    if isinstance(old, bool):
        setattr(M, k, v.strip().lower() in ('1', 'true', 'yes', 'sim'))
    elif isinstance(old, int):
        setattr(M, k, int(float(v)))
    else:
        setattr(M, k, float(v))

obs = [float(v) for v in a.obstacles.split(',') if v.strip()]
obs = list(zip(obs[0::2], obs[1::2]))
rng = random.Random(a.seed) if a.seed >= 0 else None
if rng:
    obs = [(ox + rng.uniform(-0.03, 0.03), oy + rng.uniform(-0.03, 0.03)) for ox, oy in obs]
if a.ftol > 0:
    _minimize = M.minimize

    def minimize_ftol(*args, **kw):
        kw.setdefault('options', {})['ftol'] = a.ftol
        return _minimize(*args, **kw)

    M.minimize = minimize_ftol

rclpy.init()
q = QPathPublisher()
cap = {}
q.path_pub.publish = lambda m: cap.__setitem__('path', m)
q.marker_pub.publish = lambda m: None
q.on_timer()
path = [(ps.pose.position.x, ps.pose.position.y) for ps in cap['path'].poses]

c = M.MPCController()
cmd = {'rpm': 0.0, 'servo': M.SERVO_CENTER}
c.speed_pub.publish = lambda m: cmd.__setitem__('rpm', m.data)
c.steer_pub.publish = lambda m: cmd.__setitem__('servo', m.data)
c.marker_pub.publish = lambda m: None
logs = []
c.throttled_log = lambda s: logs.append(s)

diag = []
orig_plan = c.plan_from_index


def plan_wrapped(start_idx, wv, wg, H=M.H_LONG):
    ok, v, g, st, rel = orig_plan(start_idx, wv, wg, H=H)
    hit = c.states_hit_obstacle(st, rel)
    dev = max(c.nearest_path_dist(x, y) for x, y, _, _ in st)
    gap = math.hypot(st[0][0] - c.ref_path[start_idx][0], st[0][1] - c.ref_path[start_idx][1])
    diag.append(dict(cand=start_idx, ok=ok, hit=hit, tube_dev=dev, gap0=gap))
    return ok, v, g, st, rel


c.plan_from_index = plan_wrapped

opt_log = []
orig_opt = c.run_optimization


def opt_wrapped(H, wv, wg):
    ok, v, g, st = orig_opt(H, wv, wg)
    extra = c.obstacle_extra(M.PATH_FILTER_MARGIN) if hasattr(c, 'obstacle_extra') else M.PATH_FILTER_MARGIN
    ob = c.obstacles_relevant_for_points(c.get_local_reference_base(c.current_segment, H=M.H_SHORT)[0], extra=extra)
    dmin = min((math.hypot(x - ox, y - oy) for x, y, _, _ in st for ox, oy in ob), default=float('inf'))
    dev = max(c.nearest_path_dist(x, y) for x, y, _, _ in st)
    opt_log.append(dict(H=H, ok=ok, dmin=dmin, dev=dev))
    return ok, v, g, st


c.run_optimization = opt_wrapped

if a.gate > 0:
    orig_block = c.find_unsafe_block

    def gated_block(start_idx, obstacles, max_scan=80):
        b = orig_block(start_idx, obstacles, max_scan=max_scan)
        if b is not None and (a.gate >= 999 or b[0] - start_idx > a.gate):
            return None
        return b

    c.find_unsafe_block = gated_block

if a.refstep > 0:
    def ref_base(start_idx, H=None):
        H = H if H is not None else c.H
        n = len(c.ref_path)
        idxs = [min(n - 1, start_idx + k * a.refstep) for k in range(H)]
        return ([c.ref_path[i] for i in idxs], [c.path_headings[i] for i in idxs],
                [c.path_curvatures[i] for i in idxs])

    c.get_local_reference_base = ref_base

# estado inicial = inicio do caminho (como o kinematic_sim)
x, y = path[0]
yaw = math.atan2(path[5][1] - y, path[5][0] - x)
if rng:
    y += rng.uniform(-0.03, 0.03)
    yaw += math.radians(rng.uniform(-2.0, 2.0))
v = delta = 0.0
c.x, c.y, c.yaw, c.velocity, c.have_odom = x, y, yaw, 0.0, True
c.path_cb(cap['path'])
pa = PoseArray()
for ox, oy in (obs or [(50.0, 50.0)]):
    p = Pose()
    p.position.x, p.position.y = ox, oy
    pa.poses.append(p)
c.obstacles_cb(pa)

SPAN = M.SERVO_MAX - M.SERVO_CENTER
min_obs = [float('inf')] * len(obs)
stops = 0
dist = 0.0
t = 0.0
cte_max = 0.0
lat_list = []
prev = dict(cmd)
for step in range(a.steps):
    diag.clear()
    logs.clear()
    opt_log.clear()
    c.x, c.y, c.yaw, c.velocity = x, y, yaw, v
    prev = dict(cmd)
    t0 = time.perf_counter()
    c.update()
    lat = time.perf_counter() - t0
    lat_list.append(lat)
    msg = logs[-1] if logs else ''
    if a.verbose or msg.startswith('STOP'):
        print(f't={t:5.1f} x={x:5.2f} y={y:5.2f} yaw={math.degrees(yaw):6.1f} v={v:.2f} idx={c.current_segment:3d} | {msg[:110]}')
    if msg.startswith('STOP: sem') and stops == 0:
        for o in opt_log:
            print(f"   plano H={o['H']:2d}: otimizador_ok={o['ok']} dist_min_obstaculo={o['dmin']:.2f}m "
                  f"(raio {M.OBSTACLE_RADIUS}) desvio_max_ao_caminho={o['dev']:.2f}m (tubo {M.PATH_TUBE_LIMIT})")
    if msg.startswith('STOP: sem trajetória livre após') and stops == 0:
        block = c.find_unsafe_block(c.current_segment, c.obstacles_relevant_for_points(
            c.get_local_reference_base(c.current_segment, H=M.H_SHORT)[0]))
        print(f'   bloco inseguro (indices do caminho): {block}, carro no indice {c.current_segment}')
        for d in diag[:6]:
            print(f"   candidato {d['cand']:3d}: ok={d['ok']} colide={d['hit']} "
                  f"desvio_max_ao_caminho={d['tube_dev']:.2f}m (tubo {M.PATH_TUBE_LIMIT}) "
                  f"ref_inicial_a={d['gap0']:.2f}m do 1o estado")
        if len(diag) > 6:
            devs = [d['tube_dev'] for d in diag]
            print(f'   ... {len(diag)} candidatos, colidem={sum(d["hit"] for d in diag)}, '
                  f'fora_do_tubo={sum(d["tube_dev"] > M.PATH_TUBE_LIMIT for d in diag)}, '
                  f'desvio min/max={min(devs):.2f}/{max(devs):.2f}m')
    stops = stops + 1 if (msg.startswith('STOP') and v < 0.05) else 0
    if stops >= 4:
        break
    # carro: modelo bicicleta + atrasos (igual ao kinematic_sim com o perfil do MPC)
    for i in range(15):  # 0.3 s a 50 Hz
        h = 0.02
        # com --latency, ate o update() acabar o carro ainda usa o comando anterior
        use = prev if (a.latency and i * h < lat) else cmd
        v_cmd = use['rpm'] / 7000.0
        ratio = max(-1.0, min(1.0, (use['servo'] - M.SERVO_CENTER) / SPAN))
        d_cmd = -ratio * math.pi / 8.0
        v += h / (0.2 + h) * (v_cmd - v)
        delta += h / (0.08 + h) * (d_cmd - delta)
        nx = x + v * math.cos(yaw) * h
        ny = y + v * math.sin(yaw) * h
        dist += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        yaw = math.atan2(math.sin(yaw + v * math.tan(delta) / 0.335 * h),
                         math.cos(yaw + v * math.tan(delta) / 0.335 * h))
        for j, (ox, oy) in enumerate(obs):
            min_obs[j] = min(min_obs[j], math.hypot(x - ox, y - oy))
        cte_max = max(cte_max, c.nearest_path_dist(x, y))
    t += 0.3

prog = c.max_index_reached / max(1, len(c.ref_path) - 1)
end = math.hypot(x - path[-1][0], y - path[-1][1])
print(f'FIM t={t:.1f}s andou={dist:.2f}m progresso={100 * prog:.0f}% dist_ao_fim={end:.2f}m '
      f'cte_max~{cte_max:.2f}m dist_min_obstaculos={[round(m, 2) for m in min_obs]} '
      f'update(): media={1000 * sum(lat_list) / len(lat_list):.0f}ms max={1000 * max(lat_list):.0f}ms')
