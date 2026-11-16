"""[ca_ws] Testes das alteracoes ao MPC (custo rapido e desvio da referencia a volta de obstaculos).

Correm sem ROS a funcionar de verdade (so rclpy.init para criar o no): os publishers sao
substituidos e o carro e o modelo bicicleta, como em tools/mpc_offline.py.
"""
import math
import random

import pytest
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path

import mpc_python.mpc_controller as M


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def path_msg(points):
    msg = Path()
    msg.header.frame_id = 'map'
    for x, y in points:
        ps = PoseStamped()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        msg.poses.append(ps)
    return msg


def obstacles_msg(obstacles):
    """(x, y) ou (x, y, raio): o raio vai em position.z (ver OBSTACLE_SIZE_FROM_Z)."""
    msg = PoseArray()
    for ob in obstacles:
        p = Pose()
        p.position.x, p.position.y = float(ob[0]), float(ob[1])
        if len(ob) > 2:
            p.position.z = float(ob[2])
        msg.poses.append(p)
    return msg


def clearance():
    """Distancia do caminho desviado a um obstaculo sem tamanho."""
    return M.OBSTACLE_RADIUS + M.AVOID_MARGIN


def curved_path():
    """Reta de 4 m + arco de raio 2.5 m (90 graus) + reta de 4 m, pontos a 5 cm."""
    pts = [(0.05 * i, 0.0) for i in range(80)]
    for i in range(int(2.5 * math.pi / 2 / 0.05)):
        a = 0.05 * i / 2.5
        pts.append((4.0 + 2.5 * math.sin(a), 2.5 - 2.5 * math.cos(a)))
    pts += [(6.5, 2.5 + 0.05 * i) for i in range(80)]
    return pts


def make_controller(points, obstacles, x=0.0, y=0.0, yaw=0.0):
    c = M.MPCController()
    out = {'rpm': 0.0, 'servo': M.SERVO_CENTER}
    c.speed_pub.publish = lambda m: out.__setitem__('rpm', m.data)
    c.steer_pub.publish = lambda m: out.__setitem__('servo', m.data)
    c.marker_pub.publish = lambda m: None
    c.throttled_log = lambda s: out.__setitem__('log', s)
    c.x, c.y, c.yaw, c.have_odom = x, y, yaw, True
    c.path_cb(path_msg(points))
    c.obstacles_cb(obstacles_msg(obstacles or [(50.0, 50.0)]))
    return c, out


def original_cost(self, u):
    """Copia do mpc_cost_function original (antes do [ca_ws] build_cost), para comparar."""
    H = self.H
    control_v = u[:H]; control_g = u[H:]
    if any((v < M.V_MIN) or (v > M.V_MAX) for v in control_v): return 1e8
    if any(abs(g) > M.GAMMA_MAX for g in control_g): return 1e8
    states = self.simulate_vehicle_dynamics(self.x, self.y, self.yaw, control_v, control_g, H=H)
    if self.ref_start_override is not None:
        start_idx = int(self.ref_start_override)
    else:
        _, start_idx, _ = self.find_closest_point_ordered(self.x, self.y)
    base_pts, base_heads, base_curv = self.get_local_reference_base(start_idx, H=H)
    margin = M.PATH_FILTER_MARGIN * (M.FILTER_MARGIN_AFTER_FACTOR if self.post_obs_bias_count > 0 else 1.0)
    relevant_obs = self.obstacles_relevant_for_points(base_pts, extra=self.obstacle_extra(margin))
    if relevant_obs:
        ref_pts, ref_heads, ref_curv = self.filter_ref_points_near_obstacles(
            base_pts, base_heads, base_curv, relevant_obs, margin=margin)
    else:
        ref_pts, ref_heads, ref_curv = base_pts, base_heads, base_curv
    if not ref_pts:
        return 1e8
    rejoin_factor = (1.0 + M.REJOIN_GAIN) if self.post_obs_bias_count > 0 else 1.0
    total = 0.0; big = 1e9
    for k, (st, rp, rh, curv) in enumerate(zip(states, ref_pts, ref_heads, ref_curv)):
        x, y, yaw, vk = st; rx, ry = rp
        r_soft = M.OBSTACLE_RADIUS + M.OBSTACLE_SOFT_MARGIN
        for (ox, oy) in relevant_obs:
            d_obs = math.hypot(x - ox, y - oy)
            if d_obs < r_soft:
                total += M.W_OBSTACLE_SOFT * (r_soft - d_obs) ** 2
        dist = math.hypot(x - rx, y - ry); total += M.w_cross_track * dist * dist
        if dist > M.TRACK_HALF_WIDTH: total += big * (dist - M.TRACK_HALF_WIDTH + 1.0)
        hd = abs(self.normalize_angle(yaw - rh)); total += M.w_heading * hd
        total += rejoin_factor * (k / max(1, H)) * M.w_terminal * dist
        eps = 1e-6
        v_curv = M.V_MAX if abs(curv) < eps else math.sqrt(max(M.MAX_LAT_ACC / (abs(curv) + eps), 0.0))
        v_curv = max(M.V_MIN, min(M.V_MAX, v_curv))
        heading_scaler = max(0.2, 1.0 - 0.95 * (hd / (math.pi / 2.0)))
        cte_scaler = max(0.25, 1.0 - 1.5 * dist)
        v_ref = max(M.V_MIN, min(M.V_MAX, v_curv * heading_scaler * cte_scaler))
        total += 3.5 * (vk - v_ref) ** 2
        v_soft_max = v_ref * (1.0 + M.V_SAFETY_MARGIN)
        if vk > v_soft_max: total += 300.0 * (vk - v_soft_max) ** 2
    total += M.w_control_effort * sum(float(v) ** 2 for v in control_v)
    total += M.w_control_effort * sum(float(g) ** 2 for g in control_g)
    for k in range(H - 1):
        total += M.w_control_smooth * (control_v[k + 1] - control_v[k]) ** 2
        total += M.w_control_smooth * (control_g[k + 1] - control_g[k]) ** 2
    return float(total)


@pytest.mark.parametrize('obstacles', [[], [(4.9, 0.6), (6.4, 1.5)]])
def test_build_cost_equals_original_cost(obstacles):
    c, _ = make_controller(curved_path(), obstacles)
    rng = random.Random(3)
    for _ in range(300):
        i = rng.randrange(0, len(c.ref_path) - 2)
        c.x = c.ref_path[i][0] + rng.uniform(-0.3, 0.3)
        c.y = c.ref_path[i][1] + rng.uniform(-0.3, 0.3)
        c.yaw = c.path_headings[i] + rng.uniform(-0.5, 0.5)
        c.current_segment = max(0, i - 3)
        c.H = H = rng.choice([M.H_SHORT, M.H_LONG])
        c.post_obs_bias_count = rng.choice([0, 0, 5])
        u = ([rng.uniform(M.V_MIN, M.V_MAX) for _ in range(H)]
             + [rng.uniform(-M.GAMMA_MAX, M.GAMMA_MAX) for _ in range(H)])
        seg = c.current_segment
        expected = original_cost(c, u)
        c.current_segment = seg
        got = c.build_cost(H)(u)
        assert got == pytest.approx(expected, rel=1e-9, abs=1e-9)
    c.destroy_node()


def test_reference_goes_around_obstacle_on_path():
    straight = [(0.05 * i, 0.0) for i in range(201)]          # 10 m
    c, _ = make_controller(straight, [(5.0, 0.0)])
    d_min = min(math.hypot(x - 5.0, y) for x, y in c.ref_path)
    assert d_min == pytest.approx(clearance(), abs=0.01)
    # lado por defeito (+1 = esquerda) e o resto do caminho fica onde estava
    assert max(y for _, y in c.ref_path) == pytest.approx(clearance(), abs=0.01)
    far = [(x, y) for x, y in c.ref_path if abs(x - 5.0) > clearance() + M.AVOID_RAMP + 0.05]
    assert far and all(abs(y) < 1e-9 for _, y in far)
    # o progresso usa os mesmos indices do caminho recebido
    assert len(c.ref_path) == len(c.nominal_path)
    c.destroy_node()


def test_reference_moves_away_from_obstacle_beside_path():
    straight = [(0.05 * i, 0.0) for i in range(201)]
    c, _ = make_controller(straight, [(5.0, 0.3)])             # 0.3 m a esquerda
    assert min(y for _, y in c.ref_path) == pytest.approx(0.3 - clearance(), abs=0.01)
    assert min(math.hypot(x - 5.0, y - 0.3) for x, y in c.ref_path) >= clearance() - 0.01
    # obstaculo longe do caminho: referencia igual ao caminho
    c.obstacles_cb(obstacles_msg([(5.0, 2.0)]))
    assert c.ref_path == c.nominal_path
    c.destroy_node()


def drive(c, out, steps, x, y, yaw):
    """Modelo bicicleta com os atrasos do kinematic_sim (perfil do MPC)."""
    v = delta = 0.0
    trace = []
    span = M.SERVO_MAX - M.SERVO_CENTER
    for _ in range(steps):
        c.x, c.y, c.yaw, c.velocity = x, y, yaw, v
        c.update()
        for _ in range(15):
            h = 0.02
            v += h / (0.2 + h) * (out['rpm'] / M.GAIN_RPM_PER_MS - v)
            ratio = max(-1.0, min(1.0, (out['servo'] - M.SERVO_CENTER) / span))
            delta += h / (0.08 + h) * (-ratio * M.GAMMA_MAX - delta)
            x += v * math.cos(yaw) * h
            y += v * math.sin(yaw) * h
            yaw += v * math.tan(delta) / M.WHEEL_BASE * h
            trace.append((x, y))
    return trace


@pytest.mark.parametrize('obstacle', [(5.0, 0.0), (5.0, -0.25)])
def test_closed_loop_passes_obstacle_on_straight(obstacle):
    straight = [(0.05 * i, 0.0) for i in range(241)]           # 12 m
    c, out = make_controller(straight, [obstacle])
    trace = drive(c, out, 70, 0.0, 0.0, 0.0)
    ox, oy = obstacle
    assert min(math.hypot(x - ox, y - oy) for x, y in trace) > M.OBSTACLE_RADIUS
    assert max(x for x, _ in trace) > 10.0, 'o carro parou antes de passar o obstaculo'
    c.destroy_node()


def test_closed_loop_passes_obstacle_in_curve():
    pts = curved_path()
    obstacle = pts[130]                                          # em cima do arco
    c, out = make_controller(pts, [obstacle])
    trace = drive(c, out, 80, 0.0, 0.0, 0.0)
    assert min(math.hypot(x - obstacle[0], y - obstacle[1]) for x, y in trace) > M.OBSTACLE_RADIUS
    assert max(y for _, y in trace) > 5.0, 'o carro parou antes de passar o obstaculo'
    c.destroy_node()


@pytest.fixture
def sized():
    """Liga OBSTACLE_SIZE_FROM_Z (como no simulador) so durante o teste."""
    old = M.OBSTACLE_SIZE_FROM_Z
    M.OBSTACLE_SIZE_FROM_Z = True
    yield
    M.OBSTACLE_SIZE_FROM_Z = old


def test_raio_duro_vem_do_tamanho_do_obstaculo(sized):
    straight = [(0.05 * i, 0.0) for i in range(201)]
    c, _ = make_controller(straight, [(5.0, 0.0, 0.15)])
    hard = 0.15 + M.CAR_HALF_WIDTH + M.OBSTACLE_MARGIN
    assert c.obs_radius((5.0, 0.0)) == pytest.approx(hard)
    d_min = min(math.hypot(x - 5.0, y) for x, y in c.ref_path)
    assert d_min == pytest.approx(hard + M.AVOID_MARGIN, abs=0.01)
    # sem tamanho (z = 0) continua o raio fixo
    c.obstacles_cb(obstacles_msg([(5.0, 0.0)]))
    assert c.obs_radius((5.0, 0.0)) == pytest.approx(M.OBSTACLE_RADIUS)
    c.destroy_node()


def test_closed_loop_obstaculo_pequeno_passa_mais_perto(sized):
    straight = [(0.05 * i, 0.0) for i in range(241)]
    c, out = make_controller(straight, [(5.0, 0.0, 0.15)])
    trace = drive(c, out, 70, 0.0, 0.0, 0.0)
    d = min(math.hypot(x - 5.0, y) for x, y in trace)
    hard = 0.15 + M.CAR_HALF_WIDTH + M.OBSTACLE_MARGIN
    assert hard < d < M.OBSTACLE_RADIUS + M.AVOID_MARGIN, f'passou a {d:.2f} m'
    assert max(x for x, _ in trace) > 10.0
    c.destroy_node()
