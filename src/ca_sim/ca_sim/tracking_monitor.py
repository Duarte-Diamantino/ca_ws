#!/usr/bin/env python3
"""Mede quao bem o carro segue o /reference_path e termina com PASS/FAIL.

Usado nos testes em circuito fechado (kinematic_sim + MPC/DWA). Termina quando:
  - o carro, depois de andar, fica parado durante stop_hold_s (o controlador deu o
    percurso por terminado ou ficou preso), ou
  - passa o tempo limite (timeout_s).
"Chegou ao fim" = parou a menos de end_tol do ultimo ponto (o MPC para de proposito
0.8 m antes do fim: END_STOP_REMAIN).
Codigo de saida 0 = PASS, 1 = FAIL.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


class TrackingMonitor(Node):
    def __init__(self):
        super().__init__('tracking_monitor')
        p = self.declare_parameter
        p('timeout_s', 90.0)
        p('max_cte', 0.5)            # erro lateral maximo admitido [m]
        p('min_progress', 0.95)      # fracao do caminho que tem de ser percorrida
        p('end_tol', 1.0)            # distancia ao ultimo ponto para considerar "chegou" [m]
        p('stop_hold_s', 3.0)        # parado este tempo (depois de andar) -> termina
        p('min_obstacle_clearance', 0.0)  # >0: falha se passar mais perto que isto de um obstaculo
        p('search_window', 40)
        p('trace_period', 0.0)       # >0: escreve t, x, y, indice, cte, v a cada N segundos
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.timeout_s = float(g('timeout_s'))
        self.max_cte = float(g('max_cte'))
        self.min_progress = float(g('min_progress'))
        self.end_tol = float(g('end_tol'))
        self.stop_hold_s = float(g('stop_hold_s'))
        self.speed = 0.0
        self.stopped_since = None
        self.drive_time = None
        self.min_clear = float(g('min_obstacle_clearance'))
        self.window = int(g('search_window'))
        self.trace_period = float(g('trace_period'))
        self.last_trace = -1e9
        self.last_cte = 0.0

        self.path = []
        self.idx = 0
        self.max_idx = 0
        self.cte_sq_sum = 0.0
        self.cte_max = 0.0
        self.n = 0
        self.dist = 0.0
        self.last_xy = None
        self.cmds = 0
        self.obstacles = []
        self.min_obs_dist = float('inf')
        self.t0 = None
        self.result = None

        self.create_subscription(Path, '/reference_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 50)
        self.create_subscription(Float64, '/commands/motor/speed', self.cmd_cb, 10)
        self.create_subscription(PoseArray, '/obstacles', self.obs_cb, 10)
        self.create_timer(0.5, self.check)

    def path_cb(self, msg):
        if self.path or len(msg.poses) < 2:
            return
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.get_logger().info(f'path recebido: {len(self.path)} pontos')

    def obs_cb(self, msg):
        self.obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def cmd_cb(self, msg):
        if msg.data != 0.0:
            self.cmds += 1

    def odom_cb(self, msg):
        if not self.path:
            return
        if self.t0 is None:
            self.t0 = self.get_clock().now()
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.speed = abs(msg.twist.twist.linear.x)
        if self.last_xy is not None:
            self.dist += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

        lo = max(0, self.idx - 5)
        hi = min(len(self.path) - 1, self.idx + self.window)
        best, best_i = float('inf'), self.idx
        for i in range(lo, hi):
            d = seg_dist(x, y, *self.path[i], *self.path[i + 1])
            if d < best:
                best, best_i = d, i
        self.idx = best_i
        self.max_idx = max(self.max_idx, best_i + 1)
        self.cte_max = max(self.cte_max, best)
        self.cte_sq_sum += best * best
        self.n += 1
        self.last_cte = best
        if self.trace_period > 0.0:
            t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            if t - self.last_trace >= self.trace_period:
                self.last_trace = t
                self.get_logger().info(
                    f'trace t={t:5.1f} x={x:6.2f} y={y:6.2f} idx={best_i:3d}/{len(self.path) - 1} '
                    f'cte={best:.2f} v={self.speed:.2f}')
        for ox, oy in self.obstacles:
            self.min_obs_dist = min(self.min_obs_dist, math.hypot(x - ox, y - oy))

    def progress(self):
        return self.max_idx / max(1, len(self.path) - 1)

    def check(self):
        if self.result is not None or self.t0 is None:
            return
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        moved = self.dist > 1.0
        if moved and self.speed < 0.02:
            if self.stopped_since is None:
                self.stopped_since = elapsed
        else:
            self.stopped_since = None
        stopped = self.stopped_since is not None and elapsed - self.stopped_since >= self.stop_hold_s
        if stopped or elapsed > self.timeout_s:
            self.drive_time = self.stopped_since if stopped else elapsed
            self.finish(elapsed)

    def dist_to_end(self):
        if not (self.last_xy and self.path):
            return float('inf')
        ex, ey = self.path[-1]
        return math.hypot(self.last_xy[0] - ex, self.last_xy[1] - ey)

    def finish(self, elapsed):
        rms = math.sqrt(self.cte_sq_sum / max(1, self.n))
        d_end = self.dist_to_end()
        checks = {
            f'parou_a<={self.end_tol:.2f}m_do_fim': d_end <= self.end_tol,
            f'progresso>={self.min_progress:.2f}': self.progress() >= self.min_progress,
            f'cte_max<={self.max_cte:.2f}m': self.cte_max <= self.max_cte,
            'recebeu_comandos': self.cmds > 0,
        }
        if self.min_clear > 0.0:
            checks[f'distancia_obstaculo>={self.min_clear:.2f}m'] = self.min_obs_dist >= self.min_clear
        ok = all(checks.values())
        lines = [
            '===== RESULTADO DO TESTE =====',
            f'tempo_a_conduzir={self.drive_time:.1f}s  distancia={self.dist:.2f}m  '
            f'progresso={100 * self.progress():.1f}%  dist_ao_fim={d_end:.2f}m',
            f'cte_max={self.cte_max:.3f}m  cte_rms={rms:.3f}m  comandos={self.cmds}',
        ]
        if self.obstacles:
            lines.append(f'distancia_min_obstaculo={self.min_obs_dist:.2f}m')
        lines += [f'  [{"OK" if v else "FALHOU"}] {k}' for k, v in checks.items()]
        lines.append('RESULTADO: ' + ('PASS' if ok else 'FAIL'))
        for line in lines:
            self.get_logger().info(line)
        self.result = 0 if ok else 1
        raise SystemExit(self.result)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingMonitor()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code)
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
