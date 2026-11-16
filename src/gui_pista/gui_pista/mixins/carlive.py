"""[ca_ws] Mostra no GUI o carro da simulacao (Gazebo) em tempo real, a partir de /odometry/filtered.

Liga-se sozinho quando o GUI abre um map.yaml (load_map_yaml / --map): so ai as coordenadas do GUI
sao as do frame 'map' (= mundo do Gazebo). Desenha o contorno do carro com a frente marcada, a
velocidade e o rasto dos ultimos TRAIL_S segundos. O no ROS do GUI passa a ser rodado (spin_once)
no ciclo do Tk de 100 em 100 ms; o desenho e feito no maximo de DRAW_PERIOD em DRAW_PERIOD s
(redesenhar a figura do matplotlib e lento).
"""
import math
import time

from matplotlib.patches import Polygon

try:
    import rclpy
    from nav_msgs.msg import Odometry

    HAVE_ODOM = True
except Exception:  # pragma: no cover - sem ROS
    HAVE_ODOM = False

# contorno do carro em relacao ao eixo traseiro (base_link), medidas do CAD (ca_gazebo)
CAR_REAR, CAR_FRONT, CAR_HALF_W = -0.12, 0.448, 0.097
TRAIL_S = 20.0
DRAW_PERIOD = 0.25


class CarLiveMixin:
    """Carro da simulacao desenhado por cima do mapa."""

    def start_car_live(self, topic='/odometry/filtered'):
        if getattr(self, '_car_live_on', False) or not HAVE_ODOM:
            return False
        if not self._ensure_ros2():
            return False
        self._car_live_on = True
        self._car_pose = None          # (x, y, yaw, v) em metros, frame map
        self._car_trail = []           # [(t, x, y)]
        self._car_artists = []
        self._car_last_draw = 0.0
        self._car_new = False
        self.ros_node.create_subscription(Odometry, topic, self._car_odom_cb, 5)
        self.log_msg(f'Carro da simulacao: a mostrar {topic} no mapa (azul, rasto a laranja).')
        self.after(100, self._car_tick)
        return True

    # ------------------------------------------------------------ ROS
    def _car_odom_cb(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        x, y = p.position.x, p.position.y
        self._car_pose = (x, y, yaw, msg.twist.twist.linear.x)
        now = time.monotonic()
        trail = self._car_trail
        if not trail or math.hypot(x - trail[-1][1], y - trail[-1][2]) > 0.03:
            trail.append((now, x, y))
        while trail and now - trail[0][0] > TRAIL_S:
            trail.pop(0)
        self._car_new = True

    def _car_tick(self):
        if not getattr(self, '_car_live_on', False):
            return
        try:
            for _ in range(10):        # a odometria chega a 50 Hz: ~5 mensagens por tick
                if self.ros_node is None:
                    break
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception:
            self._car_live_on = False  # contexto ROS fechado (a sair)
            return
        now = time.monotonic()
        if self._car_new and now - self._car_last_draw >= DRAW_PERIOD and self.image is not None:
            self._car_new = False
            self._car_last_draw = now
            self._car_draw()
            self._draw_canvas()
        self.after(100, self._car_tick)

    # ------------------------------------------------------------ desenho
    def _world_to_px(self, xm, ym):
        """Inverso de ROSMixin._px_to_world."""
        vx, vy = self.axis_x_vec
        ux, uy = self.axis_y_vec
        a = self.m2px(xm - float(self.origin_x_m))
        b = self.m2px(ym - float(self.origin_y_m))
        det = vx * uy - vy * ux
        return (a * uy - vy * b) / det, (vx * b - ux * a) / det

    def _car_outline_px(self):
        x, y, yaw, _ = self._car_pose
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for lx, ly in ((CAR_REAR, -CAR_HALF_W), (CAR_FRONT - 0.06, -CAR_HALF_W), (CAR_FRONT, 0.0),
                       (CAR_FRONT - 0.06, CAR_HALF_W), (CAR_REAR, CAR_HALF_W)):
            pts.append(self._world_to_px(x + c * lx - s * ly, y + s * lx + c * ly))
        return pts

    def _car_draw(self):
        for art in self._car_artists:
            try:
                art.remove()
            except Exception:
                pass                   # ja saiu com o ax.clear() do redraw
        self._car_artists = []
        if self._car_pose is None:
            return
        if len(self._car_trail) >= 2:
            px = [self._world_to_px(x, y) for _, x, y in self._car_trail]
            (line,) = self.ax.plot([p[0] for p in px], [p[1] for p in px], '-', color='#ff8c1a',
                                   lw=1.6, alpha=0.85, zorder=9)
            self._car_artists.append(line)
        body = Polygon(self._car_outline_px(), closed=True, facecolor='#1f5fd1', edgecolor='white',
                       linewidth=1.2, alpha=0.95, zorder=10)
        self.ax.add_patch(body)
        self._car_artists.append(body)
        x, y, _, v = self._car_pose
        tx, ty = self._world_to_px(x, y)
        txt = self.ax.text(tx + 14, ty - 14, f'{v:+.2f} m/s', color='white', fontsize=8, zorder=11,
                           bbox=dict(boxstyle='round,pad=0.2', fc='#1f5fd1', ec='none', alpha=0.85))
        self._car_artists.append(txt)

    def _perform_redraw(self, *args, **kwargs):
        super()._perform_redraw(*args, **kwargs)
        if getattr(self, '_car_live_on', False) and self._car_pose is not None:
            self._car_artists = []     # o ax.clear() ja os tirou
            self._car_draw()
