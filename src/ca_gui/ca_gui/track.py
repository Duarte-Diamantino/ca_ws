"""A pista: mapa (map.yaml), onde e estrada, e caminhos pela estrada entre pontos.

O desenho da pista FNR e um PNG com as linhas brancas sobre preto. Daqui sai:
  - estrada: as zonas pretas fechadas pelas linhas continuas (menos o fora da pista e o miolo do
    laco) + as linhas pintadas "pequenas" (tracejado do meio, passadeira), que se podem pisar;
  - paredes: as linhas continuas (componentes brancas grandes), o fora e o miolo.
O planeamento e um caminho mais curto numa grelha de CELL m (scipy dijkstra) com custo maior perto
das paredes e em cima das linhas pintadas, depois suavizado e reamostrado.
"""
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

CELL = 0.05              # m, grelha de planeamento
CAR_HALF_W = 0.11        # m, meia largura do carro + folga: o centro nunca fica mais perto das paredes
D_PREF = 0.30            # m, distancia as paredes a partir da qual nao ha penalizacao
W_WALL = 6.0             # custo extra (x comprimento) perto das paredes
W_PAINT = 1.5            # custo extra em cima de linhas pintadas (tracejado, passadeira)
SMALL_LINE_PX = 4000     # componente branca com menos pixeis (no mapa 2x) = linha pintada, pisavel
LOOP_CENTER = (5.505, 2.159)   # miolo do laco grande (m)
# zonas (m) onde as linhas brancas sao pintura que se pisa mesmo sendo grandes: a caixa da
# passadeira na reta de baixo (linhas de 12 cm de x=1.27 a 1.92, y=-0.6 a 0.8)
CROSSABLE_ZONES = [((1.22, -0.65), (1.97, 0.86))]
SPACING = 0.10           # m entre pontos do caminho publicado


class Track:
    def __init__(self, map_yaml):
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), img_path)
        self.res = float(meta['resolution'])
        self.origin = (float(meta['origin'][0]), float(meta['origin'][1]))
        img = np.array(Image.open(img_path).convert('L'))
        if int(meta.get('negate', 0)):
            img = 255 - img
        self.h, self.w = img.shape
        self.size_m = (self.w * self.res, self.h * self.res)
        self._classify(img >= 128)
        self._build_grid()

    # ------------------------------------------------------------------ mapa
    def world_to_px(self, x, y):
        return (x - self.origin[0]) / self.res, self.h - (y - self.origin[1]) / self.res

    def _classify(self, white):
        closed = ndimage.binary_closing(white, structure=np.ones((3, 3)), iterations=1)
        lab, _ = ndimage.label(~closed)
        border = set(np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]]))) - {0}
        c, r = self.world_to_px(*LOOP_CENTER)
        loop = lab[int(r), int(c)]
        road = (lab > 0) & ~np.isin(lab, list(border | {loop}))
        wl, n = ndimage.label(white)
        sizes = ndimage.sum(np.ones_like(wl), wl, index=np.arange(1, n + 1))
        small = np.zeros(n + 1, bool)
        small[1:] = sizes < SMALL_LINE_PX
        paint = small[wl] & white
        for (x0, y0), (x1, y1) in CROSSABLE_ZONES:
            c0, r1 = self.world_to_px(x0, y0)
            c1, r0 = self.world_to_px(x1, y1)
            zone = np.zeros_like(white)
            zone[max(0, int(r0)):int(r1) + 1, max(0, int(c0)):int(c1) + 1] = True
            paint |= white & zone
        self.paint_px = paint
        self.drivable_px = road | paint
        self.white_px = white

    def _build_grid(self):
        k = max(1, int(round(CELL / self.res)))
        self.cell = k * self.res
        H, W = self.h // k, self.w // k
        drv = self.drivable_px[:H * k, :W * k].reshape(H, k, W, k).mean(axis=(1, 3))
        paint = self.paint_px[:H * k, :W * k].reshape(H, k, W, k).mean(axis=(1, 3))
        self.gh, self.gw, self.k = H, W, k
        free = drv > 0.5
        # distancia (m) a parede mais proxima
        self.dist = ndimage.distance_transform_edt(free) * self.cell
        self.free = free & (self.dist >= CAR_HALF_W)
        pen = np.clip((D_PREF - self.dist) / D_PREF, 0.0, 1.0) ** 2
        self.cost = 1.0 + W_WALL * pen + W_PAINT * (paint > 0.3)
        self._graph = None

    def cell_of(self, x, y):
        c, r = self.world_to_px(x, y)
        return int(r // self.k), int(c // self.k)

    def cell_center(self, r, c):
        return (self.origin[0] + (c + 0.5) * self.cell,
                self.origin[1] + self.h * self.res - (r + 0.5) * self.cell)

    def is_drivable(self, x, y, margin=True):
        r, c = self.cell_of(x, y)
        if not (0 <= r < self.gh and 0 <= c < self.gw):
            return False
        return bool(self.free[r, c] if margin else self.dist[r, c] > 0)

    # ------------------------------------------------------------------ planeamento
    def _ensure_graph(self):
        if self._graph is not None:
            return
        H, W = self.gh, self.gw
        rr, cc = np.nonzero(self.free)
        rows, cols, vals = [], [], []
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            r2, c2 = rr + dr, cc + dc
            inside = (r2 >= 0) & (r2 < H) & (c2 >= 0) & (c2 < W)
            r1, c1, r2, c2 = rr[inside], cc[inside], r2[inside], c2[inside]
            ok = self.free[r2, c2]
            r1, c1, r2, c2 = r1[ok], c1[ok], r2[ok], c2[ok]
            w = math.hypot(dr, dc) * self.cell * 0.5 * (self.cost[r1, c1] + self.cost[r2, c2])
            ia, ib = r1 * W + c1, r2 * W + c2
            rows += [ia, ib]
            cols += [ib, ia]
            vals += [w, w]
        self._graph = csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                                 shape=(H * W, H * W))

    def nearest_free(self, x, y, max_m=0.6):
        """Celula livre mais proxima de (x, y) (o clique pode cair numa linha)."""
        r0, c0 = self.cell_of(x, y)
        n = int(max_m / self.cell)
        best = None
        for r in range(max(0, r0 - n), min(self.gh, r0 + n + 1)):
            for c in range(max(0, c0 - n), min(self.gw, c0 + n + 1)):
                if self.free[r, c]:
                    d = (r - r0) ** 2 + (c - c0) ** 2
                    if best is None or d < best[0]:
                        best = (d, r, c)
        return None if best is None else (best[1], best[2])

    def route(self, a, b):
        """Caminho pela estrada de a para b (pontos em m). None se nao houver."""
        self._ensure_graph()
        ca, cb = self.nearest_free(*a), self.nearest_free(*b)
        if ca is None or cb is None:
            return None
        ia, ib = ca[0] * self.gw + ca[1], cb[0] * self.gw + cb[1]
        dist, pred = dijkstra(self._graph, indices=ia, return_predecessors=True)
        if not np.isfinite(dist[ib]):
            return None
        cells = [ib]
        while cells[-1] != ia:
            cells.append(pred[cells[-1]])
        cells.reverse()
        return [self.cell_center(i // self.gw, i % self.gw) for i in cells]

    def route_through(self, waypoints, closed=False):
        """Caminho suave pela estrada a passar pelos pontos (em m). Devolve (pontos, avisos)."""
        pts, warns = [], []
        legs = list(zip(waypoints, waypoints[1:]))
        if closed and len(waypoints) >= 3:
            legs.append((waypoints[-1], waypoints[0]))
        for i, (a, b) in enumerate(legs):
            seg = self.route(a, b)
            if seg is None:
                warns.append(f'sem caminho pela estrada do ponto {i + 1} para o {(i + 1) % len(waypoints) + 1}')
                seg = [a, b]
            pts += seg if not pts else seg[1:]
        if len(pts) < 2:
            return pts, warns
        return self.smooth(pts, closed=closed), warns

    def smooth(self, pts, closed=False, window_m=0.6, iters=3):
        """Suaviza (media movel) sem sair da estrada e reamostra a SPACING."""
        p = resample(np.asarray(pts, float), 0.05, closed)
        n = max(1, int(window_m / 0.05 / 2))
        for _ in range(iters):
            if closed:
                ext = np.vstack([p[-n:], p, p[:n]])
            else:
                ext = np.vstack([np.repeat(p[:1], n, 0), p, np.repeat(p[-1:], n, 0)])
            ker = np.ones(2 * n + 1) / (2 * n + 1)
            q = np.c_[np.convolve(ext[:, 0], ker, 'valid'), np.convolve(ext[:, 1], ker, 'valid')]
            if not closed:
                q[0], q[-1] = p[0], p[-1]
            ok = np.array([self.is_drivable(x, y, margin=False) for x, y in q])
            p = np.where(ok[:, None], q, p)
        return [tuple(v) for v in resample(p, SPACING, closed)]


def resample(p, step, closed=False):
    p = np.asarray(p, float)
    if closed:
        p = np.vstack([p, p[:1]])
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 1e-9:
        return p[:1]
    n = max(2, int(round(s[-1] / step)) + 1)
    t = np.linspace(0.0, s[-1], n)
    out = np.c_[np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])]
    return out[:-1] if closed else out


def path_stats(pts, closed=False):
    """(comprimento m, curvatura maxima 1/m)."""
    p = np.asarray(pts, float)
    if len(p) < 3:
        return 0.0, 0.0
    if closed:
        p = np.vstack([p, p[:2]])
    length = float(np.sum(np.hypot(*np.diff(p, axis=0).T)))
    a, b, c = p[:-2], p[1:-1], p[2:]
    ab, bc, ca = (np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(a - c).T))
    cross = np.abs((b - a)[:, 0] * (c - a)[:, 1] - (b - a)[:, 1] * (c - a)[:, 0])
    k = np.where(ab * bc * ca > 1e-9, 2.0 * cross / np.maximum(ab * bc * ca, 1e-12), 0.0)
    return length, float(k.max())


def q_path():
    """O caminho Q de teste (pickup_description/q_path_publisher), em m."""
    import rclpy
    from pickup_description.q_path_publisher import QPathPublisher
    own = not rclpy.ok()
    if own:
        rclpy.init()
    try:
        q = QPathPublisher()
        cap = {}
        q.path_pub.publish = lambda m: cap.__setitem__('p', m)
        q.marker_pub.publish = lambda m: None
        q.on_timer()
        q.destroy_node()
        return [(ps.pose.position.x, ps.pose.position.y) for ps in cap['p'].poses]
    finally:
        if own:
            rclpy.shutdown()
