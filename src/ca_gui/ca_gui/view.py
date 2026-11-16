"""Vista do mapa: a pista a escala real (1 unidade da cena = 1 m, y para cima), zoom e arrastar,
e as camadas: caminho, pontos, obstaculos, carro, rasto e plano previsto do MPC."""
import math

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import (QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPixmap,
                         QPolygonF, QTransform)
from PyQt5.QtWidgets import (QGraphicsEllipseItem, QGraphicsItem, QGraphicsPathItem,
                             QGraphicsPixmapItem, QGraphicsPolygonItem, QGraphicsScene,
                             QGraphicsSimpleTextItem, QGraphicsView)

# contorno do carro em relacao ao eixo traseiro (medidas do CAD, ca_gazebo)
CAR_REAR, CAR_FRONT, CAR_HALF_W = -0.12, 0.448, 0.097

C_PATH = QColor('#22d3ee')
C_PATH_WARN = QColor('#f43f5e')
C_WP = QColor('#facc15')
C_OBST = QColor('#ef4444')
C_CAR = QColor('#2563eb')
C_TRAIL = QColor('#fb923c')
C_PRED = QColor('#fde047')
C_START = QColor('#22c55e')


def S(x, y):
    """Mundo (m, y para cima) -> cena (y para baixo)."""
    return QPointF(x, -y)


def W(p):
    return p.x(), -p.y()


def cosmetic(color, width=2.0, style=Qt.SolidLine):
    pen = QPen(color, width, style)
    pen.setCosmetic(True)
    pen.setCapStyle(Qt.RoundCap)
    pen.setJoinStyle(Qt.RoundJoin)
    return pen


class Label(QGraphicsSimpleTextItem):
    """Texto com tamanho fixo no ecra."""

    def __init__(self, text='', color=Qt.white, size=9, bold=False):
        super().__init__(text)
        f = QFont()
        f.setPointSize(size)
        f.setBold(bold)
        self.setFont(f)
        self.setBrush(QBrush(QColor(color)))
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)


class Waypoint(QGraphicsEllipseItem):
    R = 0.09

    def __init__(self, view, index, x, y):
        super().__init__(-self.R, -self.R, 2 * self.R, 2 * self.R)
        self.view = view
        self.index = index
        self.setPos(S(x, y))
        self.setBrush(QBrush(C_WP))
        self.setPen(cosmetic(QColor('#1f2937'), 2))
        self.setZValue(20)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setCursor(Qt.SizeAllCursor)
        self.label = Label(str(index + 1), '#111827', 8, True)
        self.label.setParentItem(self)
        self.label.setPos(-0.035, -0.06)

    def mouseReleaseEvent(self, ev):
        super().mouseReleaseEvent(ev)
        self.view.waypoint_moved.emit(self.index, *W(self.pos()))

    def mousePressEvent(self, ev):
        if ev.button() == Qt.RightButton:
            self.view.waypoint_removed.emit(self.index)
            ev.accept()
            return
        super().mousePressEvent(ev)


class MapView(QGraphicsView):
    clicked = pyqtSignal(float, float, int)          # x, y, botao (modo pontos/obstaculos)
    drawn = pyqtSignal(list)                         # pontos desenhados a mao
    waypoint_moved = pyqtSignal(int, float, float)
    waypoint_removed = pyqtSignal(int)
    mouse_at = pyqtSignal(float, float)
    key_event = pyqtSignal(int, bool)                # tecla, carregada

    MODES = ('mover', 'pontos', 'desenhar', 'obstaculos')

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene_ = QGraphicsScene(self)
        self.setScene(self.scene_)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        self.setBackgroundBrush(QBrush(QColor('#2a2d33')))
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.mode = 'pontos'
        self._drawing = None
        self._panning = None
        self.world_rect = QRectF(-1.3, -6.2, 10.2, 7.0)

        self.path_item = self._path_item(C_PATH, 3.5, 10)
        self.warn_item = self._path_item(C_PATH_WARN, 6.0, 11)
        self.draw_item = self._path_item(QColor('#a78bfa'), 2.5, 12)
        self.trail_item = self._path_item(C_TRAIL, 2.0, 30)
        self.pred_item = self._path_item(C_PRED, 2.0, 31)
        self.start_item = QGraphicsPolygonItem()
        self.start_item.setBrush(QBrush(C_START))
        self.start_item.setPen(cosmetic(QColor('#052e16'), 1.5))
        self.start_item.setZValue(13)
        self.scene_.addItem(self.start_item)
        self.car_item = QGraphicsPolygonItem()
        self.car_item.setBrush(QBrush(C_CAR))
        self.car_item.setPen(cosmetic(Qt.white, 1.5))
        self.car_item.setZValue(40)
        self.car_item.setVisible(False)
        self.scene_.addItem(self.car_item)
        self.car_label = Label('', 'white', 9, True)
        self.car_label.setZValue(41)
        self.car_label.setVisible(False)
        self.scene_.addItem(self.car_label)
        self.waypoints = []
        self.obstacle_items = []

    def _path_item(self, color, width, z):
        it = QGraphicsPathItem()
        it.setPen(cosmetic(color, width))
        it.setZValue(z)
        self.scene_.addItem(it)
        return it

    # ------------------------------------------------------------------ fundo
    def set_background(self, image_path, origin, size_m):
        """Imagem da pista a cobrir o retangulo do mapa (origin = canto inferior esquerdo)."""
        pix = QPixmap(image_path)
        ox, oy = origin
        w, h = size_m
        item = QGraphicsPixmapItem(pix)
        item.setTransformationMode(Qt.SmoothTransformation)
        item.setTransform(QTransform.fromScale(w / pix.width(), h / pix.height()))
        item.setPos(S(ox, oy + h))
        item.setZValue(0)
        self.scene_.addItem(item)
        self.world_rect = QRectF(S(ox, oy + h), S(ox + w, oy)).normalized()
        grid = QPainterPath()
        for x in range(math.ceil(ox), math.floor(ox + w) + 1):
            grid.moveTo(S(x, oy))
            grid.lineTo(S(x, oy + h))
        for y in range(math.ceil(oy), math.floor(oy + h) + 1):
            grid.moveTo(S(ox, y))
            grid.lineTo(S(ox + w, y))
        g = QGraphicsPathItem(grid)
        g.setPen(cosmetic(QColor(255, 255, 255, 28), 1.0))
        g.setZValue(1)
        self.scene_.addItem(g)
        # escala de 1 m no canto
        bar = QPainterPath()
        bx, by = ox + 0.15, oy + 0.15
        bar.moveTo(S(bx, by))
        bar.lineTo(S(bx + 1.0, by))
        b = QGraphicsPathItem(bar)
        b.setPen(cosmetic(Qt.white, 3))
        b.setZValue(2)
        self.scene_.addItem(b)
        t = Label('1 m', 'white', 8, True)
        t.setPos(S(bx + 0.35, by + 0.2))
        t.setZValue(2)
        self.scene_.addItem(t)
        self.scene_.setSceneRect(self.world_rect.adjusted(-2, -2, 2, 2))
        self.fit()

    def fit(self):
        self.fitInView(self.world_rect.adjusted(-0.2, -0.2, 0.2, 0.2), Qt.KeepAspectRatio)

    # ------------------------------------------------------------------ camadas
    @staticmethod
    def _polyline(pts, closed=False):
        p = QPainterPath()
        if pts:
            p.moveTo(S(*pts[0]))
            for q in pts[1:]:
                p.lineTo(S(*q))
            if closed:
                p.closeSubpath()
        return p

    def set_path(self, pts, closed=False, warn_idx=()):
        self.path_item.setPath(self._polyline(pts, closed))
        wp = QPainterPath()
        for i in warn_idx:
            if 0 < i < len(pts) - 1:
                wp.moveTo(S(*pts[i - 1]))
                wp.lineTo(S(*pts[i + 1]))
        self.warn_item.setPath(wp)
        if len(pts) >= 2:
            (x0, y0), (x1, y1) = pts[0], pts[min(3, len(pts) - 1)]
            a = math.atan2(y1 - y0, x1 - x0)
            tri = [(0.22, 0.0), (-0.10, 0.12), (-0.10, -0.12)]
            self.start_item.setPolygon(QPolygonF([S(x0 + math.cos(a) * u - math.sin(a) * v,
                                                    y0 + math.sin(a) * u + math.cos(a) * v) for u, v in tri]))
        else:
            self.start_item.setPolygon(QPolygonF())

    def set_waypoints(self, pts):
        for it in self.waypoints:
            self.scene_.removeItem(it)
        self.waypoints = [Waypoint(self, i, x, y) for i, (x, y) in enumerate(pts)]
        for it in self.waypoints:
            self.scene_.addItem(it)

    def set_obstacles(self, obstacles, rings=None):
        """rings: raio (m) do tracejado de cada obstaculo (o raio duro do MPC), ou None."""
        for it in self.obstacle_items:
            self.scene_.removeItem(it)
        self.obstacle_items = []
        for i, (x, y, r) in enumerate(obstacles):
            hard_radius = rings[i] if rings else None
            body = QGraphicsEllipseItem(QRectF(S(x - r, y + r), S(x + r, y - r)))
            body.setBrush(QBrush(C_OBST))
            body.setPen(cosmetic(QColor('#7f1d1d'), 1.5))
            body.setZValue(25)
            self.scene_.addItem(body)
            self.obstacle_items.append(body)
            if hard_radius:
                R = hard_radius
                ring = QGraphicsEllipseItem(QRectF(S(x - R, y + R), S(x + R, y - R)))
                ring.setPen(cosmetic(QColor(239, 68, 68, 150), 1.2, Qt.DashLine))
                ring.setZValue(24)
                self.scene_.addItem(ring)
                self.obstacle_items.append(ring)

    def set_car(self, x, y, yaw, v):
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(CAR_REAR, -CAR_HALF_W), (CAR_FRONT - 0.07, -CAR_HALF_W), (CAR_FRONT, 0.0),
               (CAR_FRONT - 0.07, CAR_HALF_W), (CAR_REAR, CAR_HALF_W)]
        self.car_item.setPolygon(QPolygonF([S(x + c * u - s * w, y + s * u + c * w) for u, w in pts]))
        self.car_item.setVisible(True)
        self.car_label.setText(f'{v:+.2f} m/s')
        self.car_label.setPos(S(x + 0.25, y + 0.35))
        self.car_label.setVisible(True)

    def set_trail(self, pts):
        self.trail_item.setPath(self._polyline(pts))

    def set_prediction(self, pts):
        self.pred_item.setPath(self._polyline(pts))

    def set_drawing(self, pts):
        self.draw_item.setPath(self._polyline(pts))

    # ------------------------------------------------------------------ rato e teclado
    def set_mode(self, mode):
        self.mode = mode
        self.setDragMode(QGraphicsView.ScrollHandDrag if mode == 'mover' else QGraphicsView.NoDrag)
        self.viewport().setCursor(Qt.OpenHandCursor if mode == 'mover' else Qt.CrossCursor)

    def wheelEvent(self, ev):
        f = 1.0015 ** ev.angleDelta().y()
        cur = self.transform().m11()
        if 15 < cur * f < 3000:
            self.scale(f, f)

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MiddleButton:
            self._panning = ev.pos()
            ev.accept()
            return
        item = self.itemAt(ev.pos())
        if isinstance(item, (Waypoint, Label)) and self.mode == 'pontos':
            super().mousePressEvent(ev)
            return
        x, y = W(self.mapToScene(ev.pos()))
        if self.mode == 'desenhar' and ev.button() == Qt.LeftButton:
            self._drawing = [(x, y)]
            ev.accept()
            return
        if self.mode in ('pontos', 'obstaculos'):
            self.clicked.emit(x, y, int(ev.button()))
            ev.accept()
            return
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        x, y = W(self.mapToScene(ev.pos()))
        self.mouse_at.emit(x, y)
        if self._panning is not None:
            d = ev.pos() - self._panning
            self._panning = ev.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - d.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - d.y())
            return
        if self._drawing is not None:
            lx, ly = self._drawing[-1]
            if math.hypot(x - lx, y - ly) > 0.03:
                self._drawing.append((x, y))
                self.set_drawing(self._drawing)
            return
        super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.MiddleButton:
            self._panning = None
            return
        if self._drawing is not None:
            pts, self._drawing = self._drawing, None
            self.set_drawing([])
            if len(pts) >= 3:
                self.drawn.emit(pts)
            return
        super().mouseReleaseEvent(ev)

    def keyPressEvent(self, ev):
        if not ev.isAutoRepeat():
            self.key_event.emit(ev.key(), True)
        ev.accept()

    def keyReleaseEvent(self, ev):
        if not ev.isAutoRepeat():
            self.key_event.emit(ev.key(), False)
        ev.accept()
