#!/usr/bin/env python3
"""ca_gui: planear e conduzir o carro simulado na pista FNR.

  ros2 run ca_gui ca_gui [--map map.yaml] [--texture pista.png] [--obstacles "x1,y1,x2,y2"]
  (normalmente: tools/sim.sh path:=gui)

Mapa a escala real com o carro do Gazebo em tempo real. Clicas pontos na pista e o caminho e
calculado pela estrada (nao corta por cima das linhas), ou desenhas a mao; pes obstaculos (aparecem
no Gazebo e o MPC/DWA veem-nos); arrancas o MPC/DWA ou conduzes com o teclado; afinas o MPC.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QKeySequence, QPalette
from PyQt5.QtWidgets import (QAbstractItemView, QApplication, QButtonGroup, QCheckBox,
                             QDoubleSpinBox, QFileDialog, QFrame, QGridLayout, QGroupBox,
                             QHBoxLayout, QHeaderView, QLabel, QMainWindow, QMessageBox,
                             QPlainTextEdit, QPushButton, QShortcut, QSplitter, QTableWidget,
                             QTableWidgetItem, QTabWidget, QToolButton, QVBoxLayout, QWidget)

from ca_gui.ros_io import RosBridge, mpc_param_files, read_param_files, write_user_params, USER_MPC_PARAMS
from ca_gui.track import Track, path_stats, q_path
from ca_gui.view import MapView

K_CAR = 1.24          # 1/m, curvatura maxima do carro com a calibracao do MPC (22.5 graus, L 0.335)
TRAIL_S = 20.0
STYLE = """
QWidget { font-size: 10pt; }
QGroupBox { border: 1px solid #3f4451; border-radius: 6px; margin-top: 14px; padding: 8px 6px 6px 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #93c5fd; font-weight: bold; }
QPushButton, QToolButton { background: #2f3440; border: 1px solid #454b59; border-radius: 5px; padding: 5px 10px; }
QPushButton:hover, QToolButton:hover { background: #3a4050; }
QPushButton:checked, QToolButton:checked { background: #0e7490; border-color: #22d3ee; }
QPushButton#primary { background: #0e7490; border-color: #22d3ee; font-weight: bold; padding: 8px; }
QPushButton#primary:hover { background: #0891b2; }
QPushButton#stop { background: #991b1b; border-color: #f87171; font-weight: bold; padding: 8px; }
QPushButton#stop:hover { background: #b91c1c; }
QPushButton#dirty { background: #b45309; border-color: #fbbf24; font-weight: bold; padding: 8px; }
QPlainTextEdit { background: #15171c; font-family: monospace; font-size: 9pt; }
QTableWidget { background: #1b1e24; gridline-color: #333844; }
QLabel#big { font-size: 12pt; font-weight: bold; }
QLabel#hint { color: #9ca3af; }
"""


def dark_palette(app):
    app.setStyle('Fusion')
    p = QPalette()
    for role, c in ((QPalette.Window, '#23262d'), (QPalette.WindowText, '#e5e7eb'),
                    (QPalette.Base, '#1b1e24'), (QPalette.AlternateBase, '#23262d'),
                    (QPalette.ToolTipBase, '#111827'), (QPalette.ToolTipText, '#e5e7eb'),
                    (QPalette.Text, '#e5e7eb'), (QPalette.Button, '#2f3440'),
                    (QPalette.ButtonText, '#e5e7eb'), (QPalette.Highlight, '#0e7490'),
                    (QPalette.HighlightedText, '#ffffff')):
        p.setColor(role, QColor(c))
    app.setPalette(p)
    app.setStyleSheet(STYLE)


class MainWindow(QMainWindow):
    def __init__(self, map_yaml, texture, obstacles=(), ros=True):
        super().__init__()
        self.setWindowTitle('Pista FNR: planear e conduzir (ca_gui)')
        self.resize(1500, 920)
        self.track = Track(map_yaml)
        self.waypoints = []
        self.path = []
        self.closed = False
        self.obstacles = [(x, y, 0.15) for x, y in obstacles]
        self.dirty = False
        self.car = None
        self.trail = []
        self.cmd = None
        self.driver_id = ''
        self.keys = set()
        self.kb_v = 0.0
        self.kb_delta = 0.0
        self.kb_last = time.monotonic()

        self.view = MapView()
        self.view.set_background(texture if os.path.isfile(texture or '') else self._map_image(map_yaml),
                                 self.track.origin, self.track.size_m)
        self.view.clicked.connect(self.on_click)
        self.view.drawn.connect(self.on_drawn)
        self.view.waypoint_moved.connect(self.on_wp_moved)
        self.view.waypoint_removed.connect(self.on_wp_removed)
        self.view.mouse_at.connect(lambda x, y: self.mouse_lbl.setText(f'rato: x {x:6.2f}  y {y:6.2f} m'))
        self.view.key_event.connect(self.on_key)

        self._build_ui()
        self.ros = RosBridge() if ros else None
        if self.ros:
            self.ros.odom.connect(self.on_odom)
            self.ros.commands.connect(lambda rpm, servo: setattr(self, 'cmd', (rpm, servo)))
            self.ros.prediction.connect(self.view.set_prediction)
            self.ros.driver.connect(self.on_driver)
            self.ros.log.connect(self.on_ros_log)
            self.ros.controller_ended.connect(self.on_controller_ended)
            if self.obstacles:
                self.ros.set_obstacles(self.obstacles)
        from ca_sim.vesc import VescCalibration
        self.calib = VescCalibration.from_profile('mpc')
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_status)
        self.ui_timer.start(100)
        self.kb_timer = QTimer(self)
        self.kb_timer.timeout.connect(self.keyboard_tick)
        self.kb_timer.start(50)
        self.set_mode('pontos')
        self.redraw_obstacles()
        self.mpc_load_table()
        self.log('Clica na pista para pores pontos: o caminho segue a estrada. Depois "Publicar caminho".')

    @staticmethod
    def _map_image(map_yaml):
        import yaml
        with open(map_yaml) as f:
            img = yaml.safe_load(f)['image']
        return img if os.path.isabs(img) else os.path.join(os.path.dirname(map_yaml), img)

    # ================================================================== interface
    def _build_ui(self):
        root = QSplitter(Qt.Horizontal)
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(6, 6, 0, 6)
        bar = QHBoxLayout()
        self.mode_group = QButtonGroup(self)
        for mode, text, tip in (('mover', 'Mover', 'arrastar o mapa (tambem com o botao do meio)'),
                                ('pontos', 'Pontos', 'clica na pista: o caminho segue a estrada; arrasta '
                                 'os pontos; botao direito apaga'),
                                ('desenhar', 'Desenhar', 'desenha o caminho a mao (e suavizado)'),
                                ('obstaculos', 'Obstaculos', 'clica para por; botao direito tira')):
            b = QToolButton()
            b.setText(text)
            b.setToolTip(tip)
            b.setCheckable(True)
            b.clicked.connect(lambda _, m=mode: self.set_mode(m))
            self.mode_group.addButton(b)
            b.setProperty('mode', mode)
            bar.addWidget(b)
        bar.addSpacing(12)
        fit = QToolButton()
        fit.setText('Ajustar vista')
        fit.clicked.connect(self.view.fit)
        bar.addWidget(fit)
        self.follow_cb = QCheckBox('seguir o carro')
        bar.addWidget(self.follow_cb)
        bar.addStretch(1)
        self.mouse_lbl = QLabel('')
        self.mouse_lbl.setObjectName('hint')
        bar.addWidget(self.mouse_lbl)
        lv.addLayout(bar)
        lv.addWidget(self.view, 1)
        root.addWidget(left)

        right = QWidget()
        right.setMinimumWidth(400)
        right.setMaximumWidth(520)
        rv = QVBoxLayout(right)
        rv.setContentsMargins(6, 6, 6, 6)
        rv.addWidget(self._drive_box())
        tabs = QTabWidget()
        tabs.addTab(self._path_tab(), 'Caminho')
        tabs.addTab(self._obstacles_tab(), 'Obstaculos')
        tabs.addTab(self._mpc_tab(), 'MPC (afinar)')
        tabs.currentChanged.connect(self.on_tab)
        self.tabs = tabs
        rv.addWidget(tabs, 1)
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(800)
        self.log_box.setFixedHeight(150)
        rv.addWidget(self.log_box)
        root.addWidget(right)
        root.setStretchFactor(0, 1)
        self.setCentralWidget(root)
        self.status_lbl = QLabel('')
        self.statusBar().addWidget(self.status_lbl, 1)
        QShortcut(QKeySequence('Ctrl+Z'), self, activated=self.undo_point)

    def _drive_box(self):
        box = QGroupBox('Conduzir')
        g = QGridLayout(box)
        self.drive_lbl = QLabel('ninguem a conduzir')
        self.drive_lbl.setObjectName('big')
        g.addWidget(self.drive_lbl, 0, 0, 1, 4)
        self.btn_mpc = QPushButton('MPC')
        self.btn_dwa = QPushButton('DWA')
        self.btn_kb = QPushButton('Teclado')
        for b in (self.btn_mpc, self.btn_dwa, self.btn_kb):
            b.setCheckable(True)
        self.btn_mpc.setToolTip('arranca/para o mpc_controller (com os parametros do separador MPC)')
        self.btn_dwa.setToolTip('arranca/para o dwa_global_follower')
        self.btn_kb.setToolTip('W/S ou setas: acelerar/travar; A/D: virar. Podes carregar duas ao mesmo '
                               'tempo. Ao largar, desce ate 0 e a direcao centra. O mapa tem de estar selecionado.')
        self.btn_mpc.clicked.connect(lambda on: self.toggle_controller('mpc', on))
        self.btn_dwa.clicked.connect(lambda on: self.toggle_controller('dwa', on))
        self.btn_kb.clicked.connect(self.toggle_keyboard)
        stop = QPushButton('PARAR')
        stop.setObjectName('stop')
        stop.setToolTip('para o MPC/DWA/teclado e trava o carro (Espaco tambem)')
        stop.clicked.connect(self.stop_all)
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.stop_all)
        g.addWidget(self.btn_mpc, 1, 0)
        g.addWidget(self.btn_dwa, 1, 1)
        g.addWidget(self.btn_kb, 1, 2)
        g.addWidget(stop, 1, 3)
        self.tele_lbl = QLabel('carro: sem pose')
        self.tele_lbl.setFont(QFont('monospace', 9))
        g.addWidget(self.tele_lbl, 2, 0, 1, 4)
        reset = QPushButton('Carro ao inicio do caminho')
        reset.clicked.connect(self.car_to_start)
        g.addWidget(reset, 3, 0, 1, 4)
        return box

    def _path_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        hint = QLabel('Modo "Pontos": clica na pista e o caminho vai pela estrada, por ordem. Arrasta '
                      'os pontos para o mudar; botao direito num ponto apaga-o. Modo "Desenhar": a mao.')
        hint.setWordWrap(True)
        hint.setObjectName('hint')
        v.addWidget(hint)
        self.closed_cb = QCheckBox('volta fechada (do ultimo ponto volta ao primeiro)')
        self.closed_cb.toggled.connect(self.on_closed)
        v.addWidget(self.closed_cb)
        g = QGridLayout()
        for i, (text, fn) in enumerate((('Caminho Q (teste)', self.load_q), ('Inverter sentido', self.reverse),
                                        ('Desfazer ponto', self.undo_point), ('Limpar', self.clear_path),
                                        ('Abrir...', self.open_path), ('Guardar...', self.save_path))):
            b = QPushButton(text)
            b.clicked.connect(fn)
            g.addWidget(b, i // 2, i % 2)
        v.addLayout(g)
        self.stats_lbl = QLabel('sem caminho')
        self.stats_lbl.setWordWrap(True)
        v.addWidget(self.stats_lbl)
        v.addStretch(1)
        self.btn_pub = QPushButton('Publicar caminho')
        self.btn_pub.setObjectName('primary')
        self.btn_pub.setToolTip('publica /reference_path (o carro vai para o inicio se estiver parado)')
        self.btn_pub.clicked.connect(self.publish)
        v.addWidget(self.btn_pub)
        return w

    def _obstacles_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        hint = QLabel('Modo "Obstaculos": clica na pista para por um cilindro, botao direito perto de um '
                      'para o tirar. Aparecem no Gazebo e vao para /obstacles (MPC) e /obstacles_marker (DWA).')
        hint.setWordWrap(True)
        hint.setObjectName('hint')
        v.addWidget(hint)
        h = QHBoxLayout()
        h.addWidget(QLabel('raio do obstaculo (m):'))
        self.obst_r = QDoubleSpinBox()
        self.obst_r.setRange(0.05, 0.6)
        self.obst_r.setSingleStep(0.05)
        self.obst_r.setValue(0.15)
        h.addWidget(self.obst_r)
        all_r = QPushButton('aplicar a todos')
        all_r.setToolTip('muda o raio de todos os obstaculos ja postos')
        all_r.clicked.connect(self.resize_all_obstacles)
        h.addWidget(all_r)
        h.addStretch(1)
        v.addLayout(h)
        how = QLabel('O MPC usa o tamanho de cada obstaculo: mantem o centro do carro a '
                     'raio + meia largura do carro (CAR_HALF_WIDTH) + folga (OBSTACLE_MARGIN). '
                     'A folga e a meia largura mudam-se no separador MPC (afinar).')
        how.setWordWrap(True)
        how.setObjectName('hint')
        v.addWidget(how)
        self.ring_cb = QCheckBox('mostrar essa distancia (tracejado)')
        self.ring_cb.setChecked(False)
        self.ring_cb.toggled.connect(lambda _: self.redraw_obstacles())
        v.addWidget(self.ring_cb)
        clear = QPushButton('Tirar todos')
        clear.clicked.connect(lambda: self.set_obstacles([]))
        v.addWidget(clear)
        self.obst_lbl = QLabel('')
        v.addWidget(self.obst_lbl)
        v.addStretch(1)
        return w

    def _mpc_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.mpc_src_lbl = QLabel('')
        self.mpc_src_lbl.setWordWrap(True)
        self.mpc_src_lbl.setObjectName('hint')
        v.addWidget(self.mpc_src_lbl)
        self.mpc_table = QTableWidget(0, 2)
        self.mpc_table.setHorizontalHeaderLabels(['parametro', 'valor'])
        self.mpc_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.mpc_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.mpc_table.verticalHeader().setVisible(False)
        self.mpc_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.mpc_table.itemChanged.connect(self.on_mpc_edit)
        v.addWidget(self.mpc_table, 1)
        g = QGridLayout()
        for i, (text, fn, tip) in enumerate((
                ('Aplicar', self.mpc_apply, 'com o MPC a correr: muda ja (no proximo ciclo)'),
                ('Ler do MPC', self.mpc_load_table, 'le os valores do MPC a correr (ou dos ficheiros)'),
                ('Guardar (os meus)', self.mpc_save, f'grava em {USER_MPC_PARAMS}: usados sempre que o MPC arranca'),
                ('Repor os de origem', self.mpc_reset, 'apaga os teus e volta aos de mpc_python/config/mpc.yaml'))):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            g.addWidget(b, i // 2, i % 2)
        v.addLayout(g)
        return w

    def log(self, text):
        self.log_box.appendPlainText(time.strftime('%H:%M:%S ') + text)

    # mensagens conhecidas dos controladores -> explicacao (mostrada uma vez)
    EXPLAIN = {
        'Relaxing two-hit constraint': (
            'DWA: nenhuma trajetoria segura passa pelos 2 proximos pontos do caminho (a 18 cm), por isso '
            'aceita a melhor segura mesmo assim. E normal com um obstaculo em cima do caminho, com o '
            'carro fora do caminho ou numa curva muito apertada. Nao e um erro.'),
        'sem trajetória livre': (
            'MPC: nenhum plano evita os obstaculos dentro do tubo a volta do caminho, por isso trava. '
            'Afasta o obstaculo ou aumenta PATH_TUBE_LIMIT / baixa OBSTACLE_RADIUS no separador MPC.'),
    }
    REPEAT_S = 15.0
    # avisos normais: aparecem uma vez (com a explicacao) e depois so contam no painel Conduzir
    QUIET = {'Relaxing two-hit constraint': 'DWA a aliviar a regra dos 2 pontos'}

    def on_ros_log(self, text):
        """Linhas dos controladores: sem o carimbo do ROS, repetidas so de 15 em 15 s (com a contagem)."""
        import re
        m = re.match(r'^\[(\w+)\] \[(\w+)\] \[[\d.]+\] \[[^\]]+\]: (.*)$', text)
        if not m:
            self.log(text)
            return
        who, level, msg = m.groups()
        level = {'WARN': 'AVISO', 'ERROR': 'ERRO', 'INFO': ''}.get(level, level)
        quiet = next((k for k in self.QUIET if k in msg), None)
        if quiet and quiet in getattr(self, '_explained', set()):
            counts = getattr(self, 'quiet_counts', {})
            counts[quiet] = counts.get(quiet, 0) + 1
            self.quiet_counts = counts
            return
        key = (who, msg if 'idx=' not in msg else 'idx')
        now = time.monotonic()
        seen = getattr(self, '_seen', {})
        self._seen = seen
        last, count = seen.get(key, (0.0, 0))
        if now - last < self.REPEAT_S:
            seen[key] = (last, count + 1)
            return
        extra = f'  (+{count} iguais nos ultimos {self.REPEAT_S:.0f} s)' if count else ''
        seen[key] = (now, 0)
        self.log(f'[{who}] {level + ": " if level else ""}{msg}{extra}')
        for pat, why in self.EXPLAIN.items():
            if pat in msg and not getattr(self, '_explained', set()).__contains__(pat):
                self._explained = getattr(self, '_explained', set()) | {pat}
                self.log('   -> ' + why)

    # ================================================================== modos e caminho
    def set_mode(self, mode):
        self.view.set_mode(mode)
        for b in self.mode_group.buttons():
            b.setChecked(b.property('mode') == mode)

    def on_tab(self, i):
        if i == 1:
            self.set_mode('obstaculos')
        elif i == 0 and self.view.mode == 'obstaculos':
            self.set_mode('pontos')

    def on_click(self, x, y, button):
        if self.view.mode == 'pontos' and button == int(Qt.LeftButton):
            if not self.track.is_drivable(x, y, margin=False):
                self.log(f'({x:.2f}, {y:.2f}) nao e estrada: o ponto vai para a estrada mais proxima')
            self.waypoints.append((x, y))
            self.reroute()
        elif self.view.mode == 'obstaculos':
            if button == int(Qt.LeftButton):
                self.set_obstacles(self.obstacles + [(round(x, 3), round(y, 3), self.obst_r.value())])
            else:
                if self.obstacles:
                    d = [math.hypot(x - ox, y - oy) for ox, oy, _ in self.obstacles]
                    i = int(np.argmin(d))
                    if d[i] < 0.5:
                        self.set_obstacles(self.obstacles[:i] + self.obstacles[i + 1:])

    def on_drawn(self, pts):
        self.waypoints = []
        self.path = self.track.smooth(pts, closed=self.closed)
        self.view.set_waypoints([])
        self.path_changed()

    def on_wp_moved(self, i, x, y):
        if 0 <= i < len(self.waypoints):
            self.waypoints[i] = (x, y)
            self.reroute()

    def on_wp_removed(self, i):
        if 0 <= i < len(self.waypoints):
            del self.waypoints[i]
            self.reroute()

    def on_closed(self, on):
        self.closed = on
        if self.waypoints:
            self.reroute()

    def reroute(self):
        self.view.set_waypoints(self.waypoints)
        if len(self.waypoints) >= 2:
            self.path, warns = self.track.route_through(self.waypoints, closed=self.closed)
            for wmsg in warns:
                self.log('AVISO: ' + wmsg)
        else:
            self.path = []
        self.path_changed()

    def path_changed(self):
        warn = []
        if len(self.path) >= 3:
            p = np.asarray(self.path)
            a, b, c = p[:-2], p[1:-1], p[2:]
            ab, bc, ca = np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(a - c).T)
            cross = np.abs((b - a)[:, 0] * (c - a)[:, 1] - (b - a)[:, 1] * (c - a)[:, 0])
            k = np.where(ab * bc * ca > 1e-9, 2 * cross / np.maximum(ab * bc * ca, 1e-12), 0)
            warn = list(np.nonzero(k > K_CAR)[0] + 1)
        self.view.set_path(self.path, self.closed, warn)
        if self.path:
            L, kmax = path_stats(self.path, self.closed)
            txt = (f'{len(self.path)} pontos, {L:.2f} m, curvatura maxima {kmax:.2f} 1/m '
                   f'(raio {1 / max(kmax, 1e-6):.2f} m)')
            if warn:
                txt += (f'<br><span style="color:#f87171">curvas mais apertadas do que o carro faz '
                        f'(raio minimo {1 / K_CAR:.2f} m) a vermelho</span>')
            self.stats_lbl.setText(txt)
        else:
            self.stats_lbl.setText('sem caminho: clica pelo menos 2 pontos na pista')
        self.set_dirty(bool(self.path))

    def set_dirty(self, dirty):
        self.dirty = dirty
        self.btn_pub.setObjectName('dirty' if dirty else 'primary')
        self.btn_pub.setText('Publicar caminho (alterado)' if dirty else 'Publicar caminho')
        self.btn_pub.style().unpolish(self.btn_pub)
        self.btn_pub.style().polish(self.btn_pub)

    def load_q(self):
        self.waypoints = []
        self.view.set_waypoints([])
        self.closed_cb.setChecked(False)
        self.path = [tuple(p) for p in q_path()]
        self.path_changed()

    def reverse(self):
        self.waypoints.reverse()
        self.path = list(reversed(self.path))
        self.view.set_waypoints(self.waypoints)
        self.path_changed()

    def undo_point(self):
        if self.waypoints:
            self.waypoints.pop()
            self.reroute()

    def clear_path(self):
        self.waypoints, self.path = [], []
        self.view.set_waypoints([])
        self.path_changed()

    def save_path(self):
        if not self.path:
            return
        fn, _ = QFileDialog.getSaveFileName(self, 'Guardar caminho', os.path.expanduser('~/caminho.csv'),
                                            'CSV (*.csv)')
        if fn:
            np.savetxt(fn, np.asarray(self.path), delimiter=',', header='x,y', comments='', fmt='%.4f')
            self.log(f'caminho guardado em {fn}')

    def open_path(self):
        fn, _ = QFileDialog.getOpenFileName(self, 'Abrir caminho', os.path.expanduser('~'), 'CSV (*.csv)')
        if fn:
            p = np.loadtxt(fn, delimiter=',', skiprows=1)
            self.waypoints = []
            self.view.set_waypoints([])
            self.path = [tuple(r) for r in p[:, :2]]
            self.path_changed()

    def publish(self):
        if len(self.path) < 2:
            QMessageBox.information(self, 'Publicar caminho', 'Ainda nao ha caminho: clica pontos na pista.')
            return
        if self.ros:
            pts = list(self.path) + ([self.path[0]] if self.closed else [])
            self.ros.publish_path(pts)
        self.set_dirty(False)
        self.log(f'caminho publicado ({len(self.path)} pontos). Se o carro estava parado, foi para o inicio.')

    def car_to_start(self):
        if len(self.path) < 2 or not self.ros:
            return
        (x0, y0), (x1, y1) = self.path[0], self.path[min(5, len(self.path) - 1)]
        self.stop_all()
        self.ros.move_car(x0, y0, math.atan2(y1 - y0, x1 - x0))

    # ================================================================== obstaculos
    def set_obstacles(self, obs):
        self.obstacles = obs
        self.redraw_obstacles()
        if self.ros:
            self.ros.set_obstacles(obs)

    def mpc_hard_radius(self, r):
        """Raio duro do MPC para um obstaculo de raio r (como o mpc_controller.obs_radius)."""
        try:
            p = self.mpc_current() if hasattr(self, 'mpc_rows') else {}
        except ValueError:
            p = {}
        if p.get('OBSTACLE_SIZE_FROM_Z', True) and r > 0:
            return r + p.get('CAR_HALF_WIDTH', 0.10) + p.get('OBSTACLE_MARGIN', 0.10)
        return p.get('OBSTACLE_RADIUS', 0.5)

    def redraw_obstacles(self):
        rings = [self.mpc_hard_radius(r) for _, _, r in self.obstacles] if self.ring_cb.isChecked() else None
        self.view.set_obstacles(self.obstacles, rings=rings)
        n = len(self.obstacles)
        if n:
            r = self.obstacles[-1][2]
            self.obst_lbl.setText(f'{n} obstaculo(s). O MPC mantem o centro do carro a {self.mpc_hard_radius(r):.2f} m '
                                  f'de um obstaculo de raio {r:.2f} m.')
        else:
            self.obst_lbl.setText('0 obstaculos')

    def resize_all_obstacles(self):
        r = self.obst_r.value()
        self.set_obstacles([(x, y, r) for x, y, _ in self.obstacles])

    # ================================================================== conduzir
    def toggle_controller(self, kind, on):
        if not self.ros:
            return
        btn = self.btn_mpc if kind == 'mpc' else self.btn_dwa
        if on:
            self.quiet_counts = {}            # contadores de avisos normais: recomecam
            if len(self.path) < 2:
                btn.setChecked(False)
                QMessageBox.information(
                    self, kind.upper(),
                    f'O {kind.upper()} segue o caminho publicado e ainda nao ha caminho.\n\n'
                    'Clica pontos na pista (modo "Pontos") ou carrega em "Caminho Q (teste)", '
                    'e depois volta a carregar aqui.')
                return
            if self.dirty or self.ros.path_msg is None:
                self.publish()                    # o controlador segue o /reference_path
            self.btn_kb.setChecked(False)
            (self.btn_dwa if kind == 'mpc' else self.btn_mpc).setChecked(False)
            self.ros.start_controller(kind)
            self.log(f'{kind.upper()} a arrancar (demora ~2 s; o log dele aparece aqui)...')
        else:
            self.ros.stop_controller()
            self.log(f'{kind.upper()} parado, carro travado')
        btn.setChecked(on)

    def on_controller_ended(self, kind, code):
        self.log(f'o {kind.upper()} terminou sozinho (codigo {code})')
        (self.btn_mpc if kind == 'mpc' else self.btn_dwa).setChecked(False)

    def toggle_keyboard(self, on):
        if not self.ros:
            return
        if on:
            self.ros.stop_controller(brake=False)
            self.btn_mpc.setChecked(False)
            self.btn_dwa.setChecked(False)
            self.ros.claim('teclado', 'mpc')
            self.kb_v = self.kb_delta = 0.0
            self.keys.clear()
            self.view.setFocus()
            self.log('Teclado: W/S (ou setas) acelera/trava, A/D vira; duas teclas ao mesmo tempo funcionam. '
                     'Clica no mapa se as teclas nao fizerem nada.')
        else:
            self.ros.send_command(0.0, self.calib.servo_center)

    def on_key(self, key, down):
        names = {Qt.Key_W: 'w', Qt.Key_Up: 'w', Qt.Key_S: 's', Qt.Key_Down: 's',
                 Qt.Key_A: 'a', Qt.Key_Left: 'a', Qt.Key_D: 'd', Qt.Key_Right: 'd'}
        if key == Qt.Key_Space and down:
            self.stop_all()
            return
        k = names.get(key)
        if k:
            (self.keys.add if down else self.keys.discard)(k)

    def keyboard_tick(self):
        from ca_sim import drive as rc
        now = time.monotonic()
        dt = min(0.2, now - self.kb_last)
        self.kb_last = now
        if not self.btn_kb.isChecked() or not self.ros:
            return
        if not self.view.hasFocus():
            self.keys.clear()                    # janela sem foco: largar tudo
        w, s, a, d = ('w' in self.keys, 's' in self.keys, 'a' in self.keys, 'd' in self.keys)
        if w and not s:
            self.kb_v = min(rc.V_MAX, self.kb_v + rc.ACCEL * dt)
        elif s and not w:
            self.kb_v = max(0.0, self.kb_v - rc.BRAKE * dt) if self.kb_v > 0 else max(rc.V_MIN, self.kb_v - rc.REVERSE_ACCEL * dt)
        else:
            self.kb_v = math.copysign(max(0.0, abs(self.kb_v) - rc.COAST * dt), self.kb_v)
        target = self.calib.steer_max * ((1.0 if a else 0.0) - (1.0 if d else 0.0))
        rate = rc.STEER_RATE if target != 0.0 else rc.STEER_RETURN
        self.kb_delta += max(-rate * dt, min(rate * dt, target - self.kb_delta))
        self.ros.send_command(self.calib.motor_cmd(self.kb_v), self.calib.servo_pos(self.kb_delta))

    def stop_all(self):
        if self.ros:
            self.ros.stop_controller(brake=True)
        self.btn_mpc.setChecked(False)
        self.btn_dwa.setChecked(False)
        self.btn_kb.setChecked(False)
        self.kb_v = self.kb_delta = 0.0

    def on_driver(self, who):
        self.driver_id = who
        mine = self.ros and who == self.ros.my_id
        if not mine and (self.btn_mpc.isChecked() or self.btn_dwa.isChecked() or self.btn_kb.isChecked()):
            self.log(f'{who.split(":")[0]} assumiu o controlo do carro: a GUI largou-o')
            if self.ros:
                self.ros.stop_controller(brake=False)
            self.btn_mpc.setChecked(False)
            self.btn_dwa.setChecked(False)
            self.btn_kb.setChecked(False)

    def on_odom(self, x, y, yaw, v):
        self.car = (x, y, yaw, v)
        now = time.monotonic()
        if not self.trail or math.hypot(x - self.trail[-1][1], y - self.trail[-1][2]) > 0.03:
            self.trail.append((now, x, y))
        while self.trail and now - self.trail[0][0] > TRAIL_S:
            self.trail.pop(0)

    def refresh_status(self):
        if self.car:
            x, y, yaw, v = self.car
            self.view.set_car(x, y, yaw, v)
            self.view.set_trail([(px, py) for _, px, py in self.trail])
            if self.follow_cb.isChecked():
                self.view.centerOn(x, -y)
            car = f'carro {v:+5.2f} m/s  x {x:5.2f} y {y:5.2f}  {math.degrees(yaw):4.0f}°'
        else:
            car = 'carro: sem pose (a simulacao esta aberta?)'
        if self.cmd:
            rpm, servo = self.cmd
            ang = math.degrees(self.calib.steer(servo))
            lado = 'esq' if ang > 0.5 else 'dir' if ang < -0.5 else ''
            cmd = f'cmd   {self.calib.speed(rpm):+5.2f} m/s  direcao {abs(ang):4.1f}° {lado}'
        else:
            cmd = 'comando: --'
        quiet = ''.join(f'\n{self.QUIET[k]}: {n}x (normal com obstaculos)'
                        for k, n in getattr(self, 'quiet_counts', {}).items() if n)
        self.tele_lbl.setText(car + '\n' + cmd + quiet)
        who = self.driver_id.split(':')[0] if self.driver_id else ''
        names = {'mpc': 'MPC (terminal)', 'dwa': 'DWA (terminal)', 'remote': 'comando (terminal)',
                 'gui-mpc': 'MPC (GUI)', 'gui-dwa': 'DWA (GUI)', 'gui-teclado': 'teclado (GUI)'}
        active = (self.btn_mpc.isChecked() or self.btn_dwa.isChecked() or self.btn_kb.isChecked()
                  or (who and not who.startswith('gui')))
        self.drive_lbl.setText(f'a conduzir: {names.get(who, who)}' if active and who else 'ninguem a conduzir')
        self.status_lbl.setText(car + '   |   ' + cmd)

    # ================================================================== MPC
    def mpc_spec(self):
        if not hasattr(self, '_tunable'):
            from mpc_python import mpc_controller as M
            self._tunable = [(n, d, lo, hi, getattr(M, n)) for n, d, lo, hi, _ in M.TUNABLE]
        return self._tunable

    def mpc_current(self):
        out = {}
        for n, item in getattr(self, 'mpc_rows', {}).items():
            out[n] = self._parse(n, item)
        return out

    def _parse(self, name, item):
        spec = {n: (lo, hi, default) for n, _, lo, hi, default in self.mpc_spec()}[name]
        lo, hi, default = spec
        if isinstance(default, bool):
            return item.checkState() == Qt.Checked
        txt = item.text().strip().replace(',', '.')
        val = int(float(txt)) if isinstance(default, int) else float(txt)
        if lo is not None and not (lo <= val <= hi):
            raise ValueError(f'{name} tem de estar entre {lo} e {hi}')
        return val

    def mpc_load_table(self):
        names = [n for n, *_ in self.mpc_spec()]
        live = self.ros.mpc_get_params(names) if (self.ros and self.ros.mpc_get.service_is_ready()) else None
        if live:
            values, src = live, 'valores do MPC a correr: "Aplicar" muda ja.'
        else:
            values = {n: d for n, _, _, _, d in self.mpc_spec()}
            files = mpc_param_files()
            values.update(read_param_files(files))
            mine = os.path.isfile(USER_MPC_PARAMS)
            src = ('MPC parado: valores dos ficheiros' + (' (com os teus, ' + USER_MPC_PARAMS + ')' if mine else '')
                   + '. "Guardar" grava-os para o proximo arranque.')
        self.mpc_src_lbl.setText(src)
        self.mpc_table.blockSignals(True)
        self.mpc_table.setRowCount(0)
        self.mpc_rows = {}
        for n, desc, lo, hi, default in self.mpc_spec():
            r = self.mpc_table.rowCount()
            self.mpc_table.insertRow(r)
            name_item = QTableWidgetItem(n)
            name_item.setFlags(Qt.ItemIsEnabled)
            tip = desc + ('' if lo is None else f'  [{lo} .. {hi}]') + f'\npor defeito: {default}'
            name_item.setToolTip(tip)
            v = values.get(n, default)
            if isinstance(default, bool):
                it = QTableWidgetItem('')
                it.setFlags(Qt.ItemIsEnabled | Qt.ItemIsUserCheckable)
                it.setCheckState(Qt.Checked if v else Qt.Unchecked)
            else:
                it = QTableWidgetItem(f'{v:g}' if isinstance(v, float) else str(v))
                it.setFlags(Qt.ItemIsEnabled | Qt.ItemIsEditable)
            it.setToolTip(tip)
            self.mpc_table.setItem(r, 0, name_item)
            self.mpc_table.setItem(r, 1, it)
            self.mpc_rows[n] = it
        self.mpc_table.blockSignals(False)
        self._mpc_loaded = self.mpc_current()

    def on_mpc_edit(self, item):
        name = next((n for n, it in self.mpc_rows.items() if it is item), None)
        if name is None:
            return
        try:
            changed = self._parse(name, item) != self._mpc_loaded.get(name)
            item.setBackground(QColor('#78350f') if changed else QColor('#1b1e24'))
        except ValueError as exc:
            item.setBackground(QColor('#7f1d1d'))
            self.log(str(exc))

    def mpc_apply(self):
        try:
            cur = self.mpc_current()
        except ValueError as exc:
            QMessageBox.warning(self, 'MPC', str(exc))
            return
        changed = {n: v for n, v in cur.items() if v != self._mpc_loaded.get(n)}
        if not changed:
            self.log('MPC: nada alterado')
            return
        if not (self.ros and self.ros.mpc_set.service_is_ready()):
            self.log('o MPC nao esta a correr: usa "Guardar" para o proximo arranque')
            return
        res = self.ros.mpc_set_params(changed) or []
        for name, ok, reason in res:
            self.log(f'MPC: {name} = {changed[name]}' if ok else f'MPC: {name} recusado ({reason})')
        self.mpc_load_table()
        self.redraw_obstacles()

    def mpc_save(self):
        try:
            path = write_user_params(self.mpc_current())
        except ValueError as exc:
            QMessageBox.warning(self, 'MPC', str(exc))
            return
        self.log(f'parametros do MPC gravados em {path} (usados sempre que o MPC arranca)')
        self.mpc_apply()

    def mpc_reset(self):
        if os.path.isfile(USER_MPC_PARAMS):
            os.remove(USER_MPC_PARAMS)
            self.log(f'{USER_MPC_PARAMS} apagado: voltam os de origem no proximo arranque do MPC')
        if self.ros and self.ros.mpc_set.service_is_ready():
            defaults = {n: d for n, _, _, _, d in self.mpc_spec()}
            defaults.update(read_param_files(mpc_param_files()))
            self.ros.mpc_set_params(defaults)
        self.mpc_load_table()

    def closeEvent(self, ev):
        if self.ros:
            self.ros.shutdown()
        super().closeEvent(ev)


def main(args=None):
    import rclpy
    argv = rclpy.utilities.remove_ros_args(sys.argv if args is None else args)[1:]
    share = get_package_share_directory
    ap = argparse.ArgumentParser(prog='ca_gui')
    ap.add_argument('--map', default=os.path.join(share('pickup_description'), 'maps', 'map.yaml'))
    ap.add_argument('--texture', default=os.path.join(share('ca_gazebo'), 'worlds', 'media', 'pista.png'))
    ap.add_argument('--obstacles', default='', help='x1,y1,x2,y2,... (m)')
    a = ap.parse_args(argv)
    vals = [float(v) for v in a.obstacles.replace('[', '').replace(']', '').split(',') if v.strip()]
    app = QApplication(sys.argv[:1])
    # Ctrl+C (ou o fim do launch) fecha a janela de forma limpa. Instalado antes do rclpy.init:
    # o rclpy encadeia o seu tratamento de sinais neste (senao saia um KeyboardInterrupt a meio
    # de um timer qualquer)
    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    dark_palette(app)
    win = MainWindow(a.map, a.texture, obstacles=list(zip(vals[0::2], vals[1::2])))
    win.show()
    code = app.exec_()
    win.close()                       # closeEvent -> para o MPC/DWA e tira os obstaculos do Gazebo
    sys.exit(code)


if __name__ == '__main__':
    main()
