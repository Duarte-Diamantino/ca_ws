"""Abre a janela sem ecra (Qt offscreen, sem ROS) e faz um caminho e obstaculos."""
import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from PyQt5.QtCore import Qt  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402


@pytest.fixture(scope='module')
def win():
    from ca_gui.app import MainWindow, dark_palette
    app = QApplication.instance() or QApplication([])
    dark_palette(app)
    share = get_package_share_directory
    w = MainWindow(os.path.join(share('pickup_description'), 'maps', 'map.yaml'),
                   os.path.join(share('ca_gazebo'), 'worlds', 'media', 'pista.png'), ros=False)
    w.show()
    app.processEvents()
    yield w
    w.close()


def test_pontos_dao_caminho(win):
    for x, y in [(-0.43, 0.0), (5.4, 0.0), (7.6, 2.6)]:
        win.on_click(x, y, int(Qt.LeftButton))
    assert len(win.waypoints) == 3 and len(win.path) > 50
    assert win.dirty
    win.on_wp_removed(2)
    assert len(win.waypoints) == 2


def test_obstaculos(win):
    win.set_mode('obstaculos')
    win.on_click(2.7, 0.0, int(Qt.LeftButton))
    win.on_click(2.75, 0.05, int(Qt.RightButton))
    assert win.obstacles == []


def test_tabela_do_mpc(win):
    assert 'V_MAX' in win.mpc_rows and win.mpc_current()['V_MAX'] > 0


def test_tabela_do_dwa(win):
    values = win.dwa_current()
    assert 'v_max' in win.dwa_rows
    assert values['v_max'] == pytest.approx(1.2)       # src/dwa_follower/config/dwa.yaml
    assert values['use_marker_size'] is True


def test_sair_do_separador_obstaculos_desliga_o_modo(win):
    win.on_tab(1)
    assert win.view.mode == 'obstaculos'
    win.on_tab(3)                                      # DWA (afinar)
    assert win.view.mode == 'pontos'


def test_anel_tracejado_desligado_por_defeito(win):
    win.set_obstacles([(2.7, 0.0, 0.15)])
    assert len(win.view.obstacle_items) == 1          # so o cilindro
    win.ring_cb.setChecked(True)
    assert len(win.view.obstacle_items) == 2          # cilindro + raio do MPC
    win.ring_cb.setChecked(False)
    win.set_obstacles([])


def test_avisos_repetidos_aparecem_uma_vez(win):
    win.log_box.clear()
    line = ('[dwa] [WARN] [1794585939.187107070] [dwa_global_follower]: '
            'Relaxing two-hit constraint at first level (blocked segment?)')
    for _ in range(5):
        win.on_ros_log(line)
    text = win.log_box.toPlainText()
    assert text.count('Relaxing two-hit') == 1
    assert '[dwa] AVISO: Relaxing' in text and 'Nao e um erro' in text
    assert win.quiet_counts.get('Relaxing two-hit constraint') == 4   # o resto so conta no painel
    win.refresh_status()
    assert '4x' in win.tele_lbl.text()
