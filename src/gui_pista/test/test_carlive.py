"""[ca_ws] Carro da simulacao desenhado no GUI (mixins/carlive.py), sem ROS a correr."""
import math
import os

import pytest

pytest.importorskip('tkinter')
import tkinter as tk  # noqa: E402

from ament_index_python.packages import get_package_share_directory  # noqa: E402


@pytest.fixture
def app():
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        pytest.skip('sem display')
    from gui_pista.app import TrajApp
    from gui_pista.theme import apply_theme
    root = tk.Tk()
    root.withdraw()
    apply_theme(root)
    a = TrajApp(master=root)
    a.start_car_live = lambda *args, **kw: False      # sem ROS neste teste
    a.log_msg = lambda *args, **kw: None
    yaml_path = os.path.join(get_package_share_directory('pickup_description'), 'maps', 'map.yaml')
    assert a.load_map_yaml(yaml_path)
    yield a
    root.destroy()


def test_world_to_px_inverte_px_to_world(app):
    for xm, ym in [(-0.432, 0.0), (5.505, 2.159), (8.0, -0.5)]:
        px, py = app._world_to_px(xm, ym)
        bx, by = app._px_to_world(px, py)
        assert bx == pytest.approx(xm, abs=1e-9) and by == pytest.approx(ym, abs=1e-9)


def test_desenha_carro_e_rasto_e_sobrevive_ao_redraw(app):
    app._car_live_on = True
    app._car_trail = [(0.0, -0.4, 0.0), (0.1, 0.0, 0.0), (0.2, 0.5, 0.05)]
    app._car_pose = (0.5, 0.05, math.radians(30.0), 1.2)
    app._car_artists = []
    app._car_draw()
    assert len(app._car_artists) == 3                 # rasto, carro, velocidade
    body = app._car_artists[1]
    xy = body.get_xy()
    front = app._px_to_world(*xy[2])                  # vertice da frente
    c, s = math.cos(math.radians(30.0)), math.sin(math.radians(30.0))
    assert front[0] == pytest.approx(0.5 + 0.448 * c, abs=1e-6)
    assert front[1] == pytest.approx(0.05 + 0.448 * s, abs=1e-6)
    app._perform_redraw()                             # ax.clear() + volta a desenhar o carro
    assert len(app._car_artists) == 3 and app._car_artists[1] in app.ax.patches
