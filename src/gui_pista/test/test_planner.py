"""Testes do planeador do GUI sem cliques: pista sintetica, origem, objetivos e run_all().

Precisa de um display (WSLg / X11). Sem DISPLAY os testes sao saltados.
"""
import math
import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')),
    reason='sem display para o Tk')


@pytest.fixture
def app():
    import tkinter as tk

    import numpy as np
    from PIL import Image

    from gui_pista.app import TrajApp
    from gui_pista.theme import apply_theme

    root = tk.Tk()
    root.withdraw()
    apply_theme(root)
    a = TrajApp(master=root)
    # pista sintetica 600x400 px, toda livre
    img = Image.fromarray(np.full((400, 600, 3), 255, dtype=np.uint8))
    a.image_original = img
    a.image = img.copy()
    a.image_array = np.array(a.image)
    a.grid_spacing = 400 / a.grid_count
    a.update_scale()
    yield a
    a.stop_all()
    root.destroy()


def _run_until_done(app, timeout_s=120.0):
    app.run_all()
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        app.master.update()
        st = app.run_state
        if st is not None and st['seg_index'] >= st['seg_count']:
            return st
        time.sleep(0.001)
    pytest.fail(f'planeador nao terminou em {timeout_s}s (estado: {app.status_label.cget("text")})')


def test_planeia_dois_objetivos_sem_obstaculos(app):
    app.start_point = (60.0, 200.0)
    app.start_theta = 0.0
    app.goals = [[300.0, 200.0, 18], [520.0, 200.0, 18]]
    st = _run_until_done(app)
    path = st['full_path']
    assert len(path) > 10, 'caminho vazio'
    ex, ey = path[-1][0], path[-1][1]
    assert math.hypot(ex - 520.0, ey - 200.0) < 18 + app.m2px(0.3), 'nao chegou ao ultimo objetivo'


# [ca_ws] era xfail(strict): o DWA ficava "sem progresso" atras da parede, o segmento era saltado
# e o caminho final ficava vazio. Corrigido em PlanningMixin._dwa_failed (usa o Hybrid A*).
def test_contorna_parede(app):
    app.start_point = (60.0, 200.0)
    app.start_theta = 0.0
    # parede vertical em x=300, de y=80 a y=320 -> tem de passar por cima ou por baixo
    app.obstacle_points = [(300.0, float(y)) for y in range(80, 321, 4)]
    app.goals = [[520.0, 200.0, 18]]
    st = _run_until_done(app)
    path = st['full_path']
    assert len(path) > 10
    for (x, y, *_rest) in path:
        assert app.is_valid(x, y), f'caminho atravessa a parede em ({x:.0f}, {y:.0f})'


def test_nav_path_mantem_ordem_e_unidades(app):
    """O Path enviado para o ROS tem de estar em metros e pela ordem do caminho."""
    pytest.importorskip('rclpy')
    pts = [(60.0 + 10 * i, 200.0, 0.0, 1.0) for i in range(30)]
    msg = app._build_nav_path(pts)
    xs = [ps.pose.position.x for ps in msg.poses]
    assert len(xs) == 30
    assert all(b > a for a, b in zip(xs, xs[1:])), 'ordem trocada'
    assert math.isclose(xs[1] - xs[0], app.px2m(10.0), rel_tol=1e-6)
