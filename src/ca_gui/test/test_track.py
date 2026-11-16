"""Estrada e caminhos pela estrada na pista FNR (ca_gui.track)."""
import math
import os

import pytest
from ament_index_python.packages import get_package_share_directory

from ca_gui.track import Track, path_stats


@pytest.fixture(scope='module')
def track():
    return Track(os.path.join(get_package_share_directory('pickup_description'), 'maps', 'map.yaml'))


def test_classificacao(track):
    assert track.is_drivable(-0.43, 0.0)            # inicio do caminho Q
    assert track.is_drivable(5.5, 0.0)              # reta de baixo
    assert track.is_drivable(7.6, 2.6)              # laco
    assert not track.is_drivable(5.5, 2.16, margin=False)   # miolo do laco
    assert not track.is_drivable(4.0, 6.5, margin=False)    # fora da pista


def test_reta_atravessa_a_passadeira(track):
    seg = track.route((-0.43, 0.0), (5.4, 0.0))
    assert seg is not None, 'a passadeira (x 1.3-1.9) nao pode cortar a reta'
    length, _ = path_stats(seg)
    assert 5.8 < length < 6.6
    assert all(abs(y) < 0.6 for _, y in seg)


def test_caminho_pelos_pontos_fica_na_estrada(track):
    wps = [(-0.43, 0.0), (5.4, 0.0), (7.6, 2.6), (5.0, 4.4), (2.9, 1.0)]
    path, warns = track.route_through(wps)
    assert not warns
    assert all(track.is_drivable(x, y, margin=False) for x, y in path)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
    assert max(gaps) < 0.15
    for wx, wy in wps:                               # passa perto de cada ponto
        assert min(math.hypot(x - wx, y - wy) for x, y in path) < 0.3
