"""Testes da calibracao VESC e da geometria Ackermann (sem ROS a correr)."""
import math

import pytest

from ca_sim.vesc import (PROFILES, VescCalibration, ackermann_angles, rear_wheel_speeds,
                         start_pose_from_path)


@pytest.mark.parametrize('name', sorted(PROFILES))
def test_servo_ao_centro_e_reto_e_satura_no_maximo(name):
    p = PROFILES[name]
    c = VescCalibration(p['speed_cmd_per_mps'], p['servo_center'], p['servo_span'],
                        p['steer_max_rad'], p['steer_sign'])
    assert c.steer(p['servo_center']) == 0.0
    top = p['servo_center'] + p['servo_span']
    assert c.steer(top) == pytest.approx(-p['steer_max_rad'])        # steer_sign = -1
    assert c.steer(top + 0.3) == pytest.approx(-p['steer_max_rad'])  # satura
    assert c.steer(p['servo_center'] - p['servo_span']) == pytest.approx(p['steer_max_rad'])


def test_velocidade_mpc():
    p = PROFILES['mpc']
    c = VescCalibration(p['speed_cmd_per_mps'], p['servo_center'], p['servo_span'],
                        p['steer_max_rad'])
    assert c.speed(7000.0) == pytest.approx(1.0)
    assert c.speed(3000.0) == pytest.approx(3.0 / 7.0)


def test_mpc_e_dwa_tem_calibracoes_diferentes():
    """Documenta o problema do README: se isto falhar, alguem acertou as calibracoes."""
    mpc, dwa = PROFILES['mpc'], PROFILES['dwa']
    assert mpc['servo_center'] != dwa['servo_center']
    assert mpc['steer_max_rad'] != dwa['steer_max_rad']


def test_ackermann_roda_de_dentro_vira_mais():
    L, W = 0.335, 0.25
    assert ackermann_angles(0.0, L, W) == (0.0, 0.0)
    left, right = ackermann_angles(0.3, L, W)       # curva a esquerda
    assert left > 0.3 > right > 0
    left, right = ackermann_angles(-0.3, L, W)      # curva a direita
    assert right < -0.3 < left < 0
    # as duas rodas apontam para o mesmo centro de rotacao (no prolongamento do eixo traseiro)
    left, right = ackermann_angles(0.25, L, W)
    radius = L / math.tan(0.25)
    assert L / math.tan(left) == pytest.approx(radius - W / 2)
    assert L / math.tan(right) == pytest.approx(radius + W / 2)


def test_diferencial_media_igual_a_velocidade():
    vl, vr = rear_wheel_speeds(2.0, 0.3, 0.335, 0.25)
    assert (vl + vr) / 2 == pytest.approx(2.0)
    assert vl < 2.0 < vr
    assert rear_wheel_speeds(2.0, 0.0, 0.335, 0.25) == (2.0, 2.0)


def test_pose_inicial_do_caminho():
    pts = [(-0.4 + 0.1 * i, 0.0) for i in range(10)]
    x, y, yaw = start_pose_from_path(pts)
    assert (x, y) == (-0.4, 0.0) and yaw == pytest.approx(0.0)
    x, y, yaw = start_pose_from_path([(0, 0), (0, 1)])
    assert yaw == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        start_pose_from_path([(0, 0)])
