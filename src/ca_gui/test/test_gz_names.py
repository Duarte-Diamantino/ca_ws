"""Nomes dos obstaculos no Gazebo (ca_gui/ros_io.py): fixos por posicao/raio e reconheciveis."""
from ca_gui.ros_io import obstacle_name, parse_gz_models


def test_nome_fixo_por_posicao_e_raio():
    assert obstacle_name(2.7, 0.0, 0.15) == obstacle_name(2.7000004, -0.0, 0.15)
    assert obstacle_name(2.7, 0.0, 0.15) != obstacle_name(2.7, 0.0, 0.2)
    assert obstacle_name(-1.25, 0.5, 0.1) == 'ca_obs_m1250_500_100'


def test_le_os_obstaculos_da_resposta_do_gazebo():
    scene = '''name: "pista"
model {
  name: "ca_car"
  link { name: "chassis" }
}
model {
  name: "ca_obs_2700_0_150"
  link { name: "l" visual { name: "v" } }
}
model {
  name: "ca_obstaculo_3"
}
model {
  name: "cone_1"
}'''
    assert parse_gz_models(scene) == {'ca_obs_2700_0_150', 'ca_obstaculo_3'}
