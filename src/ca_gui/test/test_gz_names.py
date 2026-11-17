"""Nomes e sincronizacao dos obstaculos no Gazebo."""
from types import SimpleNamespace

from ca_gui.ros_io import RosBridge, obstacle_name, parse_gz_models
from visualization_msgs.msg import Marker


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


def test_pedidos_de_sincronizacao_sao_agregados():
    class Queue:
        def __init__(self):
            self.calls = []

        def submit(self, *args):
            self.calls.append(args)

    queue = Queue()
    bridge = SimpleNamespace(_gz_pending=False, _gz_closed=False, _gz_dirty=True,
                             obstacles=[(2.7, 0.0, 0.15)], _gz=queue,
                             _gz_job=lambda obstacles: None)
    RosBridge.request_gz_sync(bridge)
    RosBridge.request_gz_sync(bridge)
    assert bridge._gz_pending
    assert len(queue.calls) == 1


def test_marker_deleteall_nao_duplica_primeiro_obstaculo():
    class Pub:
        def publish(self, msg):
            self.msg = msg

    markers = Pub()
    bridge = SimpleNamespace(obstacles=[(2.7, 0.0, 0.15)], obst_pub=Pub(), obst_marker_pub=markers)
    RosBridge.publish_obstacles(bridge)
    keys = [(m.ns, m.id) for m in markers.msg.markers]
    assert len(keys) == len(set(keys))
    assert markers.msg.markers[0].action == Marker.DELETEALL
