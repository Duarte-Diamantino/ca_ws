"""Verificacoes do modelo e do mundo sem arrancar o Gazebo."""
import importlib.util
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

SRC = os.environ.get('CA_GAZEBO_SRC', os.path.join(os.path.dirname(__file__), '..'))
XACRO = os.path.join(SRC, 'urdf', 'ca_car.urdf.xacro')
WORLD = os.path.join(SRC, 'worlds', 'pista.sdf')
BRIDGE = os.path.join(SRC, 'config', 'bridge.yaml')


def urdf(**args):
    return ET.fromstring(xacro.process_file(XACRO, mappings={k: str(v) for k, v in args.items()}).toxml())


def joint(root, name):
    j = root.find(f"joint[@name='{name}']")
    assert j is not None, f'falta a junta {name}'
    return j


def xyz(j):
    return [float(v) for v in j.find('origin').get('xyz').split()]


@pytest.mark.parametrize('wheelbase', [0.335, 0.324])
def test_geometria_segue_os_argumentos(wheelbase):
    root = urdf(wheelbase=wheelbase, track=0.25)
    fl = xyz(joint(root, 'front_left_steering_joint'))
    rl = xyz(joint(root, 'rear_left_wheel_joint'))
    rr = xyz(joint(root, 'rear_right_wheel_joint'))
    assert math.isclose(fl[0] - rl[0], wheelbase), 'distancia entre eixos errada'
    assert math.isclose(rl[1] - rr[1], 0.25), 'bitola errada'
    assert rl[0] == 0.0 and rl[1] == -rr[1], 'base_link tem de estar no centro do eixo traseiro'


def test_base_link_sem_massa_e_todas_as_outras_com():
    root = urdf()
    for link in root.findall('link'):
        has_inertia = link.find('inertial') is not None
        assert has_inertia == (link.get('name') != 'base_link'), link.get('name')


def test_topicos_dos_plugins_batem_com_o_bridge():
    root = urdf()
    topics = {t.text for t in root.iter('topic')} | {t.text for t in root.iter('odom_topic')} | \
        {t.text for t in root.iter('tf_topic')}
    with open(BRIDGE) as f:
        bridged = {e['gz_topic_name'] for e in yaml.safe_load(f)}
    falta = topics - bridged
    assert not falta, f'topicos do modelo sem bridge: {falta}'
    joints = {j.text for j in root.iter('joint_name')}
    assert joints == {'front_left_steering_joint', 'front_right_steering_joint',
                      'rear_left_wheel_joint', 'rear_right_wheel_joint'}


def test_mundo_e_xml_valido_com_marcador_de_obstaculos():
    with open(WORLD) as f:
        text = f.read()
    assert '<!-- OBSTACULOS -->' in text
    world = ET.fromstring(text).find('world')
    assert world.get('name') == 'pista', 'o launch e o gz_spawner assumem o mundo "pista"'
    assert os.path.isfile(os.path.join(SRC, 'worlds', 'media', 'pista.png'))


def test_build_world_insere_obstaculos():
    spec = importlib.util.spec_from_file_location(
        'gazebo_launch', os.path.join(SRC, 'launch', 'gazebo.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    path = mod.build_world(WORLD, [1.0, 2.0, 3.5, -1.0], 0.15)
    try:
        root = ET.parse(path).getroot()
        obs = [m for m in root.iter('model') if m.get('name', '').startswith('obstaculo_')]
        assert [m.find('pose').text.split()[:2] for m in obs] == [['1.0', '2.0'], ['3.5', '-1.0']]
        tex = root.find(".//albedo_map").text
        assert os.path.isabs(tex) and os.path.isfile(tex)
    finally:
        os.remove(path)
    with pytest.raises(RuntimeError):
        mod.build_world(WORLD, [1.0, 2.0, 3.0], 0.15)


@pytest.mark.skipif(shutil.which('ign') is None or shutil.which('ruby') is None,
                    reason='Gazebo (ign) nao instalado')
def test_urdf_converte_para_sdf(tmp_path):
    """O Gazebo converte o URDF para SDF (e o que o "create" faz ao criar o carro)."""
    f = tmp_path / 'ca_car.urdf'
    f.write_text(xacro.process_file(XACRO).toxml())
    res = subprocess.run(['ruby', shutil.which('ign'), 'sdf', '-p', str(f)],
                         capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    sdf = ET.fromstring(res.stdout)
    model = sdf.find('model')
    names = {j.get('name') for j in model.findall('joint')}
    assert 'front_left_steering_joint' in names and 'rear_right_wheel_joint' in names
    # o chassis (junta fixa) e fundido no base_link -> o base_link fica com a massa
    base = model.find("link[@name='base_link']")
    assert float(base.find('inertial/mass').text) == pytest.approx(3.0)
    plugins = [p.get('name') for p in model.findall('plugin')]
    assert plugins.count('ignition::gazebo::systems::JointPositionController') == 2
    assert plugins.count('ignition::gazebo::systems::JointController') == 2
