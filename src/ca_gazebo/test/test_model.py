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


@pytest.mark.parametrize('wheelbase', [0.324, 0.335])
def test_geometria_segue_os_argumentos(wheelbase):
    root = urdf(wheelbase=wheelbase, track=0.1975)
    fl = xyz(joint(root, 'front_left_steering_joint'))
    rl = xyz(joint(root, 'rear_left_wheel_joint'))
    rr = xyz(joint(root, 'rear_right_wheel_joint'))
    assert math.isclose(fl[0] - rl[0], wheelbase), 'distancia entre eixos errada'
    assert math.isclose(rl[1] - rr[1], 0.1975), 'bitola errada'
    assert rl[0] == 0.0 and rl[1] == -rr[1], 'base_link tem de estar no centro do eixo traseiro'


def test_defeitos_sao_os_do_cad():
    """Os defeitos do xacro tem de bater com ca_sim/vesc.py CAR (medidos no CAD)."""
    root = urdf()
    fl = xyz(joint(root, 'front_left_steering_joint'))
    assert math.isclose(fl[0], 0.324) and math.isclose(2 * fl[1], 0.1975)
    sph = root.find("link[@name='rear_left_wheel']/collision/geometry/sphere")
    assert math.isclose(float(sph.get('radius')), 0.0545)


def test_meshes_existem_e_estao_em_mm():
    root = urdf()
    meshes = [m for m in root.iter('mesh')]
    assert len(meshes) == 5, 'carrocaria + 4 rodas'
    for m in meshes:
        path = m.get('filename').replace('file://', '')
        assert os.path.isfile(path), path
        assert m.get('scale') == '0.001 0.001 0.001', 'as STL do CAD estao em mm'


def test_carrocaria_e_rodas_estao_centradas_lateralmente():
    """A bbox da carroçaria tem centro y=28,5 mm; o visual tem de o pôr em y=0."""
    root = urdf()
    mesh_origin = root.find("link[@name='chassis']/visual/origin")
    mesh_y = float(mesh_origin.get('xyz').split()[1])
    assert mesh_y + 0.0285 == pytest.approx(0.0)
    left = xyz(joint(root, 'rear_left_wheel_joint'))[1]
    right = xyz(joint(root, 'rear_right_wheel_joint'))[1]
    assert left + right == pytest.approx(0.0)


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
    names = [m.get('name') for m in world.findall('model')]
    assert 'armazem_paredes' in names
    assert sum(n.startswith('cone_') for n in names) >= 10


def test_cones_e_armazem_fora_da_estrada_e_do_caminho_q():
    """Nada fisico (cones, pilares, estantes, paletes) pode estar a menos de 0.5 m do caminho Q."""
    import sys
    sys.path.insert(0, os.path.join(SRC, '..', 'pickup_description'))
    try:
        import rclpy
        from pickup_description.q_path_publisher import QPathPublisher
    except ImportError:
        pytest.skip('pickup_description nao disponivel')
    rclpy.init()
    try:
        q = QPathPublisher()
        cap = {}
        q.path_pub.publish = lambda m: cap.__setitem__('p', m)
        q.marker_pub.publish = lambda m: None
        q.on_timer()
        path = [(p.pose.position.x, p.pose.position.y) for p in cap['p'].poses]
        q.destroy_node()
    finally:
        rclpy.shutdown()
    world = ET.parse(WORLD).getroot().find('world')
    checked = 0
    for m in world.findall('model'):
        if m.get('name') in ('chao', 'mapa', 'faixas_chao') or m.get('name') == 'armazem_paredes':
            continue
        mp = m.find('pose')
        base = [float(v) for v in mp.text.split()[:2]] if mp is not None else [0.0, 0.0]
        for link in m.findall('link'):
            lp = link.find('pose')
            off = [float(v) for v in lp.text.split()[:2]] if lp is not None else [0.0, 0.0]
            x, y = base[0] + off[0], base[1] + off[1]
            d = min(math.hypot(x - px, y - py) for px, py in path)
            assert d > 0.5, f'{m.get("name")}/{link.get("name")} a {d:.2f} m do caminho Q'
            checked += 1
    assert checked > 20


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
        for uri in root.iter('uri'):
            assert os.path.isabs(uri.text) and os.path.isfile(uri.text), uri.text
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
    assert float(base.find('inertial/mass').text) == pytest.approx(2.6)
    plugins = [p.get('name') for p in model.findall('plugin')]
    assert plugins.count('ignition::gazebo::systems::JointPositionController') == 2
    assert plugins.count('ignition::gazebo::systems::JointController') == 2
