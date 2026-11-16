"""Mover o carro no Gazebo (servico /world/<mundo>/set_pose), usado pelo drive (tecla r) e pelo
gz_spawner (carro no inicio do caminho publicado no GUI)."""
import math
import os
import shutil
import subprocess

# igual ao ca_gazebo/launch/gazebo.launch.py: o Gazebo so fala por 127.0.0.1
os.environ.setdefault('IGN_IP', '127.0.0.1')


def set_car_pose(x, y, yaw, z=0.07, world='pista', name='ca_car', timeout_ms=3000):
    """Teleporta o modelo. Devolve (ok, texto da resposta/erro)."""
    ign = shutil.which('ign')
    if ign is None:
        return False, 'sem Gazebo (ign)'
    req = (f'name: "{name}" position: {{x: {x:.4f} y: {y:.4f} z: {z:.3f}}} '
           f'orientation: {{x: 0 y: 0 z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f}}}')
    try:
        res = subprocess.run(['ruby', ign, 'service', '-s', f'/world/{world}/set_pose',
                              '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                              '--timeout', str(timeout_ms), '--req', req],
                             capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 7.0)
    except subprocess.TimeoutExpired:
        return False, 'timeout'
    out = (res.stdout + res.stderr).strip()
    return res.returncode == 0 and 'true' in res.stdout, out[:200]
