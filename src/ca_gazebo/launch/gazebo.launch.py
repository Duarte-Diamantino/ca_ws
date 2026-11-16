#!/usr/bin/env python3
"""Gazebo Fortress com a pista e o carro (sem controlador).

  ros2 launch ca_gazebo gazebo.launch.py                    # servidor sem janela + carro em (0, 0)
  ros2 launch ca_gazebo gazebo.launch.py gui:=true          # com a janela do Gazebo
  ros2 launch ca_gazebo gazebo.launch.py obstacles:="[3.0, 0.0, 5.0, 1.0]"
  ros2 launch ca_gazebo gazebo.launch.py spawn:=false       # o carro e criado por outro no
                                                            # (ex.: ca_sim/gz_spawner no inicio do caminho)

Topicos ROS: /odometry/filtered, /tf, /joint_states, /clock (Gazebo -> ROS) e
/ca_car/cmd/{steer_left,steer_right,wheel_rear_left,wheel_rear_right} (ROS -> Gazebo).
Para conduzir com os comandos do VESC usa o ca_sim (vesc_to_gz), ex.:
  ros2 launch ca_sim sim_closed_loop.launch.py sim:=gazebo gui:=true
"""
import os
import shutil
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# O Gazebo (ign-transport) so por 127.0.0.1. Por defeito anuncia-se em todas as interfaces de rede
# (Wi-Fi, Tailscale/VPN, ...); se a rede do Windows muda com a simulacao aberta (o WSL segue-a, ex.
# o PC adormece ou troca de Wi-Fi), a descoberta parte-se ("Exception sending a multicast message:
# Network is unreachable") e a ponte deixa de receber a odometria do carro: o MPC fica parado.
# Quem fala com este Gazebo (ca_sim/drive, "ign topic" a mao) tem de usar o mesmo IGN_IP.
os.environ.setdefault('IGN_IP', '127.0.0.1')

OBSTACLE_SDF = """
    <model name="obstaculo_{i}">
      <static>true</static>
      <pose>{x} {y} {half_h} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def parse_list(text):
    text = text.strip().strip('[]')
    return [float(v) for v in text.split(',') if v.strip()] if text else []


def build_world(world_file, obstacles, radius, height=0.3):
    """Copia o mundo para um ficheiro temporario com os obstaculos e o caminho da textura absoluto."""
    with open(world_file) as f:
        sdf = f.read()
    if len(obstacles) % 2:
        raise RuntimeError(f'obstacles tem de ter pares x, y (recebi {len(obstacles)} valores)')
    models = ''.join(
        OBSTACLE_SDF.format(i=i // 2, x=obstacles[i], y=obstacles[i + 1], r=radius, h=height,
                            half_h=height / 2.0)
        for i in range(0, len(obstacles), 2))
    sdf = sdf.replace('<!-- OBSTACULOS -->', models)
    # caminhos relativos -> absolutos (o mundo e copiado para /tmp)
    worlds = os.path.dirname(os.path.abspath(world_file))
    sdf = sdf.replace('>media/', '>' + os.path.join(worlds, 'media') + '/')
    sdf = sdf.replace('<uri>meshes/', '<uri>' + os.path.join(os.path.dirname(worlds), 'meshes') + '/')
    fd, path = tempfile.mkstemp(prefix='ca_gazebo_', suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)
    return path


def robot_description(xacro_args):
    share = get_package_share_directory('ca_gazebo')
    doc = xacro.process_file(os.path.join(share, 'urdf', 'ca_car.urdf.xacro'), mappings=xacro_args)
    return doc.toxml()


def _setup(context):
    share = get_package_share_directory('ca_gazebo')
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    gui = cfg('gui').lower() in ('1', 'true', 'yes')
    use_sim_time = cfg('use_sim_time').lower() in ('1', 'true', 'yes')
    world = build_world(cfg('world'), parse_list(cfg('obstacles')), float(cfg('obstacle_radius')))

    # Nao uso o gz_sim.launch.py do ros_gz_sim: corre o Gazebo com shell=True e o SIGINT do
    # launch fica no /bin/sh -> ao fim de 5 s o sh leva SIGTERM e o servidor fica orfao
    # (a correr fisica em fundo). Sem shell, o SIGINT chega ao servidor e ele fecha limpo.
    ign = shutil.which('ign')
    if ign is None:
        raise RuntimeError('nao encontro o "ign" (Gazebo Fortress): sudo apt install ros-humble-ros-gz')
    cmd = ['ruby', ign, 'gazebo', '-r', '-v', cfg('gz_verbosity')]
    if not gui:
        cmd.append('-s')
    cmd += [world, '--force-version', '6']
    gz = ExecuteProcess(cmd=cmd, name='gazebo', output='screen', sigterm_timeout='8',
                        # fechar a janela do Gazebo termina o launch todo
                        on_exit=[EmitEvent(event=Shutdown(reason='o Gazebo terminou'))])

    car = {n: cfg(n) for n in ('wheelbase', 'track', 'wheel_radius')}
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', output='log',
               parameters=[{'robot_description': robot_description(car),
                            'use_sim_time': use_sim_time}])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='ca_gz_bridge',
                  output='log',
                  parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                               'use_sim_time': use_sim_time}])
    actions = [gz, rsp, bridge]
    if cfg('spawn').lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='ros_gz_sim', executable='create', name='spawn_ca_car', output='log',
            arguments=['-world', 'pista', '-name', 'ca_car', '-topic', 'robot_description',
                       '-x', cfg('x'), '-y', cfg('y'), '-z', '0.07', '-Y', cfg('yaw')]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('ca_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='abrir a janela do Gazebo'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'pista.sdf')),
        DeclareLaunchArgument('obstacles', default_value='[]',
                              description='lista plana [x1, y1, x2, y2, ...] em metros (cilindros)'),
        DeclareLaunchArgument('obstacle_radius', default_value='0.15',
                              description='raio dos cilindros [m] (o marker do obstacle_publisher tem 0.3 m de diametro)'),
        DeclareLaunchArgument('spawn', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # geometria do carro (medida no CAD: ver urdf/ca_car.urdf.xacro)
        DeclareLaunchArgument('wheelbase', default_value='0.324'),
        DeclareLaunchArgument('track', default_value='0.1975'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0545'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_verbosity', default_value='1'),
        OpaqueFunction(function=_setup),
    ])
