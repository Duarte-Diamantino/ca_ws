#!/usr/bin/env python3
"""Simulacao no Gazebo que fica aberta, para conduzir a partir de outros terminais.

Terminal 1:  ros2 launch ca_sim gazebo_sim.launch.py
Terminal 2:  ros2 run ca_sim drive mpc       (espaco = iniciar/parar o MPC)
Terminal 3:  ros2 run ca_sim drive dwa       (espaco = iniciar/parar o DWA)
Terminal 4:  ros2 run ca_sim drive remote    (W/S velocidade, A/D direcao, espaco trava)

Arranca: Gazebo (janela) + carro + ca_sim/vesc_to_gz (comandos VESC -> rodas) + caminho de
referencia + obstaculos + map_server (mapa da pista) + RViz. Nao arranca nenhum controlador
nem o monitor dos testes: so termina quando fechares o Gazebo ou carregares Ctrl+C.

Argumentos:
  path:=q        caminho Q de teste (q_path_publisher)       [defeito]
  path:=gui      GUI_pista: desenhas e publicas o caminho na janela do GUI
  path:=none     nada publica /reference_path (usa o teu proprio no)
  obstacles:="[8.1, 2.16]"   obstaculos (cilindros no Gazebo + /obstacles para os controladores)
  gui:=false  rviz:=false  map:=false
  profile:=mpc|dwa   calibracao inicial do carro (o drive troca-a para a do controlador que conduz)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ca_sim.vesc import CAR, PROFILES


def _truthy(text):
    return text.lower() in ('1', 'true', 'yes')


def _parse_list(text):
    text = text.strip().strip('[]')
    return [float(v) for v in text.split(',') if v.strip()] if text else []


def _setup(context):
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    path = cfg('path')
    if path not in ('q', 'gui', 'gui_antigo', 'none'):
        raise RuntimeError(f"path tem de ser q, gui, gui_antigo ou none (nao '{path}')")
    profile = cfg('profile')
    if profile not in PROFILES:
        raise RuntimeError(f"profile tem de ser um de {list(PROFILES)} (nao '{profile}')")
    obstacles = _parse_list(cfg('obstacles'))
    sim_time = {'use_sim_time': True}
    # path:=gui: a ca_gui e a dona dos obstaculos (poe-nos no Gazebo e publica /obstacles e
    # /obstacles_marker); o obstacle_publisher fica so para o /odom_for_dwa e o /goal
    gui_owns_obstacles = path == 'gui'

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ca_gazebo'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'gui': cfg('gui'), 'spawn': 'false',
                              'obstacles': '[]' if gui_owns_obstacles else cfg('obstacles'),
                              'use_sim_time': 'true',
                              **{k: str(v) for k, v in CAR.items()}}.items()),
        # geometria = a do carro (CAR); a calibracao dos comandos vem do perfil
        Node(package='ca_sim', executable='vesc_to_gz', name='vesc_to_gz', output='log',
             parameters=[{'profile': profile, **CAR}, sim_time]),
        # o carro aparece no inicio do caminho (path:=none -> em x0, y0, yaw0)
        # path:=gui: o carro aparece logo em x0/y0 e vai para o inicio de cada caminho novo do GUI
        Node(package='ca_sim', executable='gz_spawner', name='gz_spawner', output='screen',
             parameters=[{'init_from_path': path == 'q', 'follow_path': path.startswith('gui'),
                          'x0': float(cfg('x0')), 'y0': float(cfg('y0')),
                          'yaw0': float(cfg('yaw0'))}]),
        # sem "positions" o obstacle_publisher publica uma lista vazia (o MPC so arranca depois de
        # receber /obstacles). O "goal" e obrigatorio: sem ele o no aborta (le goal[0], goal[1]).
        Node(package='obstacle_publisher', executable='obstacle_publisher_node',
             name='obstacle_publisher', output='log',
             parameters=[{'goal': [0.0, 0.0]} | ({'positions': obstacles}
                                                 if obstacles and not gui_owns_obstacles else {})],
             remappings=([('/obstacles', '/obstacle_publisher/obstacles'),
                          ('/obstacles_marker', '/obstacle_publisher/obstacles_marker')]
                         if gui_owns_obstacles else [])),
    ]
    if path == 'q':
        actions.append(Node(package='pickup_description', executable='q_path_publisher',
                            name='q_path_publisher', output='log'))
    elif path == 'gui' and _truthy(cfg('gui_app')):
        # GUI nova (ca_gui): mapa, caminhos pela estrada, obstaculos, conduzir, afinar o MPC
        actions.append(Node(package='ca_gui', executable='ca_gui', name='ca_gui', output='screen',
                            arguments=['--obstacles', ','.join(str(v) for v in obstacles)]))
    elif path == 'gui_antigo':
        # [ca_ws] o GUI abre ja o map.yaml da pista: escala e origem do frame map (= mundo Gazebo)
        gui_map = os.path.join(get_package_share_directory('pickup_description'), 'maps', 'map.yaml')
        actions.append(Node(package='gui_pista', executable='gui_pista', name='gui_pista',
                            output='screen', arguments=['--map', gui_map]))  # [ca_ws]
    if _truthy(cfg('map')):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pickup_description'), 'launch', 'map_only.launch.py'))))
        # o RViz le /map_rviz: o mapa volta depois de um Reset do RViz (ver map_relay.py)
        actions.append(Node(package='ca_sim', executable='map_relay', name='map_relay', output='log'))
    if _truthy(cfg('rviz')):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', cfg('rviz_config')], parameters=[sim_time]))
    actions.append(LogInfo(msg=(
        '\n  Simulacao pronta. Noutros terminais (com o workspace carregado):\n'
        '    ros2 run ca_sim drive mpc      # espaco = iniciar/parar o MPC\n'
        '    ros2 run ca_sim drive dwa      # espaco = iniciar/parar o DWA\n'
        '    ros2 run ca_sim drive remote   # W/S velocidade, A/D direcao, espaco trava\n'
        '  Em todos: r = carro de volta ao inicio do caminho, q = sair.\n')))
    return actions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('ca_sim'), 'config', 'sim.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('path', default_value='q', description='q | gui | gui_antigo | none'),
        DeclareLaunchArgument('gui_app', default_value='true',
                              description='path:=gui: abrir a ca_gui (false = so a simulacao preparada para ela)'),
        DeclareLaunchArgument('obstacles', default_value='[]',
                              description='lista plana [x1, y1, x2, y2, ...] em metros'),
        DeclareLaunchArgument('gui', default_value='true', description='janela do Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value='true', description='map_server com o mapa da pista'),
        DeclareLaunchArgument('profile', default_value='mpc', description='calibracao inicial: mpc | dwa'),
        DeclareLaunchArgument('x0', default_value='-0.4'),
        DeclareLaunchArgument('y0', default_value='0.0'),
        DeclareLaunchArgument('yaw0', default_value='0.0'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        OpaqueFunction(function=_setup),
    ])
