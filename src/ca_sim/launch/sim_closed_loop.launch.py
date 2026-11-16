#!/usr/bin/env python3
"""Teste em circuito fechado: controlador da TESE + carro simulado + monitor PASS/FAIL.

  q_path_publisher -> /reference_path ─┐
  obstacle_publisher -> /obstacles ────┼─> controlador (MPC ou DWA) -> /commands/*
  carro simulado <- /commands/*  ──────┘        ^
        └──> /odometry/filtered ────────────────┘
  tracking_monitor mede o erro e termina o launch com PASS/FAIL.

Carro simulado (sim:=):
  kinematic  modelo bicicleta em Python (ca_sim/kinematic_sim), rapido, sem fisica
  gazebo     Gazebo Fortress (ca_gazebo) com rodas, atrito e servo; os comandos do VESC
             passam pelo ca_sim/vesc_to_gz; todos os nos usam o relogio da simulacao

Exemplos:
  ros2 launch ca_sim sim_closed_loop.launch.py controller:=mpc
  ros2 launch ca_sim sim_closed_loop.launch.py controller:=dwa rviz:=true
  ros2 launch ca_sim sim_closed_loop.launch.py sim:=gazebo gui:=true
  ros2 launch ca_sim sim_closed_loop.launch.py obstacles:="[3.0, 0.0]"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ca_sim.vesc import CAR, PROFILES


def _parse_list(text):
    text = text.strip().strip('[]')
    return [float(v) for v in text.split(',') if v.strip()] if text else []


def _truthy(text):
    return text.lower() in ('1', 'true', 'yes')


def _setup(context):
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    controller = cfg('controller')
    if controller not in PROFILES:
        raise RuntimeError(f"controller tem de ser um de {list(PROFILES)}, nao '{controller}'")
    sim = cfg('sim')
    if sim not in ('kinematic', 'gazebo'):
        raise RuntimeError(f"sim tem de ser 'kinematic' ou 'gazebo', nao '{sim}'")
    gazebo = sim == 'gazebo'
    profile = PROFILES[controller]
    obstacles_txt = cfg('obstacles')
    obstacles = _parse_list(obstacles_txt)
    sim_time = {'use_sim_time': gazebo}

    path_pub = Node(package='pickup_description', executable='q_path_publisher',
                    name='q_path_publisher', output='log')
    obstacles_pub = Node(
        package='obstacle_publisher', executable='obstacle_publisher_node',
        name='obstacle_publisher', output='log',
        parameters=[{'positions': obstacles, 'goal': [0.0, 0.0]}])
    if controller == 'mpc':
        ctrl = Node(package='mpc_python', executable='mpc_controller',
                    name='mpc_controller', output='log', parameters=[sim_time])
    else:
        # parametros do simulador (dwa_follower/config/dwa.yaml: v_max, goal_tol, ...); dwa_goal_tol:=
        # por cima
        dwa_yaml = os.path.join(get_package_share_directory('dwa_follower'), 'config', 'dwa.yaml')
        ctrl = Node(package='dwa_follower', executable='dwa_global_follower',
                    name='dwa_global_follower', output='log',
                    parameters=[dwa_yaml, {'goal_tol': float(cfg('dwa_goal_tol'))}, sim_time])
    monitor = Node(package='ca_sim', executable='tracking_monitor', name='tracking_monitor',
                   output='screen',
                   parameters=[{'timeout_s': float(cfg('timeout')),
                                'max_cte': float(cfg('max_cte')),
                                'min_obstacle_clearance': float(cfg('min_obstacle_clearance')),
                                'trace_period': float(cfg('trace'))}, sim_time])
    actions = [path_pub, obstacles_pub, monitor]

    if not gazebo:
        car = Node(package='ca_sim', executable='kinematic_sim', name='kinematic_sim',
                   output='screen', parameters=[profile])
        # o controlador so arranca quando o sim e o monitor ja tem o caminho
        actions += [car, TimerAction(period=2.0, actions=[ctrl])]
    else:
        world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ca_gazebo'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'gui': cfg('gui'), 'spawn': 'false',
                              'obstacles': obstacles_txt, 'use_sim_time': 'true',
                              **{k: str(v) for k, v in CAR.items()}}.items())
        # calibracao do controlador + geometria do carro (CAR sobrepoe o wheelbase do perfil)
        vesc = Node(package='ca_sim', executable='vesc_to_gz', name='vesc_to_gz', output='log',
                    parameters=[profile, CAR, sim_time])
        spawner = Node(package='ca_sim', executable='gz_spawner', name='gz_spawner',
                       output='screen')

        def after_spawn(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='gz_spawner nao criou o carro'))]
            # 1 s para o carro assentar no chao antes de o controlador comecar
            return [TimerAction(period=1.0, actions=[ctrl])]

        actions += [world, vesc, spawner,
                    RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_spawn))]

    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=monitor,
        on_exit=[EmitEvent(event=Shutdown(reason='tracking_monitor terminou'))])))
    if _truthy(cfg('rviz')):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', cfg('rviz_config')], parameters=[sim_time]))
    return actions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('ca_sim'), 'config', 'sim.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='mpc', description='mpc | dwa'),
        DeclareLaunchArgument('sim', default_value='kinematic', description='kinematic | gazebo'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='sim:=gazebo: abrir a janela do Gazebo'),
        # longe do caminho por defeito: o MPC so arranca depois de receber /obstacles
        DeclareLaunchArgument('obstacles', default_value='[50.0, 50.0]',
                              description='lista plana [x1, y1, x2, y2, ...] em metros'),
        DeclareLaunchArgument('dwa_goal_tol', default_value='0.3',
                              description='tolerancia de chegada do DWA [m] (default do no: 0.05)'),
        DeclareLaunchArgument('timeout', default_value='90.0'),
        DeclareLaunchArgument('max_cte', default_value='0.5'),
        DeclareLaunchArgument('min_obstacle_clearance', default_value='0.0'),
        DeclareLaunchArgument('trace', default_value='0.0',
                              description='>0: o monitor escreve a pose a cada N segundos'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        OpaqueFunction(function=_setup),
    ])
