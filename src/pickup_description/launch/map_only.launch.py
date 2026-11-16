#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('pickup_description')
    # [ca_ws] map_file:= escolhe (nao 'map': o gazebo_sim.launch.py inclui este launch com map:=true) o mapa desta pasta (ver maps/README.md); por defeito o map.yaml de sempre
    map_yaml = PathJoinSubstitution([pkg_share, 'maps', LaunchConfiguration('map_file')])

    # 1Container isolado para componentes
    container = ComposableNodeContainer(
        name='map_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[{
                    'yaml_filename': map_yaml,
                    'frame_id': 'map',
                    'topic_name': 'map'
                }],
            ),
        ],
        output='screen',
    )

    # 2Lifecycle manager para configurar+ativar automaticamente
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value='map.yaml',
                              description='ficheiro .yaml em pickup_description/maps (ex.: map_afonso.yaml)'),
        container,
        lifecycle_manager,
    ])
