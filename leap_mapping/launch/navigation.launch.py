"""Localize on the saved map, then wait for the user's RViz navigation goal."""
import os
from pathlib import Path
import socket
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def navigation_nodes(context):
    own = Path(get_package_share_directory('leap_mapping'))
    demo = Path(get_package_share_directory('leap_nav_demo'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    metadata = yaml.safe_load(map_file.read_text(encoding='utf-8'))
    if not isinstance(metadata, dict) or not isinstance(metadata.get('image'), str):
        raise RuntimeError('Map YAML must contain an image path')
    if not (map_file.parent / metadata['image']).is_file():
        raise RuntimeError(f'Map image does not exist: {metadata["image"]}')

    # Match the existing RPP baseline; only the map and manual initialization differ.
    params = yaml.safe_load((nav2 / 'params/nav2_params.yaml').read_text())
    for key in ('local_costmap', 'global_costmap'):
        params[key][key]['ros__parameters']['robot_radius'] = 0.13
    controller = params['controller_server']['ros__parameters']
    controller['FollowPath'] = yaml.safe_load((demo / 'config/rpp.yaml').read_text())
    controller['general_goal_checker']['xy_goal_tolerance'] = 0.20
    params['amcl']['ros__parameters']['set_initial_pose'] = False

    parameter_dir = tempfile.TemporaryDirectory(prefix='leap-map-nav-')
    params_file = Path(parameter_dir.name) / 'nav2.yaml'
    params_file.write_text(yaml.safe_dump(params), encoding='utf-8')

    def cleanup_parameters(context):
        parameter_dir.cleanup()

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=cleanup_parameters),
        ])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(demo / 'launch/simulation.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/bringup_launch.py')),
            launch_arguments={
                'map': str(map_file), 'params_file': str(params_file),
                'use_sim_time': 'true', 'autostart': 'true',
                'use_composition': 'False', 'slam': 'False',
            }.items()),
        Node(package='rviz2', executable='rviz2', name='navigation_view',
             arguments=['-d', str(own / 'rviz/navigation.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Navigation requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    with socket.socket() as probe:
        probe.settimeout(0.2)
        if probe.connect_ex(('127.0.0.1', 11365)) == 0:
            raise RuntimeError('Close the previous mapping/Gazebo session before navigation')
    own = Path(get_package_share_directory('leap_mapping'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=str(
            own / 'maps/my_first_map_20261023_nav.yaml')),
        OpaqueFunction(function=navigation_nodes),
    ])
