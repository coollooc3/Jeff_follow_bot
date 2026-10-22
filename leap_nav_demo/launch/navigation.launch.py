"""Bring up the fixed simulation scene, AMCL and the installed Humble Nav2 stack."""

import os
from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def navigation_nodes(context):
    own = Path(get_package_share_directory('leap_nav_demo'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    tb3_nav = Path(get_package_share_directory('turtlebot3_navigation2'))
    controller = LaunchConfiguration('controller').perform(context)
    if controller not in ('dwb', 'rpp'):
        raise RuntimeError('controller must be dwb or rpp')
    params = yaml.safe_load((nav2 / 'params/nav2_params.yaml').read_text())
    for key in ('local_costmap', 'global_costmap'):
        params[key][key]['ros__parameters']['robot_radius'] = 0.13
    follow = params['controller_server']['ros__parameters']['FollowPath']
    follow['max_vel_x'], follow['max_speed_xy'] = 0.22, 0.22
    if controller == 'rpp':
        params['controller_server']['ros__parameters']['FollowPath'] = yaml.safe_load(
            (own / 'config/rpp.yaml').read_text())
        # Leave room for AMCL updates during final rotation and the two-second
        # post-task check, whose independent position limit remains 0.25 m.
        params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = 0.20
    if LaunchConfiguration('strict_goal').perform(context) == 'true':
        # The occupied-goal fixture must not accept a nearby alternative endpoint.
        params['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.0
    record = LaunchConfiguration('params_output').perform(context)
    if record:
        params_file = Path(record)
    else:
        with tempfile.NamedTemporaryFile(prefix='leap-nav-', suffix='.yaml', delete=False) as file:
            params_file = Path(file.name)
    params_file.write_text(yaml.safe_dump(params), encoding='utf-8')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2 / 'launch/bringup_launch.py')),
        launch_arguments={
            'map': str(tb3_nav / 'map/map.yaml'), 'params_file': str(params_file),
            'use_sim_time': 'true', 'autostart': 'true',
            'use_composition': 'False', 'slam': 'False',
        }.items())]


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Simulation requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    own = Path(get_package_share_directory('leap_nav_demo'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('start_x', default_value='-2.0'),
        DeclareLaunchArgument('start_y', default_value='-0.5'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('controller', default_value='dwb'),
        DeclareLaunchArgument('params_output', default_value=''),
        DeclareLaunchArgument('strict_goal', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(own / 'launch/simulation.launch.py')),
            launch_arguments={name: LaunchConfiguration(name)
                              for name in ('gui', 'start_x', 'start_y', 'start_yaw')}.items()),
        OpaqueFunction(function=navigation_nodes),
    ])
