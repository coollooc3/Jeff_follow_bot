"""Start the existing simulation, SLAM Toolbox and a mapping-only RViz view."""
import os
from pathlib import Path
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Mapping requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    with socket.socket() as probe:
        probe.settimeout(0.2)
        if probe.connect_ex(('127.0.0.1', 11365)) == 0:
            raise RuntimeError('Close the previous run_manual/Gazebo session before mapping')

    own = Path(get_package_share_directory('leap_mapping'))
    simulation = Path(get_package_share_directory('leap_nav_demo')) / 'launch/simulation.launch.py'
    upstream = Path(get_package_share_directory('slam_toolbox')) / 'config/mapper_params_online_async.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(simulation)),
                                 launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[str(upstream), str(own / 'config/mapping.yaml')], output='screen'),
        Node(package='rviz2', executable='rviz2', name='mapping_view',
             arguments=['-d', str(own / 'rviz/mapping.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
