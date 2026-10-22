"""Reuse installed Humble components in a simulation-only ROS domain."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Set these in the calling shell, before launch creates any ROS entities.
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Simulation requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')

    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    system_models = Path('/usr/share/gazebo-11/models')
    if not (system_models / 'ground_plane/model.sdf').is_file():
        raise RuntimeError('Gazebo 11 system models are missing')

    def include(path, arguments=None, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments=(arguments or {}).items(), **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('start_x', default_value='-2.0'),
        DeclareLaunchArgument('start_y', default_value='-0.5'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11365'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join([
            str(tb3 / 'models'), str(system_models), os.environ.get('GAZEBO_MODEL_PATH', '')])),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', os.pathsep.join([
            '/usr/share/gazebo-11', os.environ.get('GAZEBO_RESOURCE_PATH', '')])),
        include(gazebo / 'launch/gzserver.launch.py', {
            'world': str(tb3 / 'worlds/turtlebot3_world.world'),
            'server_required': 'true', 'seed': '42'}),
        include(gazebo / 'launch/gzclient.launch.py', condition=IfCondition(LaunchConfiguration('gui'))),
        # The installed TB3 launcher appends '/' to even an empty frame prefix.
        # Load its URDF directly so TF frame names have no leading slash.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description':
                          (tb3 / 'urdf/turtlebot3_burger.urdf').read_text()}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=[
            '-entity', 'burger', '-file', str(tb3 / 'models/turtlebot3_burger/model.sdf'),
            '-x', LaunchConfiguration('start_x'), '-y', LaunchConfiguration('start_y'),
            '-z', '0.01', '-Y', LaunchConfiguration('start_yaw')], output='screen'),
    ])
