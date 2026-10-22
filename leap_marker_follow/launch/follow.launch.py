from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('leap_marker_follow')) / 'config/follow.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        Node(package='leap_marker_follow', executable='marker_follow',
             name='marker_follow', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
