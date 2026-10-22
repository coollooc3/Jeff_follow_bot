from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route = str(Path(get_package_share_directory('leap_patrol')) / 'config/route.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('route_file', default_value=route),
        Node(package='leap_patrol', executable='patrol_manager',
             name='patrol_manager', output='screen',
             parameters=[{'use_sim_time': True,
                          'route_file': LaunchConfiguration('route_file')}]),
    ])
