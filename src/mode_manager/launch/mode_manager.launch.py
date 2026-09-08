from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default = os.path.join(get_package_share_directory('mode_manager'),
                           'config', 'mode_manager.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        Node(package='mode_manager', executable='mode_manager',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
