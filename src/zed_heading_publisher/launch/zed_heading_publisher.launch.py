from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('zed_heading_publisher'))
    config_path = package_share / 'config' / 'zed_heading_publisher.yaml'
    mag_topic = LaunchConfiguration('mag_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mag_topic',
            default_value='/zed2i/zed_node/imu/mag',
            description='MagneticField topic published by the ZED data hub.',
        ),
        Node(
            package='zed_heading_publisher',
            executable='zed_heading_publisher',
            name='zed_heading_publisher',
            output='screen',
            parameters=[
                str(config_path),
                {
                    'mag_topic': mag_topic,
                },
            ],
        ),
    ])
