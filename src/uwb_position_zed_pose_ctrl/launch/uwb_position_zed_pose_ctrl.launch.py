from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('uwb_position_zed_pose_ctrl'),
                'config',
                'uwb_position_zed_pose_ctrl.yaml',
            ]),
            description='YAML config file for uwb_position_zed_pose_ctrl',
        ),
        Node(
            package='uwb_position_zed_pose_ctrl',
            executable='uwb_position_zed_pose_ctrl',
            name='uwb_position_zed_pose_ctrl',
            output='screen',
            parameters=[config_file],
        ),
    ])
