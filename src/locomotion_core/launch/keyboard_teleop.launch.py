from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_prefix = LaunchConfiguration('teleop_prefix')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop_prefix',
            default_value='',
            description='Optional terminal prefix for keyboard input, for example "xterm -e"',
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.3',
            description='Default linear speed used by teleop_twist_keyboard',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.3',
            description='Default angular speed used by teleop_twist_keyboard',
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_teleop',
            output='screen',
            prefix=teleop_prefix,
            remappings=[
                ('cmd_vel', '/rov_cmd_vel'),
            ],
            parameters=[{
                'speed': linear_speed,
                'turn': angular_speed,
            }],
        ),
        Node(
            package='locomotion_core',
            executable='rover_velocity',
            name='rover_velocity',
            output='screen',
        ),
        Node(
            package='locomotion_core',
            executable='cmd_roboteq',
            name='cmd_roboteq',
            output='screen',
        ),
    ])
