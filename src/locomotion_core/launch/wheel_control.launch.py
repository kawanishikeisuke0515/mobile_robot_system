from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_motor_commands = LaunchConfiguration('log_motor_commands')
    log_zero_motor_commands = LaunchConfiguration('log_zero_motor_commands')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_motor_commands',
            default_value='false',
            description='Log rover_velocity motor command vectors',
        ),
        DeclareLaunchArgument(
            'log_zero_motor_commands',
            default_value='false',
            description='Log zero motor command vectors when motor command logging is enabled',
        ),
        Node(
            package='locomotion_core',
            executable='rover_velocity',
            name='rover_velocity',
            output='screen',
            parameters=[{
                'log_motor_commands': log_motor_commands,
                'log_zero_motor_commands': log_zero_motor_commands,
            }],
        ),
        Node(
            package='locomotion_core',
            executable='cmd_roboteq',
            name='cmd_roboteq',
            output='screen'
        )
    ])
