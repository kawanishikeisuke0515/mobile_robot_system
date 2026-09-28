from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_motor_driver', default_value='true',
            description='Start the existing cmd_roboteq motor driver.'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('locomotion_closed_loop'), 'config', 'closed_loop_velocity.yaml'])),
        Node(package='locomotion_closed_loop', executable='closed_loop_velocity',
             name='closed_loop_velocity', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='locomotion_core', executable='cmd_roboteq',
             name='cmd_roboteq', output='screen',
             condition=IfCondition(LaunchConfiguration('start_motor_driver'))),
    ])
