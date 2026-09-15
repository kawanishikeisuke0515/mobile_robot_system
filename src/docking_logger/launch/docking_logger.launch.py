"""Standalone telemetry recording; starts no controllers or sensors."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('logger_config', default_value=PathJoinSubstitution([
            FindPackageShare('docking_logger'), 'config', 'docking_logger.yaml'])),
        DeclareLaunchArgument('target_marker_id'),
        DeclareLaunchArgument('log_dir', default_value='~/docking_logs'),
        DeclareLaunchArgument('experiment_name', default_value='docking'),
        DeclareLaunchArgument('experiment_note', default_value=''),
    ]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=str)
              for k in ('log_dir', 'experiment_name', 'experiment_note')}
    params['target_marker_id'] = ParameterValue(LaunchConfiguration('target_marker_id'), value_type=int)
    return LaunchDescription(declarations + [Node(
        package='docking_logger', executable='docking_logger', name='docking_logger',
        output='screen', parameters=[LaunchConfiguration('logger_config'), params])])
