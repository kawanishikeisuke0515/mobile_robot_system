"""Complete UWB/ZED/Vision docking stack, automatic start after controller and UWB readiness."""
import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def config(package, filename):
        return PathJoinSubstitution([FindPackageShare(package), 'config', filename])

    def include(package, filename, arguments, condition=None):
        return GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), 'launch', filename])),
            launch_arguments=arguments.items(),
        )], condition=condition)

    configs = {
        'uwb_distance_config': ('uwb_position_publisher', 'uwb_distance_publisher.yaml'),
        'uwb_position_config': ('uwb_position_publisher', 'uwb_position_publisher.yaml'),
        'heading_config': ('zed_heading_publisher', 'zed_heading_publisher.yaml'),
        'zed_config': ('zed_wrapper_data_hub', 'zed2i_data_hub.yaml'),
        'manager_config': ('mode_manager', 'mode_manager.yaml'),
        'uwb_controller_config': ('uwb_position_zed_pose_ctrl', 'uwb_position_zed_pose_ctrl.yaml'),
        'vision_config': ('docking_bringup', 'vision.yaml'),
        'aruco_config': ('docking_bringup', 'aruco.yaml'),
    }
    declarations = [DeclareLaunchArgument(
        name, default_value=config(package, filename), description=f'Parameter YAML: {name}')
        for name, (package, filename) in configs.items()]
    configs['logger_config'] = ('docking_logger', 'docking_logger.yaml')
    declarations.append(DeclareLaunchArgument(
        'logger_config', default_value=config('docking_logger', 'docking_logger.yaml')))
    declarations.extend([
        DeclareLaunchArgument('enable_logging', default_value='true'),
        DeclareLaunchArgument('log_dir', default_value='~/docking_logs'),
        DeclareLaunchArgument('experiment_name', default_value='docking'),
        DeclareLaunchArgument('experiment_note', default_value=''),
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='Start once controllers and UWB pose are ready'),
        DeclareLaunchArgument('handoff_x', description='Handoff UWB/world x [m]'),
        DeclareLaunchArgument('handoff_y', description='Handoff UWB/world y [m]'),
        DeclareLaunchArgument('handoff_yaw', description='Handoff ZED heading [rad]'),
        DeclareLaunchArgument('target_marker_id', description='Target ArUco ID'),
        DeclareLaunchArgument('vision_target_z', default_value='1.3',
                              description='Vision pre-docking distance [m]'),
        DeclareLaunchArgument('docking_distance', default_value='1.0',
                              description='Vision final stop distance [m]'),
        DeclareLaunchArgument('start_zed', default_value='true',
                              description='Start the ZED camera data hub'),
        DeclareLaunchArgument('start_uwb', default_value='true',
                              description='Start UWB serial reader and position publisher'),
        DeclareLaunchArgument('start_heading', default_value='true',
                              description='Start ZED magnetic heading publisher'),
        DeclareLaunchArgument('start_locomotion', default_value='true',
                              description='Start rover_velocity and cmd_roboteq motor driver'),
        DeclareLaunchArgument('zed_serial_number', default_value='0',
                              description='ZED camera serial number; 0 selects wrapper default'),
    ])
    managed_args = {name: LaunchConfiguration(name) for name in (
        'handoff_x', 'handoff_y', 'handoff_yaw', 'target_marker_id',
        'vision_target_z', 'docking_distance', 'manager_config', 'auto_start',
        'uwb_controller_config', 'vision_config', 'aruco_config')}
    def start_logger(context):
        def value(name):
            return LaunchConfiguration(name).perform(context)

        paths = {name: value(name) for name in configs}
        overrides = {name: value(name) for name in (
            'auto_start', 'handoff_x', 'handoff_y', 'handoff_yaw', 'target_marker_id',
            'vision_target_z', 'docking_distance', 'start_zed', 'start_uwb',
            'start_heading', 'start_locomotion', 'zed_serial_number')}
        return [Node(
            package='docking_logger', executable='docking_logger', name='docking_logger',
            output='screen', parameters=[paths['logger_config'], {
                'target_marker_id': int(value('target_marker_id')),
                'log_dir': ParameterValue(value('log_dir'), value_type=str),
                'experiment_name': ParameterValue(value('experiment_name'), value_type=str),
                'experiment_note': ParameterValue(value('experiment_note'), value_type=str),
                'config_paths_json': ParameterValue(json.dumps(paths), value_type=str),
                'launch_overrides_json': ParameterValue(json.dumps(overrides), value_type=str),
            }])]

    return LaunchDescription(declarations + [
        OpaqueFunction(function=start_logger, condition=IfCondition(LaunchConfiguration('enable_logging'))),
        include('zed_wrapper_data_hub', 'zed_data_hub.launch.py', {
            'camera_name': 'zed2i', 'camera_model': 'zed2i',
            'namespace': '', 'node_name': 'zed_node',
            'serial_number': LaunchConfiguration('zed_serial_number'),
            'ros_params_override_path': LaunchConfiguration('zed_config'),
        }, IfCondition(LaunchConfiguration('start_zed'))),
        Node(package='uwb_position_publisher', executable='uwb_distance_publisher',
             name='uwb_distance_publisher', output='screen',
             parameters=[LaunchConfiguration('uwb_distance_config')],
             condition=IfCondition(LaunchConfiguration('start_uwb'))),
        Node(package='uwb_position_publisher', executable='uwb_position_publisher',
             name='uwb_position_publisher', output='screen',
             parameters=[LaunchConfiguration('uwb_position_config')],
             condition=IfCondition(LaunchConfiguration('start_uwb'))),
        Node(package='zed_heading_publisher', executable='zed_heading_publisher',
             name='zed_heading_publisher', output='screen',
             parameters=[LaunchConfiguration('heading_config')],
             condition=IfCondition(LaunchConfiguration('start_heading'))),
        include('docking_bringup', 'managed_docking.launch.py', managed_args),
        Node(package='locomotion_core', executable='rover_velocity',
             name='rover_velocity', output='screen',
             condition=IfCondition(LaunchConfiguration('start_locomotion'))),
        Node(package='locomotion_core', executable='cmd_roboteq',
             name='cmd_roboteq', output='screen',
             condition=IfCondition(LaunchConfiguration('start_locomotion'))),
    ])
