"""Managed controllers and detector; sensor publishers and drivers run separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def number(name, kind=float):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    def config(package, filename):
        return PathJoinSubstitution([FindPackageShare(package), 'config', filename])

    return LaunchDescription([
        DeclareLaunchArgument('manager_config', default_value=config(
            'mode_manager', 'mode_manager.yaml')),
        DeclareLaunchArgument('uwb_controller_config', default_value=config(
            'uwb_position_zed_pose_ctrl', 'uwb_position_zed_pose_ctrl.yaml')),
        DeclareLaunchArgument('vision_config', default_value=config(
            'docking_bringup', 'vision.yaml')),
        DeclareLaunchArgument('aruco_config', default_value=config(
            'docking_bringup', 'aruco.yaml')),
        DeclareLaunchArgument('handoff_x', description='UWB world target x [m]'),
        DeclareLaunchArgument('handoff_y', description='UWB world target y [m]'),
        DeclareLaunchArgument('handoff_yaw', description='ZED target heading [rad]'),
        DeclareLaunchArgument('target_marker_id', description='Target ArUco ID'),
        DeclareLaunchArgument('vision_target_z', default_value='1.3'),
        DeclareLaunchArgument('docking_distance', default_value='1.0'),
        Node(package='mode_manager', executable='mode_manager', output='screen',
             parameters=[LaunchConfiguration('manager_config'),
                         {'target_marker_id': number('target_marker_id', int)}]),
        Node(package='uwb_position_zed_pose_ctrl', executable='uwb_position_zed_pose_ctrl',
             output='screen', parameters=[LaunchConfiguration('uwb_controller_config'), {
                 'managed_mode': True,
                 'target_x': number('handoff_x'), 'target_y': number('handoff_y'),
                 'target_yaw': number('handoff_yaw'),
             }]),
        Node(package='vision_dist_ctrl', executable='vision_distance_controller',
             output='screen', parameters=[LaunchConfiguration('vision_config'), {
                 'managed_mode': True, 'target_marker_id': number('target_marker_id', int),
                 'target_z': number('vision_target_z'),
                 'docking_distance': number('docking_distance'),
             }]),
        Node(package='aruco_distance_publisher', executable='aruco_distance_publisher',
             output='screen', parameters=[LaunchConfiguration('aruco_config')]),
    ])
