from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_yaw = LaunchConfiguration('target_yaw')
    kp_x = LaunchConfiguration('kp_x')
    kp_y = LaunchConfiguration('kp_y')
    kp_yaw = LaunchConfiguration('kp_yaw')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    yaw_linear_gate = LaunchConfiguration('yaw_linear_gate')
    log_positions = LaunchConfiguration('log_positions')
    start_locomotion = LaunchConfiguration('start_locomotion')

    uwb_distance_config = PathJoinSubstitution([
        FindPackageShare('uwb_position_publisher'),
        'config',
        'uwb_distance_publisher.yaml',
    ])
    uwb_position_config = PathJoinSubstitution([
        FindPackageShare('uwb_position_publisher'),
        'config',
        'uwb_position_publisher.yaml',
    ])
    uwb_position_logger_config = PathJoinSubstitution([
        FindPackageShare('uwb_position_publisher'),
        'config',
        'uwb_position_logger.yaml',
    ])
    zed_heading_config = PathJoinSubstitution([
        FindPackageShare('zed_heading_publisher'),
        'config',
        'zed_heading_publisher.yaml',
    ])
    pose_ctrl_config = PathJoinSubstitution([
        FindPackageShare('uwb_position_zed_pose_ctrl'),
        'config',
        'uwb_position_zed_pose_ctrl.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_x',
            default_value='-1.0',
            description='Target UWB/world x position in meters',
        ),
        DeclareLaunchArgument(
            'target_y',
            default_value='-1.0',
            description='Target UWB/world y position in meters',
        ),
        DeclareLaunchArgument(
            'target_yaw',
            default_value='0.0',
            description='Target yaw in radians',
        ),
        DeclareLaunchArgument(
            'kp_x',
            default_value='0.4',
            description='Proportional gain for body forward/backward control',
        ),
        DeclareLaunchArgument(
            'kp_y',
            default_value='0.4',
            description='Proportional gain for body lateral control',
        ),
        DeclareLaunchArgument(
            'kp_yaw',
            default_value='0.8',
            description='Proportional gain for yaw control',
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.5',
            description='Maximum body linear speed command',
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='0.5',
            description='Maximum yaw speed command',
        ),
        DeclareLaunchArgument(
            'yaw_linear_gate',
            default_value='0.35',
            description='Yaw error above this value suppresses linear commands',
        ),
        DeclareLaunchArgument(
            'log_positions',
            default_value='true',
            description='Start uwb_position_logger when true',
        ),
        DeclareLaunchArgument(
            'start_locomotion',
            default_value='true',
            description='Start rover_velocity and cmd_roboteq when true',
        ),
        Node(
            package='uwb_position_publisher',
            executable='uwb_distance_publisher',
            name='uwb_distance_publisher',
            output='screen',
            parameters=[uwb_distance_config],
        ),
        Node(
            package='uwb_position_publisher',
            executable='uwb_position_publisher',
            name='uwb_position_publisher',
            output='screen',
            parameters=[uwb_position_config],
        ),
        Node(
            package='zed_heading_publisher',
            executable='zed_heading_publisher',
            name='zed_heading_publisher',
            output='screen',
            parameters=[zed_heading_config],
        ),
        Node(
            package='uwb_position_zed_pose_ctrl',
            executable='uwb_position_zed_pose_ctrl',
            name='uwb_position_zed_pose_ctrl',
            output='screen',
            parameters=[
                pose_ctrl_config,
                {
                    'target_x': target_x,
                    'target_y': target_y,
                    'target_yaw': target_yaw,
                    'kp_x': kp_x,
                    'kp_y': kp_y,
                    'kp_yaw': kp_yaw,
                    'max_linear_speed': max_linear_speed,
                    'max_angular_speed': max_angular_speed,
                    'yaw_linear_gate': yaw_linear_gate,
                },
            ],
        ),
        Node(
            package='uwb_position_publisher',
            executable='uwb_position_logger',
            name='uwb_position_logger',
            output='screen',
            parameters=[uwb_position_logger_config],
            condition=IfCondition(log_positions),
        ),
        Node(
            package='locomotion_core',
            executable='rover_velocity',
            name='rover_velocity',
            output='screen',
            condition=IfCondition(start_locomotion),
        ),
        Node(
            package='locomotion_core',
            executable='cmd_roboteq',
            name='cmd_roboteq',
            output='screen',
            condition=IfCondition(start_locomotion),
        ),
    ])
