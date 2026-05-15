from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_side = LaunchConfiguration('camera_side')
    marker_length = LaunchConfiguration('marker_length')
    target_distance = LaunchConfiguration('target_distance')
    align_distance = LaunchConfiguration('align_distance')
    minimum_safe_z = LaunchConfiguration('minimum_safe_z')
    align_hysteresis = LaunchConfiguration('align_hysteresis')
    target_z = LaunchConfiguration('target_z')
    z_tolerance = LaunchConfiguration('z_tolerance')
    target_x = LaunchConfiguration('target_x')
    x_tolerance = LaunchConfiguration('x_tolerance')
    target_yaw = LaunchConfiguration('target_yaw')
    yaw_tolerance = LaunchConfiguration('yaw_tolerance')
    final_x_realign_threshold = LaunchConfiguration('final_x_realign_threshold')
    final_yaw_realign_threshold = LaunchConfiguration('final_yaw_realign_threshold')
    theta_x_slow_limit = LaunchConfiguration('theta_x_slow_limit')
    theta_x_stop_limit = LaunchConfiguration('theta_x_stop_limit')
    theta_y_slow_limit = LaunchConfiguration('theta_y_slow_limit')
    theta_y_stop_limit = LaunchConfiguration('theta_y_stop_limit')
    kp_lateral = LaunchConfiguration('kp_lateral')
    kp_yaw = LaunchConfiguration('kp_yaw')
    kp_far_center = LaunchConfiguration('kp_far_center')
    kp_visibility_recovery = LaunchConfiguration('kp_visibility_recovery')
    far_approach_speed = LaunchConfiguration('far_approach_speed')
    reduced_far_approach_speed = LaunchConfiguration('reduced_far_approach_speed')
    final_approach_speed = LaunchConfiguration('final_approach_speed')
    min_far_center_speed = LaunchConfiguration('min_far_center_speed')
    max_far_center_speed = LaunchConfiguration('max_far_center_speed')
    min_visibility_recovery_speed = LaunchConfiguration('min_visibility_recovery_speed')
    max_visibility_recovery_speed = LaunchConfiguration('max_visibility_recovery_speed')
    min_lateral_align_speed = LaunchConfiguration('min_lateral_align_speed')
    max_lateral_align_speed = LaunchConfiguration('max_lateral_align_speed')
    min_yaw_align_speed = LaunchConfiguration('min_yaw_align_speed')
    max_yaw_align_speed = LaunchConfiguration('max_yaw_align_speed')
    detection_timeout = LaunchConfiguration('detection_timeout')
    hold_duration = LaunchConfiguration('hold_duration')
    control_rate = LaunchConfiguration('control_rate')
    log_motor_commands = LaunchConfiguration('log_motor_commands')
    log_zero_motor_commands = LaunchConfiguration('log_zero_motor_commands')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_side',
            default_value='left',
            description='ZED2 camera side used for marker detection: left or right',
        ),
        DeclareLaunchArgument(
            'marker_length',
            default_value='0.168',
            description='ArUco marker side length in meters',
        ),
        DeclareLaunchArgument(
            'target_distance',
            default_value='1.0',
            description='Final Euclidean docking distance in meters',
        ),
        DeclareLaunchArgument(
            'align_distance',
            default_value='1.2',
            description='Euclidean distance where near alignment begins',
        ),
        DeclareLaunchArgument(
            'minimum_safe_z',
            default_value='1.0',
            description='Forward-axis safety boundary in meters',
        ),
        DeclareLaunchArgument(
            'align_hysteresis',
            default_value='0.10',
            description='Hysteresis before returning from NEAR_ALIGN to FAR_GUIDED_APPROACH',
        ),
        DeclareLaunchArgument(
            'target_z',
            default_value='1.0',
            description='Final forward-axis docking distance in meters',
        ),
        DeclareLaunchArgument(
            'z_tolerance',
            default_value='0.03',
            description='Distance error tolerance in meters',
        ),
        DeclareLaunchArgument(
            'target_x',
            default_value='0.0',
            description='Target lateral marker offset in meters',
        ),
        DeclareLaunchArgument(
            'x_tolerance',
            default_value='0.05',
            description='Lateral error tolerance in meters',
        ),
        DeclareLaunchArgument(
            'target_yaw',
            default_value='0.0',
            description='Target marker yaw angle in radians',
        ),
        DeclareLaunchArgument(
            'yaw_tolerance',
            default_value='0.06',
            description='Yaw error tolerance in radians',
        ),
        DeclareLaunchArgument(
            'final_x_realign_threshold',
            default_value='0.08',
            description='Lateral error that returns FINAL_APPROACH to NEAR_ALIGN',
        ),
        DeclareLaunchArgument(
            'final_yaw_realign_threshold',
            default_value='0.10',
            description='Yaw error that returns FINAL_APPROACH to NEAR_ALIGN',
        ),
        DeclareLaunchArgument(
            'theta_x_slow_limit',
            default_value='0.15',
            description='Horizontal visibility angle where far approach slows',
        ),
        DeclareLaunchArgument(
            'theta_x_stop_limit',
            default_value='0.25',
            description='Horizontal visibility angle where RECOVER_VISIBILITY begins',
        ),
        DeclareLaunchArgument(
            'theta_y_slow_limit',
            default_value='0.15',
            description='Vertical visibility angle where far approach slows',
        ),
        DeclareLaunchArgument(
            'theta_y_stop_limit',
            default_value='0.25',
            description='Vertical visibility angle where controller holds',
        ),
        DeclareLaunchArgument(
            'kp_lateral',
            default_value='0.4',
            description='Proportional gain for near lateral alignment',
        ),
        DeclareLaunchArgument(
            'kp_yaw',
            default_value='0.6',
            description='Proportional gain for near yaw alignment',
        ),
        DeclareLaunchArgument(
            'kp_far_center',
            default_value='0.3',
            description='Proportional gain for far marker-centering yaw',
        ),
        DeclareLaunchArgument(
            'kp_visibility_recovery',
            default_value='0.3',
            description='Proportional gain for RECOVER_VISIBILITY yaw',
        ),
        DeclareLaunchArgument(
            'far_approach_speed',
            default_value='0.3',
            description='Forward command in FAR_GUIDED_APPROACH',
        ),
        DeclareLaunchArgument(
            'reduced_far_approach_speed',
            default_value='0.3',
            description='Reduced forward command near visibility slow limits',
        ),
        DeclareLaunchArgument(
            'final_approach_speed',
            default_value='0.3',
            description='Forward command in FINAL_APPROACH',
        ),
        DeclareLaunchArgument(
            'min_far_center_speed',
            default_value='0.3',
            description='Minimum yaw command in FAR_GUIDED_APPROACH outside tolerance',
        ),
        DeclareLaunchArgument(
            'max_far_center_speed',
            default_value='0.95',
            description='Maximum yaw command in FAR_GUIDED_APPROACH',
        ),
        DeclareLaunchArgument(
            'min_visibility_recovery_speed',
            default_value='0.3',
            description='Minimum yaw command in RECOVER_VISIBILITY',
        ),
        DeclareLaunchArgument(
            'max_visibility_recovery_speed',
            default_value='0.95',
            description='Maximum yaw command in RECOVER_VISIBILITY',
        ),
        DeclareLaunchArgument(
            'min_lateral_align_speed',
            default_value='0.3',
            description='Minimum lateral command in NEAR_ALIGN outside tolerance',
        ),
        DeclareLaunchArgument(
            'max_lateral_align_speed',
            default_value='0.95',
            description='Maximum lateral command in NEAR_ALIGN',
        ),
        DeclareLaunchArgument(
            'min_yaw_align_speed',
            default_value='0.3',
            description='Minimum yaw command in NEAR_ALIGN outside tolerance',
        ),
        DeclareLaunchArgument(
            'max_yaw_align_speed',
            default_value='0.95',
            description='Maximum yaw command in NEAR_ALIGN',
        ),
        DeclareLaunchArgument(
            'detection_timeout',
            default_value='0.5',
            description='Marker timeout in seconds',
        ),
        DeclareLaunchArgument(
            'hold_duration',
            default_value='0.8',
            description='Bounded HOLD duration in seconds',
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='20.0',
            description='Controller update rate in Hz',
        ),
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
            package='aruco_distance_publisher',
            executable='aruco_distance_publisher',
            name='aruco_distance_publisher',
            output='screen',
            parameters=[{
                'camera_side': camera_side,
                'marker_length': marker_length,
            }],
        ),
        Node(
            package='aruco_dist_ctrl',
            executable='aruco_distance_controller',
            name='aruco_distance_controller',
            output='screen',
            parameters=[{
                'target_distance': target_distance,
                'align_distance': align_distance,
                'minimum_safe_z': minimum_safe_z,
                'align_hysteresis': align_hysteresis,
                'target_z': target_z,
                'z_tolerance': z_tolerance,
                'target_x': target_x,
                'x_tolerance': x_tolerance,
                'target_yaw': target_yaw,
                'kp_yaw': kp_yaw,
                'yaw_tolerance': yaw_tolerance,
                'final_x_realign_threshold': final_x_realign_threshold,
                'final_yaw_realign_threshold': final_yaw_realign_threshold,
                'theta_x_slow_limit': theta_x_slow_limit,
                'theta_x_stop_limit': theta_x_stop_limit,
                'theta_y_slow_limit': theta_y_slow_limit,
                'theta_y_stop_limit': theta_y_stop_limit,
                'kp_lateral': kp_lateral,
                'kp_far_center': kp_far_center,
                'kp_visibility_recovery': kp_visibility_recovery,
                'far_approach_speed': far_approach_speed,
                'reduced_far_approach_speed': reduced_far_approach_speed,
                'final_approach_speed': final_approach_speed,
                'min_far_center_speed': min_far_center_speed,
                'max_far_center_speed': max_far_center_speed,
                'min_visibility_recovery_speed': min_visibility_recovery_speed,
                'max_visibility_recovery_speed': max_visibility_recovery_speed,
                'min_lateral_align_speed': min_lateral_align_speed,
                'max_lateral_align_speed': max_lateral_align_speed,
                'min_yaw_align_speed': min_yaw_align_speed,
                'max_yaw_align_speed': max_yaw_align_speed,
                'detection_timeout': detection_timeout,
                'hold_duration': hold_duration,
                'control_rate': control_rate,
            }],
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
            output='screen',
        ),
    ])
