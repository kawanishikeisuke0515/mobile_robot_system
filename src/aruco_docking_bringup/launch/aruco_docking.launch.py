from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_side = LaunchConfiguration('camera_side')
    marker_length = LaunchConfiguration('marker_length')
    target_z = LaunchConfiguration('target_z')
    kp_z = LaunchConfiguration('kp_z')
    min_forward_speed = LaunchConfiguration('min_forward_speed')
    max_forward_speed = LaunchConfiguration('max_forward_speed')
    z_tolerance = LaunchConfiguration('z_tolerance')
    docking_distance = LaunchConfiguration('docking_distance')
    target_x = LaunchConfiguration('target_x')
    kp_x = LaunchConfiguration('kp_x')
    min_lateral_speed = LaunchConfiguration('min_lateral_speed')
    max_lateral_speed = LaunchConfiguration('max_lateral_speed')
    x_tolerance = LaunchConfiguration('x_tolerance')
    target_yaw = LaunchConfiguration('target_yaw')
    kp_center = LaunchConfiguration('kp_center')
    center_deadband = LaunchConfiguration('center_deadband')
    min_angular_speed = LaunchConfiguration('min_angular_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')

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
            'target_z',
            default_value='1.3',
            description='Target distance from the marker in meters',
        ),
        DeclareLaunchArgument(
            'kp_z',
            default_value='0.4',
            description='Proportional gain for forward/backward distance control',
        ),
        DeclareLaunchArgument(
            'min_forward_speed',
            default_value='0.25',
            description='Minimum forward/backward command outside z_tolerance',
        ),
        DeclareLaunchArgument(
            'max_forward_speed',
            default_value='0.95',
            description='Maximum forward/backward command',
        ),
        DeclareLaunchArgument(
            'z_tolerance',
            default_value='0.01',
            description='Distance error tolerance in meters',
        ),
        DeclareLaunchArgument(
            'docking_distance',
            default_value='1.0',
            description='Final stop distance from the marker in meters',
        ),
        DeclareLaunchArgument(
            'target_x',
            default_value='0.0',
            description='Target lateral marker offset in meters',
        ),
        DeclareLaunchArgument(
            'kp_x',
            default_value='0.4',
            description='Proportional gain for lateral alignment control',
        ),
        DeclareLaunchArgument(
            'min_lateral_speed',
            default_value='0.25',
            description='Minimum lateral command outside x_tolerance',
        ),
        DeclareLaunchArgument(
            'max_lateral_speed',
            default_value='0.95',
            description='Maximum lateral command',
        ),
        DeclareLaunchArgument(
            'x_tolerance',
            default_value='0.01',
            description='Lateral error tolerance in meters',
        ),
        DeclareLaunchArgument(
            'target_yaw',
            default_value='0.0',
            description='Target marker yaw angle in radians',
        ),
        DeclareLaunchArgument(
            'kp_center',
            default_value='0.3',
            description='Proportional gain for marker image-centering angular control',
        ),
        DeclareLaunchArgument(
            'center_deadband',
            default_value='0.05',
            description='Normalized image-center error deadband for angular control',
        ),
        DeclareLaunchArgument(
            'min_angular_speed',
            default_value='0.1',
            description='Retained for compatibility; marker-centering control does not apply direct minimum speed',
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='0.5',
            description='Maximum yaw command',
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
                'target_z': target_z,
                'kp_z': kp_z,
                'min_forward_speed': min_forward_speed,
                'max_forward_speed': max_forward_speed,
                'z_tolerance': z_tolerance,
                'docking_distance': docking_distance,
                'target_x': target_x,
                'kp_x': kp_x,
                'min_lateral_speed': min_lateral_speed,
                'max_lateral_speed': max_lateral_speed,
                'x_tolerance': x_tolerance,
                'target_yaw': target_yaw,
                'kp_center': kp_center,
                'center_deadband': center_deadband,
                'min_angular_speed': min_angular_speed,
                'max_angular_speed': max_angular_speed,
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
