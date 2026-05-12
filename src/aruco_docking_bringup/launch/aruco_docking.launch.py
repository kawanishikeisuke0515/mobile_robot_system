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
            default_value='0.5',
            description='Target distance from the marker in meters',
        ),
        DeclareLaunchArgument(
            'kp_z',
            default_value='0.4',
            description='Proportional gain for forward/backward distance control',
        ),
        DeclareLaunchArgument(
            'min_forward_speed',
            default_value='0.03',
            description='Minimum forward/backward command outside z_tolerance',
        ),
        DeclareLaunchArgument(
            'max_forward_speed',
            default_value='0.15',
            description='Maximum forward/backward command',
        ),
        DeclareLaunchArgument(
            'z_tolerance',
            default_value='0.03',
            description='Distance error tolerance in meters',
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
