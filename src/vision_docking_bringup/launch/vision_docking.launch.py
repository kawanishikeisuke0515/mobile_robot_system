from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_side = LaunchConfiguration('camera_side')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    use_camera_info = LaunchConfiguration('use_camera_info')
    use_rectified_camera_info = LaunchConfiguration('use_rectified_camera_info')
    use_camera_model_center_error = LaunchConfiguration('use_camera_model_center_error')
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
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    position_average_window_size = LaunchConfiguration('position_average_window_size')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_side',
            default_value='left',
            description='Calibration side used for marker pose estimation: left or right',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed2i/zed_node/rgb/color/rect/image',
            description='Image topic used for marker detection',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/zed2i/zed_node/rgb/color/rect/camera_info',
            description='CameraInfo topic used when use_camera_info is true',
        ),
        DeclareLaunchArgument(
            'use_camera_info',
            default_value='true',
            description='Use CameraInfo calibration instead of calibration file',
        ),
        DeclareLaunchArgument(
            'use_rectified_camera_info',
            default_value='true',
            description='Use CameraInfo projection matrix for rectified image input',
        ),
        DeclareLaunchArgument(
            'use_camera_model_center_error',
            default_value='true',
            description='Use camera cx/fx for image-center error calculation',
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
            default_value='2.0',
            description='Proportional gain for forward/backward distance control',
        ),
        DeclareLaunchArgument(
            'min_forward_speed',
            default_value='0.30',
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
            default_value='0.30',
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
            'max_angular_speed',
            default_value='0.5',
            description='Maximum yaw command',
        ),
        DeclareLaunchArgument(
            'position_average_window_size',
            default_value='5',
            description='Moving-average window size for wall-relative estimated_x and estimated_z',
        ),
        Node(
            package='aruco_distance_publisher',
            executable='aruco_distance_publisher',
            name='aruco_distance_publisher',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'use_camera_info': use_camera_info,
                'use_rectified_camera_info': use_rectified_camera_info,
                'use_camera_model_center_error': use_camera_model_center_error,
                'camera_side': camera_side,
                'marker_length': marker_length,
            }],
        ),
        Node(
            package='vision_dist_ctrl',
            executable='vision_distance_controller',
            name='vision_distance_controller',
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
                'max_angular_speed': max_angular_speed,
                'position_average_window_size': position_average_window_size,
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
