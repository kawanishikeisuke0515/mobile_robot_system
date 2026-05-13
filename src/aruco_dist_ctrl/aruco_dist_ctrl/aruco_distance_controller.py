import os
import math
from typing import Optional

import rclpy
from aruco_interfaces.msg import ArucoDistance
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ArucoDistanceController(Node):
    def __init__(self):
        super().__init__('aruco_distance_controller')

        self.declare_parameter('target_z', 1.0)
        self.declare_parameter('kp_z', 1.0)
        self.declare_parameter('min_forward_speed', 0.3)
        self.declare_parameter('max_forward_speed', 0.95)
        self.declare_parameter('z_tolerance', 0.03)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('kp_x', 1.0)
        self.declare_parameter('min_lateral_speed', 0.3)
        self.declare_parameter('max_lateral_speed', 0.95)
        self.declare_parameter('x_tolerance', 0.03)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('min_angular_speed', 0.1)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('detection_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)

        self.target_z = float(self.get_parameter('target_z').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.min_forward_speed = float(self.get_parameter('min_forward_speed').value)
        self.max_forward_speed = float(self.get_parameter('max_forward_speed').value)
        self.z_tolerance = float(self.get_parameter('z_tolerance').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.kp_x = float(self.get_parameter('kp_x').value)
        self.min_lateral_speed = float(self.get_parameter('min_lateral_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.x_tolerance = float(self.get_parameter('x_tolerance').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self._validate_parameters()

        self.latest_x: Optional[float] = None
        self.latest_z: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.last_detection_time: Optional[Time] = None
        self.was_timed_out = True

        self.cmd_publisher = self.create_publisher(Twist, '/rov_cmd_vel', 10)
        self.distance_subscriber = self.create_subscription(
            ArucoDistance,
            '/aruco/distance',
            self.distance_callback,
            10,
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_callback,
        )

        self.get_logger().info('subscribing to /aruco/distance')
        self.get_logger().info('publishing forward/backward, lateral, and yaw commands on /rov_cmd_vel')
        self.get_logger().info(
            'target_z=%.3f kp_z=%.3f min_forward_speed=%.3f max_forward_speed=%.3f '
            'z_tolerance=%.3f target_x=%.3f kp_x=%.3f min_lateral_speed=%.3f '
            'max_lateral_speed=%.3f x_tolerance=%.3f target_yaw=%.3f kp_yaw=%.3f '
            'min_angular_speed=%.3f max_angular_speed=%.3f yaw_tolerance=%.3f '
            'detection_timeout=%.3f control_rate=%.1f'
            % (
                self.target_z,
                self.kp_z,
                self.min_forward_speed,
                self.max_forward_speed,
                self.z_tolerance,
                self.target_x,
                self.kp_x,
                self.min_lateral_speed,
                self.max_lateral_speed,
                self.x_tolerance,
                self.target_yaw,
                self.kp_yaw,
                self.min_angular_speed,
                self.max_angular_speed,
                self.yaw_tolerance,
                self.detection_timeout,
                self.control_rate,
            )
        )

    def _validate_parameters(self):
        if self.min_forward_speed < 0.0:
            raise ValueError('min_forward_speed must be greater than or equal to 0')
        if self.max_forward_speed < 0.0:
            raise ValueError('max_forward_speed must be greater than or equal to 0')
        if self.min_forward_speed > self.max_forward_speed:
            raise ValueError('min_forward_speed must be less than or equal to max_forward_speed')
        if self.z_tolerance < 0.0:
            raise ValueError('z_tolerance must be greater than or equal to 0')
        if self.min_lateral_speed < 0.0:
            raise ValueError('min_lateral_speed must be greater than or equal to 0')
        if self.max_lateral_speed < 0.0:
            raise ValueError('max_lateral_speed must be greater than or equal to 0')
        if self.min_lateral_speed > self.max_lateral_speed:
            raise ValueError('min_lateral_speed must be less than or equal to max_lateral_speed')
        if self.x_tolerance < 0.0:
            raise ValueError('x_tolerance must be greater than or equal to 0')
        if self.min_angular_speed < 0.0:
            raise ValueError('min_angular_speed must be greater than or equal to 0')
        if self.max_angular_speed < 0.0:
            raise ValueError('max_angular_speed must be greater than or equal to 0')
        if self.min_angular_speed > self.max_angular_speed:
            raise ValueError('min_angular_speed must be less than or equal to max_angular_speed')
        if self.yaw_tolerance < 0.0:
            raise ValueError('yaw_tolerance must be greater than or equal to 0')
        if self.detection_timeout <= 0.0:
            raise ValueError('detection_timeout must be greater than 0')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be greater than 0')

    def distance_callback(self, msg: ArucoDistance):
        self.latest_x = float(msg.x)
        self.latest_z = float(msg.z)
        self.latest_yaw = float(msg.yaw)
        self.last_detection_time = self.get_clock().now()

    def control_callback(self):
        cmd = Twist()

        if self._has_recent_detection():
            self.was_timed_out = False
            cmd.linear.x = self._calculate_forward_velocity(self.latest_z)
            cmd.linear.y = self._calculate_lateral_velocity(self.latest_x)
            cmd.angular.z = self._calculate_angular_velocity(self.latest_yaw)
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            if not self.was_timed_out:
                self.get_logger().warn('ArUco detection timed out; stopping robot')
                self.was_timed_out = True

        self.cmd_publisher.publish(cmd)

    def _has_recent_detection(self) -> bool:
        if (
            self.latest_x is None
            or self.latest_z is None
            or self.latest_yaw is None
            or self.last_detection_time is None
        ):
            return False

        elapsed = self.get_clock().now() - self.last_detection_time
        return elapsed.nanoseconds * 1e-9 <= self.detection_timeout

    def _calculate_forward_velocity(self, aruco_z: float) -> float:
        error_z = aruco_z - self.target_z
        if abs(error_z) < self.z_tolerance:
            return 0.0

        velocity = self.kp_z * error_z
        velocity = _clamp(
            velocity,
            -self.max_forward_speed,
            self.max_forward_speed,
        )
        if 0.0 < abs(velocity) < self.min_forward_speed:
            velocity = self.min_forward_speed if velocity > 0.0 else -self.min_forward_speed

        return velocity

    def _calculate_angular_velocity(self, aruco_yaw: float) -> float:
        error_yaw = _wrap_pi(aruco_yaw - self.target_yaw)
        if abs(error_yaw) < self.yaw_tolerance:
            return 0.0

        velocity = self.kp_yaw * error_yaw
        velocity = _clamp(
            velocity,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if 0.0 < abs(velocity) < self.min_angular_speed:
            velocity = self.min_angular_speed if velocity > 0.0 else -self.min_angular_speed

        return velocity

    def _calculate_lateral_velocity(self, aruco_x: float) -> float:
        error_x = aruco_x - self.target_x
        if abs(error_x) < self.x_tolerance:
            return 0.0

        velocity = -self.kp_x * error_x
        velocity = _clamp(
            velocity,
            -self.max_lateral_speed,
            self.max_lateral_speed,
        )
        if 0.0 < abs(velocity) < self.min_lateral_speed:
            velocity = self.min_lateral_speed if velocity > 0.0 else -self.min_lateral_speed

        return velocity


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDistanceController()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
