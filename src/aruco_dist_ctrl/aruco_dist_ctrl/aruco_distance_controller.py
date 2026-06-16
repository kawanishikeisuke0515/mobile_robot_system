import os
import math
from collections import deque
from typing import Deque
from typing import Optional

import rclpy
from aruco_interfaces.msg import ArucoDistance
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time


PRE_DOCKING = 'PRE_DOCKING'
FINAL_DOCKING = 'FINAL_DOCKING'


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _mean(values: Deque[float]) -> float:
    return sum(values) / len(values)


class ArucoDistanceController(Node):
    def __init__(self):
        super().__init__('aruco_distance_controller')

        self.declare_parameter('target_z', 1.3)
        self.declare_parameter('kp_z', 2.0)
        self.declare_parameter('min_forward_speed', 0.30)
        self.declare_parameter('max_forward_speed', 0.95)
        self.declare_parameter('z_tolerance', 0.01)
        self.declare_parameter('docking_distance', 1.0)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('kp_x', 0.4)
        self.declare_parameter('min_lateral_speed', 0.30)
        self.declare_parameter('max_lateral_speed', 0.95)
        self.declare_parameter('x_tolerance', 0.01)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('kp_center', 0.3)
        self.declare_parameter('center_deadband', 0.05)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('position_average_window_size', 5)
        self.declare_parameter('detection_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)

        self.target_z = float(self.get_parameter('target_z').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.min_forward_speed = float(self.get_parameter('min_forward_speed').value)
        self.max_forward_speed = float(self.get_parameter('max_forward_speed').value)
        self.z_tolerance = float(self.get_parameter('z_tolerance').value)
        self.docking_distance = float(self.get_parameter('docking_distance').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.kp_x = float(self.get_parameter('kp_x').value)
        self.min_lateral_speed = float(self.get_parameter('min_lateral_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.x_tolerance = float(self.get_parameter('x_tolerance').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.kp_center = float(self.get_parameter('kp_center').value)
        self.center_deadband = float(self.get_parameter('center_deadband').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.position_average_window_size = int(
            self.get_parameter('position_average_window_size').value
        )
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self._validate_parameters()

        self.latest_z: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.latest_estimated_x: Optional[float] = None
        self.latest_estimated_z: Optional[float] = None
        self.latest_normalized_center_error: Optional[float] = None
        self.last_detection_time: Optional[Time] = None
        self.estimated_x_buffer: Deque[float] = deque(
            maxlen=self.position_average_window_size
        )
        self.estimated_z_buffer: Deque[float] = deque(
            maxlen=self.position_average_window_size
        )
        self.was_timed_out = True
        self.state = PRE_DOCKING

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
            'z_tolerance=%.3f docking_distance=%.3f '
            'target_x=%.3f kp_x=%.3f min_lateral_speed=%.3f '
            'max_lateral_speed=%.3f x_tolerance=%.3f target_yaw=%.3f '
            'kp_center=%.3f center_deadband=%.3f max_angular_speed=%.3f '
            'position_average_window_size=%d '
            'detection_timeout=%.3f control_rate=%.1f'
            % (
                self.target_z,
                self.kp_z,
                self.min_forward_speed,
                self.max_forward_speed,
                self.z_tolerance,
                self.docking_distance,
                self.target_x,
                self.kp_x,
                self.min_lateral_speed,
                self.max_lateral_speed,
                self.x_tolerance,
                self.target_yaw,
                self.kp_center,
                self.center_deadband,
                self.max_angular_speed,
                self.position_average_window_size,
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
        if self.docking_distance < 0.0:
            raise ValueError('docking_distance must be greater than or equal to 0')
        if self.docking_distance > self.target_z:
            raise ValueError('docking_distance must be less than or equal to target_z')
        if self.min_lateral_speed < 0.0:
            raise ValueError('min_lateral_speed must be greater than or equal to 0')
        if self.max_lateral_speed < 0.0:
            raise ValueError('max_lateral_speed must be greater than or equal to 0')
        if self.min_lateral_speed > self.max_lateral_speed:
            raise ValueError('min_lateral_speed must be less than or equal to max_lateral_speed')
        if self.x_tolerance < 0.0:
            raise ValueError('x_tolerance must be greater than or equal to 0')
        if self.kp_center < 0.0:
            raise ValueError('kp_center must be greater than or equal to 0')
        if self.center_deadband < 0.0:
            raise ValueError('center_deadband must be greater than or equal to 0')
        if self.max_angular_speed < 0.0:
            raise ValueError('max_angular_speed must be greater than or equal to 0')
        if self.position_average_window_size < 1:
            raise ValueError('position_average_window_size must be greater than or equal to 1')
        if self.detection_timeout <= 0.0:
            raise ValueError('detection_timeout must be greater than 0')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be greater than 0')

    def distance_callback(self, msg: ArucoDistance):
        now = self.get_clock().now()
        if self.last_detection_time is not None:
            elapsed = now - self.last_detection_time
            if elapsed.nanoseconds * 1e-9 > self.detection_timeout:
                self._clear_position_average()

        self.latest_z = float(msg.z)
        self.latest_yaw = float(msg.yaw)
        self.latest_normalized_center_error = float(msg.normalized_center_error)
        raw_estimated_x, raw_estimated_z = self._estimate_wall_relative_position(
            self.latest_z,
            self.latest_yaw,
        )
        self.estimated_x_buffer.append(raw_estimated_x)
        self.estimated_z_buffer.append(raw_estimated_z)
        self.latest_estimated_x = _mean(self.estimated_x_buffer)
        self.latest_estimated_z = _mean(self.estimated_z_buffer)
        self.last_detection_time = now

    def control_callback(self):
        cmd = Twist()

        if self._has_recent_detection():
            self.was_timed_out = False
            if self.state == PRE_DOCKING and self._is_ready_for_final_docking():
                self.state = FINAL_DOCKING
                self.get_logger().info('PRE_DOCKING -> FINAL_DOCKING')

            if self.state == FINAL_DOCKING:
                cmd = self._calculate_final_docking_command()
            else:
                cmd = self._calculate_pre_docking_command()
        else:
            if self.state != PRE_DOCKING:
                self.state = PRE_DOCKING
                self.get_logger().warn('ArUco detection lost; returning to PRE_DOCKING')
            self._clear_position_average()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            if not self.was_timed_out:
                self.get_logger().warn('ArUco detection timed out; stopping robot')
                self.was_timed_out = True

        self.cmd_publisher.publish(cmd)

    def _has_recent_detection(self) -> bool:
        if (
            self.latest_z is None
            or self.latest_yaw is None
            or self.latest_estimated_x is None
            or self.latest_estimated_z is None
            or self.latest_normalized_center_error is None
            or self.last_detection_time is None
        ):
            return False

        elapsed = self.get_clock().now() - self.last_detection_time
        return elapsed.nanoseconds * 1e-9 <= self.detection_timeout

    def _calculate_pre_docking_command(self) -> Twist:
        cmd = Twist()
        cmd.linear.x = self._calculate_forward_velocity(self.latest_estimated_z)
        cmd.linear.y = self._calculate_lateral_velocity(self.latest_estimated_x)
        cmd.angular.z = self._calculate_angular_velocity(
            self.latest_normalized_center_error,
        )
        return cmd

    def _calculate_final_docking_command(self) -> Twist:
        cmd = Twist()
        if self.latest_estimated_z <= self.docking_distance:
            return cmd

        cmd.linear.x = self._calculate_final_forward_velocity(self.latest_estimated_z)
        cmd.linear.y = self._calculate_lateral_velocity(self.latest_estimated_x)
        cmd.angular.z = self._calculate_angular_velocity(
            self.latest_normalized_center_error,
        )
        return cmd

    def _calculate_final_forward_velocity(self, estimated_z: float) -> float:
        error_z = estimated_z - self.docking_distance
        if error_z <= 0.0:
            return 0.0

        velocity = self.kp_z * error_z
        velocity = _clamp(
            velocity,
            0.0,
            self.max_forward_speed,
        )
        if 0.0 < velocity < self.min_forward_speed:
            velocity = self.min_forward_speed

        return velocity

    def _is_ready_for_final_docking(self) -> bool:
        forward_error = self.latest_estimated_z - self.target_z
        lateral_error = self.latest_estimated_x - self.target_x
        return (
            abs(forward_error) < self.z_tolerance
            and abs(lateral_error) < self.x_tolerance
            and abs(self.latest_normalized_center_error) < self.center_deadband
        )

    def _estimate_wall_relative_position(
        self,
        aruco_z: float,
        aruco_yaw: float,
    ) -> tuple[float, float]:
        theta = _wrap_pi(aruco_yaw - self.target_yaw)
        estimated_x = aruco_z * math.sin(theta)
        estimated_z = aruco_z * math.cos(theta)
        return estimated_x, estimated_z

    def _clear_position_average(self):
        self.estimated_x_buffer.clear()
        self.estimated_z_buffer.clear()
        self.latest_estimated_x = None
        self.latest_estimated_z = None

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

    def _calculate_angular_velocity(
        self,
        normalized_center_error: float,
    ) -> float:
        angular_error, angular_enabled = self._calculate_angular_control(
            normalized_center_error,
        )
        if not angular_enabled:
            return 0.0
        if abs(angular_error) < self.center_deadband:
            return 0.0

        velocity = self.kp_center * angular_error
        velocity = _clamp(
            velocity,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        return velocity

    def _calculate_angular_control(
        self,
        normalized_center_error: float,
    ) -> tuple[float, bool]:
        if abs(normalized_center_error) < self.center_deadband:
            return 0.0, False

        return -normalized_center_error, True

    def _calculate_lateral_velocity(self, estimated_x: float) -> float:
        error_x = estimated_x - self.target_x
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
