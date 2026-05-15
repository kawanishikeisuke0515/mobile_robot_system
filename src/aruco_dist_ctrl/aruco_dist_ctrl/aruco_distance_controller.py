import os
import math
from typing import Optional

import rclpy
from aruco_interfaces.msg import ArucoDistance
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


STATE_WAIT_FOR_MARKER = 'WAIT_FOR_MARKER'
STATE_FAR_GUIDED_APPROACH = 'FAR_GUIDED_APPROACH'
STATE_NEAR_ALIGN = 'NEAR_ALIGN'
STATE_FINAL_APPROACH = 'FINAL_APPROACH'
STATE_HOLD = 'HOLD'
STATE_DOCKED = 'DOCKED'


class ArucoDistanceController(Node):
    def __init__(self):
        super().__init__('aruco_distance_controller')

        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('align_distance', 1.2)
        self.declare_parameter('minimum_safe_z', 1.0)
        self.declare_parameter('align_hysteresis', 0.10)
        self.declare_parameter('target_z', 1.0)
        self.declare_parameter('z_tolerance', 0.03)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('x_tolerance', 0.05)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('yaw_tolerance', 0.06)
        self.declare_parameter('final_x_realign_threshold', 0.08)
        self.declare_parameter('final_yaw_realign_threshold', 0.10)
        self.declare_parameter('theta_x_slow_limit', 0.15)
        self.declare_parameter('theta_x_stop_limit', 0.25)
        self.declare_parameter('theta_y_slow_limit', 0.15)
        self.declare_parameter('theta_y_stop_limit', 0.25)
        self.declare_parameter('kp_lateral', 0.4)
        self.declare_parameter('kp_yaw', 0.6)
        self.declare_parameter('kp_far_center', 0.3)
        self.declare_parameter('far_approach_speed', 0.3)
        self.declare_parameter('reduced_far_approach_speed', 0.3)
        self.declare_parameter('final_approach_speed', 0.3)
        self.declare_parameter('min_far_center_speed', 0.3)
        self.declare_parameter('max_far_center_speed', 0.95)
        self.declare_parameter('min_lateral_align_speed', 0.3)
        self.declare_parameter('max_lateral_align_speed', 0.95)
        self.declare_parameter('min_yaw_align_speed', 0.3)
        self.declare_parameter('max_yaw_align_speed', 0.95)
        self.declare_parameter('detection_timeout', 0.5)
        self.declare_parameter('hold_duration', 0.8)
        self.declare_parameter('control_rate', 20.0)

        self.target_distance = float(self.get_parameter('target_distance').value)
        self.align_distance = float(self.get_parameter('align_distance').value)
        self.minimum_safe_z = float(self.get_parameter('minimum_safe_z').value)
        self.align_hysteresis = float(self.get_parameter('align_hysteresis').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.z_tolerance = float(self.get_parameter('z_tolerance').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.x_tolerance = float(self.get_parameter('x_tolerance').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.final_x_realign_threshold = float(self.get_parameter('final_x_realign_threshold').value)
        self.final_yaw_realign_threshold = float(self.get_parameter('final_yaw_realign_threshold').value)
        self.theta_x_slow_limit = float(self.get_parameter('theta_x_slow_limit').value)
        self.theta_x_stop_limit = float(self.get_parameter('theta_x_stop_limit').value)
        self.theta_y_slow_limit = float(self.get_parameter('theta_y_slow_limit').value)
        self.theta_y_stop_limit = float(self.get_parameter('theta_y_stop_limit').value)
        self.kp_lateral = float(self.get_parameter('kp_lateral').value)
        self.kp_far_center = float(self.get_parameter('kp_far_center').value)
        self.far_approach_speed = float(self.get_parameter('far_approach_speed').value)
        self.reduced_far_approach_speed = float(self.get_parameter('reduced_far_approach_speed').value)
        self.final_approach_speed = float(self.get_parameter('final_approach_speed').value)
        self.min_far_center_speed = float(self.get_parameter('min_far_center_speed').value)
        self.max_far_center_speed = float(self.get_parameter('max_far_center_speed').value)
        self.min_lateral_align_speed = float(self.get_parameter('min_lateral_align_speed').value)
        self.max_lateral_align_speed = float(self.get_parameter('max_lateral_align_speed').value)
        self.min_yaw_align_speed = float(self.get_parameter('min_yaw_align_speed').value)
        self.max_yaw_align_speed = float(self.get_parameter('max_yaw_align_speed').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.hold_duration = float(self.get_parameter('hold_duration').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self._validate_parameters()

        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.latest_z: Optional[float] = None
        self.latest_distance: Optional[float] = None
        self.latest_theta: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.last_detection_time: Optional[Time] = None
        self.was_timed_out = True
        self.state = STATE_WAIT_FOR_MARKER
        self.hold_start_time: Optional[Time] = None

        self.cmd_publisher = self.create_publisher(Twist, '/rov_cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, '/aruco_docking/state', 10)
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
        self.get_logger().info('publishing distance-gated docking commands on /rov_cmd_vel')
        self.get_logger().info(
            'target_distance=%.3f align_distance=%.3f minimum_safe_z=%.3f '
            'target_z=%.3f z_tolerance=%.3f target_x=%.3f x_tolerance=%.3f '
            'target_yaw=%.3f yaw_tolerance=%.3f far_approach_speed=%.3f '
            'final_approach_speed=%.3f max_lateral_align_speed=%.3f '
            'max_yaw_align_speed=%.3f detection_timeout=%.3f hold_duration=%.3f '
            'control_rate=%.1f'
            % (
                self.target_distance,
                self.align_distance,
                self.minimum_safe_z,
                self.target_z,
                self.z_tolerance,
                self.target_x,
                self.x_tolerance,
                self.target_yaw,
                self.yaw_tolerance,
                self.far_approach_speed,
                self.final_approach_speed,
                self.max_lateral_align_speed,
                self.max_yaw_align_speed,
                self.detection_timeout,
                self.hold_duration,
                self.control_rate,
            )
        )

    def _validate_parameters(self):
        if self.target_distance <= 0.0:
            raise ValueError('target_distance must be greater than 0')
        if self.align_distance <= 0.0:
            raise ValueError('align_distance must be greater than 0')
        if self.minimum_safe_z <= 0.0:
            raise ValueError('minimum_safe_z must be greater than 0')
        if self.align_hysteresis < 0.0:
            raise ValueError('align_hysteresis must be greater than or equal to 0')
        if self.z_tolerance < 0.0:
            raise ValueError('z_tolerance must be greater than or equal to 0')
        if self.x_tolerance < 0.0:
            raise ValueError('x_tolerance must be greater than or equal to 0')
        if self.yaw_tolerance < 0.0:
            raise ValueError('yaw_tolerance must be greater than or equal to 0')
        if self.final_x_realign_threshold < 0.0:
            raise ValueError('final_x_realign_threshold must be greater than or equal to 0')
        if self.final_yaw_realign_threshold < 0.0:
            raise ValueError('final_yaw_realign_threshold must be greater than or equal to 0')
        if self.theta_x_slow_limit < 0.0 or self.theta_x_stop_limit < 0.0:
            raise ValueError('theta_x limits must be greater than or equal to 0')
        if self.theta_y_slow_limit < 0.0 or self.theta_y_stop_limit < 0.0:
            raise ValueError('theta_y limits must be greater than or equal to 0')
        if self.theta_x_slow_limit > self.theta_x_stop_limit:
            raise ValueError('theta_x_slow_limit must be less than or equal to theta_x_stop_limit')
        if self.theta_y_slow_limit > self.theta_y_stop_limit:
            raise ValueError('theta_y_slow_limit must be less than or equal to theta_y_stop_limit')
        if self.kp_lateral < 0.0:
            raise ValueError('kp_lateral must be greater than or equal to 0')
        if self.kp_yaw < 0.0:
            raise ValueError('kp_yaw must be greater than or equal to 0')
        if self.kp_far_center < 0.0:
            raise ValueError('kp_far_center must be greater than or equal to 0')
        for name, value in (
            ('far_approach_speed', self.far_approach_speed),
            ('reduced_far_approach_speed', self.reduced_far_approach_speed),
            ('final_approach_speed', self.final_approach_speed),
            ('min_far_center_speed', self.min_far_center_speed),
            ('max_far_center_speed', self.max_far_center_speed),
            ('min_lateral_align_speed', self.min_lateral_align_speed),
            ('max_lateral_align_speed', self.max_lateral_align_speed),
            ('min_yaw_align_speed', self.min_yaw_align_speed),
            ('max_yaw_align_speed', self.max_yaw_align_speed),
        ):
            if value < 0.0:
                raise ValueError('%s must be greater than or equal to 0' % name)
        if self.min_far_center_speed > self.max_far_center_speed:
            raise ValueError('min_far_center_speed must be less than or equal to max_far_center_speed')
        if self.min_lateral_align_speed > self.max_lateral_align_speed:
            raise ValueError('min_lateral_align_speed must be less than or equal to max_lateral_align_speed')
        if self.min_yaw_align_speed > self.max_yaw_align_speed:
            raise ValueError('min_yaw_align_speed must be less than or equal to max_yaw_align_speed')
        if self.detection_timeout <= 0.0:
            raise ValueError('detection_timeout must be greater than 0')
        if self.hold_duration <= 0.0:
            raise ValueError('hold_duration must be greater than 0')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be greater than 0')

    def distance_callback(self, msg: ArucoDistance):
        self.latest_x = float(msg.x)
        self.latest_y = float(msg.y)
        self.latest_z = float(msg.z)
        self.latest_distance = float(msg.distance)
        self.latest_theta = float(msg.theta)
        self.latest_yaw = float(msg.yaw)
        self.last_detection_time = self.get_clock().now()

    def control_callback(self):
        cmd = Twist()

        if not self._has_recent_detection():
            self._transition_to(STATE_WAIT_FOR_MARKER, 'marker timeout')
            if not self.was_timed_out:
                self.get_logger().warn('ArUco detection timed out; stopping robot')
            self.was_timed_out = True
            self._publish_state()
            self.cmd_publisher.publish(cmd)
            return

        self.was_timed_out = False

        if self.state == STATE_WAIT_FOR_MARKER:
            self._route_from_marker()

        if self.state == STATE_FAR_GUIDED_APPROACH:
            cmd = self._run_far_guided_approach()
        elif self.state == STATE_NEAR_ALIGN:
            cmd = self._run_near_align()
        elif self.state == STATE_FINAL_APPROACH:
            cmd = self._run_final_approach()
        elif self.state == STATE_HOLD:
            cmd = self._run_hold()
        elif self.state == STATE_DOCKED:
            cmd = Twist()
        else:
            self._transition_to(STATE_WAIT_FOR_MARKER, 'unknown state')

        self._publish_state()
        self.cmd_publisher.publish(cmd)

    def _has_recent_detection(self) -> bool:
        if (
            self.latest_x is None
            or self.latest_y is None
            or self.latest_z is None
            or self.latest_distance is None
            or self.latest_theta is None
            or self.latest_yaw is None
            or self.last_detection_time is None
        ):
            return False

        elapsed = self.get_clock().now() - self.last_detection_time
        return elapsed.nanoseconds * 1e-9 <= self.detection_timeout

    def _route_from_marker(self):
        if self._is_docked():
            self._transition_to(STATE_DOCKED, 'already docked')
        elif self._inside_safe_z():
            self._transition_to(STATE_HOLD, 'inside safe z')
        elif self.latest_distance <= self.align_distance:
            self._transition_to(STATE_NEAR_ALIGN, 'inside align distance')
        else:
            self._transition_to(STATE_FAR_GUIDED_APPROACH, 'outside align distance')

    def _run_far_guided_approach(self) -> Twist:
        cmd = Twist()
        if self._is_docked():
            self._transition_to(STATE_DOCKED, 'docked')
            return cmd
        if self._inside_safe_z():
            self._transition_to(STATE_HOLD, 'safe z reached before docked')
            return cmd
        if self._visibility_stop_guard():
            self._transition_to(STATE_HOLD, 'visibility stop guard')
            return cmd
        if self.latest_distance <= self.align_distance:
            self._transition_to(STATE_NEAR_ALIGN, 'align distance reached')
            return cmd

        cmd.linear.x = self.reduced_far_approach_speed if self._visibility_slow_guard() else self.far_approach_speed
        cmd.angular.z = self._clamp_with_min(
            self.kp_far_center * self._theta_x(),
            self.min_far_center_speed,
            self.max_far_center_speed,
        )
        return cmd

    def _run_near_align(self) -> Twist:
        cmd = Twist()
        if self._is_docked():
            self._transition_to(STATE_DOCKED, 'docked')
            return cmd
        if self._inside_safe_z():
            self._transition_to(STATE_HOLD, 'safe z reached before docked')
            return cmd
        if self._visibility_stop_guard():
            self._transition_to(STATE_HOLD, 'visibility stop guard')
            return cmd
        if self.latest_distance > self.align_distance + self.align_hysteresis:
            self._transition_to(STATE_FAR_GUIDED_APPROACH, 'outside align hysteresis')
            return cmd
        if self._near_align_complete():
            self._transition_to(STATE_FINAL_APPROACH, 'near alignment complete')
            return cmd

        cmd.linear.y = self._clamp_with_min(
            -self.kp_lateral * self._error_x(),
            self.min_lateral_align_speed,
            self.max_lateral_align_speed,
        )
        cmd.angular.z = self._clamp_with_min(
            self.kp_yaw * self._error_yaw(),
            self.min_yaw_align_speed,
            self.max_yaw_align_speed,
        )
        return cmd

    def _run_final_approach(self) -> Twist:
        cmd = Twist()
        if self._is_docked():
            self._transition_to(STATE_DOCKED, 'docked')
            return cmd
        if self._inside_safe_z():
            self._transition_to(STATE_HOLD, 'safe z reached before docked')
            return cmd
        if self._visibility_stop_guard():
            self._transition_to(STATE_HOLD, 'visibility stop guard')
            return cmd
        if self._final_realign_needed():
            self._transition_to(STATE_NEAR_ALIGN, 'final approach drift')
            return cmd

        cmd.linear.x = self.final_approach_speed
        return cmd

    def _run_hold(self) -> Twist:
        cmd = Twist()
        if self.hold_start_time is None:
            self.hold_start_time = self.get_clock().now()

        elapsed = self.get_clock().now() - self.hold_start_time
        if elapsed.nanoseconds * 1e-9 < self.hold_duration:
            return cmd

        if self._is_docked():
            self._transition_to(STATE_DOCKED, 'docked after hold')
        elif self._inside_safe_z():
            self.hold_start_time = self.get_clock().now()
        elif self._visibility_stop_guard():
            self.hold_start_time = self.get_clock().now()
        elif self.latest_distance <= self.align_distance:
            self._transition_to(STATE_NEAR_ALIGN, 'hold released near')
        else:
            self._transition_to(STATE_FAR_GUIDED_APPROACH, 'hold released far')

        return cmd

    def _transition_to(self, new_state: str, reason: str):
        if self.state == new_state:
            return
        self.get_logger().info('%s -> %s (%s)' % (self.state, new_state, reason))
        self.state = new_state
        if new_state == STATE_HOLD:
            self.hold_start_time = self.get_clock().now()
        else:
            self.hold_start_time = None

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_publisher.publish(msg)

    def _clamp_with_min(self, value: float, min_abs: float, max_abs: float) -> float:
        if value == 0.0:
            return 0.0

        value = _clamp(value, -max_abs, max_abs)
        if 0.0 < abs(value) < min_abs:
            return min_abs if value > 0.0 else -min_abs

        return value

    def _is_docked(self) -> bool:
        return (
            abs(self._error_z()) < self.z_tolerance
            and abs(self._error_x()) < self.x_tolerance
            and abs(self._error_yaw()) < self.yaw_tolerance
        )

    def _near_align_complete(self) -> bool:
        return abs(self._error_x()) < self.x_tolerance and abs(self._error_yaw()) < self.yaw_tolerance

    def _final_realign_needed(self) -> bool:
        return (
            abs(self._error_x()) > self.final_x_realign_threshold
            or abs(self._error_yaw()) > self.final_yaw_realign_threshold
        )

    def _inside_safe_z(self) -> bool:
        return self.latest_z <= self.minimum_safe_z

    def _visibility_slow_guard(self) -> bool:
        return abs(self._theta_x()) > self.theta_x_slow_limit or abs(self._theta_y()) > self.theta_y_slow_limit

    def _visibility_stop_guard(self) -> bool:
        return abs(self._theta_x()) > self.theta_x_stop_limit or abs(self._theta_y()) > self.theta_y_stop_limit

    def _theta_x(self) -> float:
        if self.latest_theta is not None:
            return self.latest_theta
        return math.atan2(self.latest_x, self.latest_z)

    def _theta_y(self) -> float:
        return math.atan2(self.latest_y, self.latest_z)

    def _error_z(self) -> float:
        return self.latest_z - self.target_z

    def _error_x(self) -> float:
        return self.latest_x - self.target_x

    def _error_yaw(self) -> float:
        return _wrap_pi(self.latest_yaw - self.target_yaw)


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDistanceController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
