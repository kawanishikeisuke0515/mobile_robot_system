import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mode_manager.controller_link import ControllerLink
from rclpy.node import Node
from uwb_interfaces.msg import UwbControlError

from .pose_control import PoseControlConfig
from .pose_control import calculate_pose_command
from .pose_control import is_finite
from .pose_control import wrap_pi


class UwbPositionZedPoseController(Node):
    def __init__(self):
        super().__init__('uwb_position_zed_pose_ctrl')

        self.declare_parameter('robot_pose_topic', '/uwb/robot_pose')
        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('cmd_vel_topic', '/rov_cmd_vel')
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('x_tolerance', 0.05)
        self.declare_parameter('y_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('kp_x', 0.4)
        self.declare_parameter('kp_y', 0.4)
        self.declare_parameter('kp_yaw', 0.8)
        self.declare_parameter('min_linear_speed', 0.25)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('min_angular_speed', 0.0)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('pose_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)

        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        self.world_frame_id = str(self.get_parameter('world_frame_id').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.config = PoseControlConfig(
            target_x=float(self.get_parameter('target_x').value),
            target_y=float(self.get_parameter('target_y').value),
            target_yaw=float(self.get_parameter('target_yaw').value),
            x_tolerance=float(self.get_parameter('x_tolerance').value),
            y_tolerance=float(self.get_parameter('y_tolerance').value),
            yaw_tolerance=float(self.get_parameter('yaw_tolerance').value),
            kp_x=float(self.get_parameter('kp_x').value),
            kp_y=float(self.get_parameter('kp_y').value),
            kp_yaw=float(self.get_parameter('kp_yaw').value),
            min_linear_speed=float(self.get_parameter('min_linear_speed').value),
            max_linear_speed=float(self.get_parameter('max_linear_speed').value),
            min_angular_speed=float(self.get_parameter('min_angular_speed').value),
            max_angular_speed=float(self.get_parameter('max_angular_speed').value),
        )
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self._validate_parameters()

        self.latest_pose: Optional[PoseStamped] = None
        self.current_yaw = 0.0
        self.last_pose_stamp = None
        self.last_clock_ns = None
        self.was_waiting_for_inputs = True
        self.was_target_reached = False

        self.link = ControllerLink(self, 'uwb', self._reset_control)
        self.declare_parameter('control_error_topic', '/uwb/control_error')
        self.error_publisher = self.create_publisher(
            UwbControlError, str(self.get_parameter('control_error_topic').value), 10)
        self.cmd_publisher = None
        if not self.link.managed:
            self.cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(PoseStamped, self.robot_pose_topic, self.pose_callback, 10)
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_callback,
        )

        self.get_logger().info(
            'subscribing %s; publishing %s'
            % (self.robot_pose_topic,
               '/uwb/control_output' if self.link.managed else self.cmd_vel_topic)
        )
        self.get_logger().info(
            'target=(%.3f, %.3f, %.3f) tolerance=(%.3f, %.3f, %.3f) '
            'kp=(%.3f, %.3f, %.3f) min_speed=(%.3f, %.3f) '
            'max_speed=(%.3f, %.3f) '
            'pose_timeout=%.3f control_rate=%.1f'
            % (
                self.config.target_x,
                self.config.target_y,
                self.config.target_yaw,
                self.config.x_tolerance,
                self.config.y_tolerance,
                self.config.yaw_tolerance,
                self.config.kp_x,
                self.config.kp_y,
                self.config.kp_yaw,
                self.config.min_linear_speed,
                self.config.min_angular_speed,
                self.config.max_linear_speed,
                self.config.max_angular_speed,
                self.pose_timeout,
                self.control_rate,
            )
        )

    def _validate_parameters(self):
        if not self.robot_pose_topic.strip() or not self.world_frame_id.strip():
            raise ValueError('robot_pose_topic and world_frame_id must not be empty')
        if self.cmd_vel_topic == '':
            raise ValueError('cmd_vel_topic must not be empty')
        if not is_finite(
            self.config.target_x,
            self.config.target_y,
            self.config.target_yaw,
            self.config.x_tolerance,
            self.config.y_tolerance,
            self.config.yaw_tolerance,
            self.config.kp_x,
            self.config.kp_y,
            self.config.kp_yaw,
            self.config.min_linear_speed,
            self.config.max_linear_speed,
            self.config.min_angular_speed,
            self.config.max_angular_speed,
            self.pose_timeout,
            self.control_rate,
        ):
            raise ValueError('numeric parameters must be finite')
        if self.config.x_tolerance < 0.0:
            raise ValueError('x_tolerance must be greater than or equal to 0')
        if self.config.y_tolerance < 0.0:
            raise ValueError('y_tolerance must be greater than or equal to 0')
        if self.config.yaw_tolerance < 0.0:
            raise ValueError('yaw_tolerance must be greater than or equal to 0')
        if self.config.kp_x < 0.0:
            raise ValueError('kp_x must be greater than or equal to 0')
        if self.config.kp_y < 0.0:
            raise ValueError('kp_y must be greater than or equal to 0')
        if self.config.kp_yaw < 0.0:
            raise ValueError('kp_yaw must be greater than or equal to 0')
        if self.config.min_linear_speed < 0.0:
            raise ValueError('min_linear_speed must be greater than or equal to 0')
        if self.config.max_linear_speed < 0.0:
            raise ValueError('max_linear_speed must be greater than or equal to 0')
        if self.config.min_linear_speed > self.config.max_linear_speed:
            raise ValueError('min_linear_speed must be less than or equal to max_linear_speed')
        if self.config.min_angular_speed < 0.0:
            raise ValueError('min_angular_speed must be greater than or equal to 0')
        if self.config.max_angular_speed < 0.0:
            raise ValueError('max_angular_speed must be greater than or equal to 0')
        if self.config.min_angular_speed > self.config.max_angular_speed:
            raise ValueError('min_angular_speed must be less than or equal to max_angular_speed')
        if self.pose_timeout <= 0.0:
            raise ValueError('pose_timeout must be greater than 0')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be greater than 0')

    def _now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now < self.last_clock_ns:
            self._reset_control()
            self.last_pose_stamp = None
        self.last_clock_ns = now
        return now

    def pose_callback(self, msg: PoseStamped):
        now = self._now_ns()
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        p, q = msg.pose.position, msg.pose.orientation
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if (msg.header.frame_id != self.world_frame_id
                or not is_finite(p.x, p.y, p.z, q.x, q.y, q.z, q.w, norm)
                or norm < 1e-12 or abs(norm - 1.0) > 1e-3
                or not 0 <= (now - stamp) * 1e-9 <= self.pose_timeout):
            self.latest_pose = None
            return
        if self.last_pose_stamp is not None and stamp <= self.last_pose_stamp:
            return
        x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
        world_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        # Preserve existing controller/target convention: zero points along world +Y.
        self.current_yaw = wrap_pi(world_yaw - math.pi / 2)
        self.latest_pose = msg
        self.last_pose_stamp = stamp

    def _reset_control(self):
        self.was_target_reached = False
        self.latest_pose = None

    def control_callback(self):
        active = self.link.active
        cmd = Twist()
        if not self._has_valid_inputs():
            if not self.was_waiting_for_inputs:
                self.get_logger().warn('Robot center pose unavailable; stopping robot')
            self.was_waiting_for_inputs = True
            self.was_target_reached = False
            self._publish_error(None, active)
            self.link.publish(cmd, inputs_valid=False)
            return

        self.was_waiting_for_inputs = False
        result = calculate_pose_command(
            current_x=float(self.latest_pose.pose.position.x),
            current_y=float(self.latest_pose.pose.position.y),
            current_yaw=self.current_yaw,
            config=self.config,
        )
        self._publish_error(result.debug, active)
        cmd.linear.x = result.linear_x
        cmd.linear.y = result.linear_y
        cmd.angular.z = result.angular_z

        if result.debug.target_reached and not self.was_target_reached:
            self.get_logger().info('target pose reached; publishing zero velocity')
        self.was_target_reached = result.debug.target_reached
        if not active:
            cmd = Twist()
        self.link.publish(cmd, inputs_valid=True,
                          target_reached=result.debug.target_reached)

    def _publish_error(self, debug, active):
        msg = UwbControlError()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.session_id = self.link.session_id
        msg.active = active
        msg.inputs_valid = debug is not None
        for field in ('raw_error_world_x', 'raw_error_world_y', 'error_world_x',
                      'error_world_y', 'error_body_x', 'error_body_y', 'yaw_error'):
            setattr(msg, field, getattr(debug, field) if debug is not None else float('nan'))
        msg.distance_error_m = math.hypot(debug.raw_error_world_x, debug.raw_error_world_y) \
            if debug is not None else float('nan')
        self.error_publisher.publish(msg)

    def _has_valid_inputs(self) -> bool:
        now = self._now_ns()
        return (self.latest_pose is not None and self.last_pose_stamp is not None
                and 0 <= (now - self.last_pose_stamp) * 1e-9 <= self.pose_timeout)


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = UwbPositionZedPoseController()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
