import os
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from uwb_interfaces.msg import UwbPosition
from zed_interfaces.msg import ZedHeading

from .pose_control import PoseControlConfig
from .pose_control import calculate_pose_command
from .pose_control import is_finite


class UwbPositionZedPoseController(Node):
    def __init__(self):
        super().__init__('uwb_position_zed_pose_ctrl')

        self.declare_parameter('uwb_position_topic', '/uwb/position')
        self.declare_parameter('zed_heading_topic', '/zed/heading')
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
        self.declare_parameter('min_linear_speed', 0.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('min_angular_speed', 0.0)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('position_timeout', 0.5)
        self.declare_parameter('heading_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)

        self.uwb_position_topic = str(self.get_parameter('uwb_position_topic').value)
        self.zed_heading_topic = str(self.get_parameter('zed_heading_topic').value)
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
        self.position_timeout = float(self.get_parameter('position_timeout').value)
        self.heading_timeout = float(self.get_parameter('heading_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self._validate_parameters()

        self.latest_position: Optional[UwbPosition] = None
        self.latest_heading: Optional[ZedHeading] = None
        self.last_position_time: Optional[Time] = None
        self.last_heading_time: Optional[Time] = None
        self.was_waiting_for_inputs = True
        self.was_target_reached = False

        self.cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(
            UwbPosition,
            self.uwb_position_topic,
            self.position_callback,
            10,
        )
        self.create_subscription(
            ZedHeading,
            self.zed_heading_topic,
            self.heading_callback,
            10,
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_callback,
        )

        self.get_logger().info(
            'subscribing %s and %s; publishing %s'
            % (self.uwb_position_topic, self.zed_heading_topic, self.cmd_vel_topic)
        )
        self.get_logger().info(
            'target=(%.3f, %.3f, %.3f) tolerance=(%.3f, %.3f, %.3f) '
            'kp=(%.3f, %.3f, %.3f) min_speed=(%.3f, %.3f) '
            'max_speed=(%.3f, %.3f) timeout=(%.3f, %.3f) control_rate=%.1f'
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
                self.position_timeout,
                self.heading_timeout,
                self.control_rate,
            )
        )

    def _validate_parameters(self):
        if self.uwb_position_topic == '':
            raise ValueError('uwb_position_topic must not be empty')
        if self.zed_heading_topic == '':
            raise ValueError('zed_heading_topic must not be empty')
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
            self.position_timeout,
            self.heading_timeout,
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
        if self.position_timeout <= 0.0:
            raise ValueError('position_timeout must be greater than 0')
        if self.heading_timeout <= 0.0:
            raise ValueError('heading_timeout must be greater than 0')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be greater than 0')

    def position_callback(self, msg: UwbPosition):
        self.latest_position = msg
        self.last_position_time = self.get_clock().now()

    def heading_callback(self, msg: ZedHeading):
        self.latest_heading = msg
        self.last_heading_time = self.get_clock().now()

    def control_callback(self):
        cmd = Twist()
        if not self._has_valid_inputs():
            if not self.was_waiting_for_inputs:
                self.get_logger().warn('UWB position or ZED heading unavailable; stopping robot')
            self.was_waiting_for_inputs = True
            self.was_target_reached = False
            self.cmd_publisher.publish(cmd)
            return

        self.was_waiting_for_inputs = False
        result = calculate_pose_command(
            current_x=float(self.latest_position.x_m),
            current_y=float(self.latest_position.y_m),
            current_yaw=float(self.latest_heading.robot_yaw_rad),
            config=self.config,
        )
        cmd.linear.x = result.linear_x
        cmd.linear.y = result.linear_y
        cmd.angular.z = result.angular_z

        if result.debug.target_reached and not self.was_target_reached:
            self.get_logger().info('target pose reached; publishing zero velocity')
        self.was_target_reached = result.debug.target_reached
        self.cmd_publisher.publish(cmd)

    def _has_valid_inputs(self) -> bool:
        if (
            self.latest_position is None
            or self.latest_heading is None
            or self.last_position_time is None
            or self.last_heading_time is None
        ):
            return False
        if not self.latest_position.valid or not self.latest_heading.valid:
            return False
        if not is_finite(
            float(self.latest_position.x_m),
            float(self.latest_position.y_m),
            float(self.latest_heading.robot_yaw_rad),
        ):
            return False

        now = self.get_clock().now()
        position_elapsed = (now - self.last_position_time).nanoseconds * 1e-9
        heading_elapsed = (now - self.last_heading_time).nanoseconds * 1e-9
        return (
            position_elapsed <= self.position_timeout
            and heading_elapsed <= self.heading_timeout
        )


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
