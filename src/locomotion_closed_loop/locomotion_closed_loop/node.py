"""Three-axis PI velocity controller with ZED feedback."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformException

from .control import VelocityControl, body_twist


class ClosedLoopVelocity(Node):
    def __init__(self):
        super().__init__('closed_loop_velocity')
        defaults = dict(command_topic='/rov_cmd_vel', odom_topic='/zed2i/zed_node/odom',
                        motor_topic='/rov/motors', body_frame='base_link', use_camera_frame=True,
                        control_frequency=30., command_timeout=0.5, odom_timeout=0.5,
                        max_odom_age=0.5, future_tolerance=0.05,
                        kp_x=0.1, kp_y=0.1, kp_yaw=0.1,
                        ki_x=0.2, ki_y=0.2, ki_yaw=0.2,
                        gain_x=1000., gain_y=1000., gain_yaw=30.,
                        velocity_filter_enabled=True, velocity_filter_window_size=5,
                        motor_command_limit=0., wheel_lever=375., wheel_radius=63.5)
        p = {}
        for name, value in defaults.items():
            p[name] = self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
        self.p = p
        for name in ('control_frequency', 'max_odom_age', 'future_tolerance'):
            if not math.isfinite(p[name]) or p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not p['use_camera_frame'] and not p['body_frame']:
            raise ValueError('body_frame is required')
        self.core = VelocityControl(
            kp=tuple(p['kp_'+axis] for axis in ('x', 'y', 'yaw')),
            ki=tuple(p['ki_'+axis] for axis in ('x', 'y', 'yaw')),
            gains=tuple(p['gain_'+axis] for axis in ('x', 'y', 'yaw')),
            window=p['velocity_filter_window_size'], filter_enabled=p['velocity_filter_enabled'],
            motor_limit=p['motor_command_limit'], command_timeout=p['command_timeout'],
            odom_timeout=p['odom_timeout'], lever=p['wheel_lever'], radius=p['wheel_radius'])
        self.tf = None if p['use_camera_frame'] else Buffer()
        self.tf_listener = TransformListener(self.tf, self) if self.tf is not None else None
        self.motor_pub = self.create_publisher(Float32MultiArray, p['motor_topic'], 10)
        self.diag_pub = self.create_publisher(String, '~/diagnostics', 10)
        self.create_subscription(Twist, p['command_topic'], self.command_callback, 10)
        self.create_subscription(Odometry, p['odom_topic'], self.odom_callback, qos_profile_sensor_data)
        self.last_clock = None
        self.latest_stamp = None
        self.last_state = None
        self.create_timer(1./p['control_frequency'], self.tick)
        if self.core.limit == 0:
            self.get_logger().warning('Set motor_command_limit to the verified driver limit to enable output.')

    def command_callback(self, msg):
        try:
            self.core.set_command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())
        except ValueError:
            self.stop()

    def invalidate(self, reason):
        self.core.invalidate_odom(reason)
        self.latest_stamp = None
        self.stop()

    def odom_callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
        if age > self.p['max_odom_age'] or age < -self.p['future_tolerance']:
            self.invalidate('invalid_odom_age')
            return
        if not self.p['use_camera_frame'] and not msg.child_frame_id:
            self.invalidate('missing_child_frame')
            return
        twist = msg.twist.twist
        rotation, offset = (0., 0., 0., 1.), (0., 0., 0.)
        try:
            if not self.p['use_camera_frame'] and msg.child_frame_id != self.p['body_frame']:
                tf = self.tf.lookup_transform(self.p['body_frame'], msg.child_frame_id, stamp).transform
                rotation = (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
                offset = (tf.translation.x, tf.translation.y, tf.translation.z)
            values = body_twist((twist.linear.x, twist.linear.y, twist.linear.z),
                                (twist.angular.x, twist.angular.y, twist.angular.z), rotation, offset)
            accepted = self.core.add_sample(values, stamp.nanoseconds, time.monotonic())
            if accepted:
                self.latest_stamp = stamp.nanoseconds
            elif self.core.fault:
                self.latest_stamp = None
                self.stop()
        except (TransformException, ValueError) as exc:
            self.invalidate('invalid_odom_or_transform')
            self.get_logger().warning(str(exc), throttle_duration_sec=5.)

    def stop(self):
        self.motor_pub.publish(Float32MultiArray(data=[0., 0., 0., 0.]))

    def tick(self):
        clock = self.get_clock().now().nanoseconds
        if self.last_clock is not None and clock < self.last_clock:
            self.invalidate('clock_reversed')
            self.core.command_time = None
        self.last_clock = clock
        if self.latest_stamp is not None:
            age = (clock-self.latest_stamp)*1e-9
            if age > self.p['max_odom_age'] or age < -self.p['future_tolerance']:
                self.invalidate('invalid_odom_age')
        result = self.core.output(time.monotonic())
        self.motor_pub.publish(Float32MultiArray(data=list(result['output'])))
        result['use_camera_frame'] = self.p['use_camera_frame']
        result['filter_enabled'] = self.p['velocity_filter_enabled']
        result['window_size'] = self.p['velocity_filter_window_size']
        result['odom_age'] = (clock-self.latest_stamp)*1e-9 if self.latest_stamp is not None else None
        self.diag_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        if result['state'] != self.last_state:
            self.get_logger().info('Control state: '+result['state'])
            self.last_state = result['state']


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ClosedLoopVelocity()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
