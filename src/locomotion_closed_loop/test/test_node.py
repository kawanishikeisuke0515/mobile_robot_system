"""Exercise ROS callbacks without connecting to a motor driver."""
import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from locomotion_closed_loop.node import ClosedLoopVelocity


def test_ros_callbacks():
    rclpy.init(args=['--ros-args', '-p', 'motor_command_limit:=100.0', '-p', 'use_camera_frame:=false'])
    node = ClosedLoopVelocity()
    try:
        command = Twist()
        command.linear.x = .5
        node.command_callback(command)
        odom = Odometry()
        odom.child_frame_id = 'base_link'
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.twist.twist.linear.x = .1
        node.odom_callback(odom)
        node.tick()
        assert node.last_state == 'running'
        assert node.core.measured == pytest.approx((.1, 0, 0))
        node.odom_callback(odom)
        assert len(node.core.samples) == 1
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.twist.twist.linear.x = float('nan')
        node.odom_callback(odom)
        node.tick()
        assert node.last_state == 'invalid_odom_or_transform'
        assert not node.core.samples
        changed = node.set_parameters([Parameter('velocity_filter_window_size', value=3)])
        assert not changed[0].successful
        odom.twist.twist.linear.x = .2
        odom.child_frame_id = 'nonexistent_frame'
        node.odom_callback(odom)
        assert node.core.measured is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_camera_frame_without_tf():
    rclpy.init(args=['--ros-args', '-p', 'motor_command_limit:=100.0'])
    node = ClosedLoopVelocity()
    try:
        assert node.tf is None
        command = Twist()
        command.linear.x = .5
        node.command_callback(command)
        odom = Odometry()
        odom.child_frame_id = 'zed_camera_without_tf'
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.twist.twist.linear.x = .1
        odom.twist.twist.linear.y = .2
        odom.twist.twist.angular.z = .3
        node.odom_callback(odom)
        node.tick()
        assert node.last_state == 'running'
        assert node.core.measured == pytest.approx((.1, .2, .3))
    finally:
        node.destroy_node()
        rclpy.shutdown()
