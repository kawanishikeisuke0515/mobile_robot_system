"""Published telemetry matches the P input, including tolerance and invalidation."""
import math
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('uwb_interfaces.msg')
from geometry_msgs.msg import PoseStamped
from uwb_position_zed_pose_ctrl.uwb_position_zed_pose_ctrl import UwbPositionZedPoseController


def test_control_error_matches_calculation(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init(args=['--ros-args', '-p', 'target_x:=3.0', '-p', 'target_y:=4.0'])
    node = None
    try:
        node = UwbPositionZedPoseController()
        messages = []
        commands = []
        node.cmd_publisher = SimpleNamespace(publish=commands.append)
        node.error_publisher = SimpleNamespace(publish=messages.append)
        position = PoseStamped()
        position.header.frame_id = 'world'
        position.header.stamp = node.get_clock().now().to_msg()
        position.pose.orientation.z = math.sqrt(.5)
        position.pose.orientation.w = math.sqrt(.5)
        node.pose_callback(position)
        node.control_callback()
        error = messages[-1]
        assert error.inputs_valid and error.active
        assert error.distance_error_m == 5.
        assert error.error_body_x == 4.
        assert error.error_body_y == 3.
        # Inside lateral tolerance: keep diagnostic error, zero the velocity.
        position.pose.position.x = 2.99
        position.header.stamp = node.get_clock().now().to_msg()
        node.pose_callback(position)
        node.control_callback()
        assert messages[-1].raw_error_world_x == pytest.approx(.01, abs=1e-6)
        assert messages[-1].error_body_y == pytest.approx(.01)
        assert commands[-1].linear.y == 0.
        # Missing/stale inputs must invalidate the previous numerical errors.
        node.latest_pose = None
        node.control_callback()
        assert not messages[-1].inputs_valid
        assert math.isnan(messages[-1].distance_error_m)
        assert math.isnan(messages[-1].error_body_x)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
