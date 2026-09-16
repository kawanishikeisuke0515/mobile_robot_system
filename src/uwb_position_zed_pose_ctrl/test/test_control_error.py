"""Published telemetry matches the P input, including tolerance and invalidation."""
import math
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('uwb_interfaces.msg')
from uwb_interfaces.msg import UwbPosition
from zed_interfaces.msg import ZedHeading
from uwb_position_zed_pose_ctrl.uwb_position_zed_pose_ctrl import UwbPositionZedPoseController


def test_control_error_matches_calculation(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init(args=['--ros-args', '-p', 'target_x:=3.0', '-p', 'target_y:=4.0'])
    node = None
    try:
        node = UwbPositionZedPoseController()
        messages = []
        node.error_publisher = SimpleNamespace(publish=messages.append)
        position = UwbPosition()
        position.valid = True
        heading = ZedHeading()
        heading.valid = True
        node.position_callback(position)
        node.heading_callback(heading)
        node.control_callback()
        error = messages[-1]
        assert error.inputs_valid and error.active
        assert error.distance_error_m == 5.
        assert error.error_body_x == 4.
        assert error.error_body_y == 3.
        # Inside x tolerance: retain raw distance while zeroing the P input.
        position.x_m = 2.99
        node.position_callback(position)
        node.control_callback()
        assert messages[-1].raw_error_world_x == pytest.approx(.01, abs=1e-6)
        assert messages[-1].error_body_y == 0.
        # Missing/stale inputs must invalidate the previous numerical errors.
        node.last_position_time = None
        node.control_callback()
        assert not messages[-1].inputs_valid
        assert math.isnan(messages[-1].distance_error_m)
        assert math.isnan(messages[-1].error_body_x)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
