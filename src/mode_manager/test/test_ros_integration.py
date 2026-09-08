"""Run after building/sourcing the workspace; no camera or motor nodes required."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mode_manager_interfaces.msg')
pytest.importorskip('uwb_position_zed_pose_ctrl.uwb_position_zed_pose_ctrl')
pytest.importorskip('vision_dist_ctrl.vision_distance_controller')
from aruco_interfaces.msg import ArucoDistance
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from uwb_interfaces.msg import UwbPosition
from zed_interfaces.msg import ZedHeading
from mode_manager.node import ModeManager
from uwb_position_zed_pose_ctrl.uwb_position_zed_pose_ctrl import UwbPositionZedPoseController
from vision_dist_ctrl.vision_distance_controller import VisionDistanceController


def test_real_controllers_handoff_recovery_completion_and_stop():
    rclpy.init(args=['--ros-args', '-p', 'managed_mode:=true', '-p',
                    'target_marker_id:=7', '-p', 'stable_detection_time:=0.15',
                    '-p', 'detection_timeout:=0.2'])
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        manager = ModeManager()
        uwb = UwbPositionZedPoseController()
        vision = VisionDistanceController()
        probe = Node('mode_manager_test_probe')
        nodes.extend([manager, uwb, vision, probe])
        for node in nodes:
            executor.add_node(node)
        position_pub = probe.create_publisher(UwbPosition, '/uwb/position', 10)
        heading_pub = probe.create_publisher(ZedHeading, '/zed/heading', 10)
        marker_pub = probe.create_publisher(ArucoDistance, '/aruco/distance', 10)
        observed = []
        probe.create_subscription(Twist, '/rov_cmd_vel', lambda msg: observed.append(msg), 10)
        position = UwbPosition(x_m=0.0, y_m=-1.0, valid=True)
        heading = ZedHeading(robot_yaw_rad=0.0, valid=True)
        marker = ArucoDistance(id=7, z=1.3, yaw=0.0, normalized_center_error=0.0)
        use_marker = True

        def pump_until(predicate, timeout=3.):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                position_pub.publish(position)
                heading_pub.publish(heading)
                if use_marker:
                    marker_pub.publish(marker)
                executor.spin_once(timeout_sec=0.01)
                if predicate():
                    return
            raise AssertionError(f'timeout in {manager.machine.state}: {manager.machine.reason}')

        pump_until(lambda: bool(observed))
        assert manager.machine.state == 'IDLE'
        assert all(msg.linear.x == 0. for msg in observed)
        # Exactly the manager publishes to the final output.
        pump_until(lambda: probe.count_publishers('/rov_cmd_vel') == 1)
        assert manager.start(Trigger.Request(), Trigger.Response()).success
        pump_until(lambda: any(msg.linear.x > 0 for msg in observed))
        position.y_m = 0.0
        pump_until(lambda: manager.machine.state == 'VISION_DOCKING')
        pump_until(lambda: vision.state == 'FINAL_DOCKING')
        use_marker = False
        position.valid = False
        pump_until(lambda: manager.machine.state == 'UWB_RECOVERY')
        pump_until(lambda: observed[-1].linear.x == 0.0)
        assert observed[-1].linear.x == 0.0
        position.valid = False
        pump_until(lambda: not uwb._has_valid_inputs())
        assert manager.machine.state == 'UWB_RECOVERY'
        position.valid = True
        position.y_m = -0.5
        marker.id = 99
        use_marker = True
        pump_until(lambda: uwb._has_valid_inputs())
        assert manager.machine.state == 'UWB_RECOVERY'
        position.y_m = 0.0
        pump_until(lambda: manager.machine.state == 'VISION_WAIT')
        # Wrong-ID samples cannot restore tracking.
        start = time.monotonic()
        pump_until(lambda: time.monotonic() - start > 0.3)
        assert manager.machine.state == 'VISION_WAIT'
        marker.id = 7
        pump_until(lambda: manager.machine.state == 'VISION_DOCKING')
        pump_until(lambda: vision.state == 'FINAL_DOCKING')
        marker.z = 0.9
        pump_until(lambda: manager.machine.state == 'DONE')
        pump_until(lambda: observed[-1].linear.x == 0.0)
        assert observed[-1].linear.x == 0.0
        use_marker = False
        start = time.monotonic()
        pump_until(lambda: time.monotonic() - start > 0.3)
        assert manager.machine.state == 'DONE'
        assert manager.stop(Trigger.Request(), Trigger.Response()).success
        assert manager.machine.state == 'IDLE'
        # A manager heartbeat outage invalidates the controller lease.
        position.y_m = -0.5
        assert manager.start(Trigger.Request(), Trigger.Response()).success
        pump_until(lambda: uwb.link.active)
        for timer in manager.timers:
            timer.cancel()
        pump_until(lambda: not uwb.link.active)
        assert not vision.link.active
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
