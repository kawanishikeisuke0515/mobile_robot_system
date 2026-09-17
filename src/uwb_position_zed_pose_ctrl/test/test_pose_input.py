import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from uwb_position_zed_pose_ctrl.uwb_position_zed_pose_ctrl import UwbPositionZedPoseController


@pytest.fixture
def controller():
    node = object.__new__(UwbPositionZedPoseController)
    node.world_frame_id = 'world'
    node.pose_timeout = .5
    node.latest_pose = None
    node.last_pose_stamp = None
    node.last_clock_ns = None
    node.current_yaw = 0.
    node.now_ns = 2_000_000_000
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=node.now_ns))
    return node


def pose(stamp=2_000_000_000, theta=0.):
    msg = PoseStamped()
    msg.header.frame_id = 'world'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    msg.pose.orientation.z = math.sin((theta + math.pi / 2) / 2)
    msg.pose.orientation.w = math.cos((theta + math.pi / 2) / 2)
    return msg


@pytest.mark.parametrize('theta', [0., .5, -1., math.pi - .01, -math.pi + .01])
def test_heading_convention(controller, theta):
    controller.pose_callback(pose(theta=theta))
    assert controller._has_valid_inputs()
    assert controller.current_yaw == pytest.approx(theta)


@pytest.mark.parametrize('case', ['stale', 'future', 'frame', 'nan', 'quaternion'])
def test_invalid_pose_stops(controller, case):
    controller.pose_callback(pose())
    msg = pose()
    if case == 'stale':
        msg = pose(1_000_000_000)
    elif case == 'future':
        msg = pose(3_000_000_000)
    elif case == 'frame':
        msg.header.frame_id = 'camera'
    elif case == 'nan':
        msg.pose.position.x = math.nan
    else:
        msg.pose.orientation.z = msg.pose.orientation.w = 0.
    controller.pose_callback(msg)
    assert not controller._has_valid_inputs()


def test_replay_timeout_reset_and_clock_jump(controller):
    controller.pose_callback(pose())
    controller.now_ns += 400_000_000
    controller.pose_callback(pose())  # Replaying cannot refresh observation time.
    controller.now_ns += 200_000_000
    assert not controller._has_valid_inputs()
    controller.pose_callback(pose(controller.now_ns))
    assert controller._has_valid_inputs()
    controller._reset_control()
    controller.pose_callback(pose(controller.now_ns))
    assert not controller._has_valid_inputs()
    controller.now_ns = 1_000_000_000
    controller.pose_callback(pose(controller.now_ns))
    assert controller._has_valid_inputs()
