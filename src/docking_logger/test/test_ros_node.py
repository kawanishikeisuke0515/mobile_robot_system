"""ROS message/callback/timer integration (does not require network delivery)."""
import csv
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('uwb_interfaces.msg')
from docking_logger.docking_logger import DockingLogger, STREAMS


def test_all_message_types_and_timer(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init(args=['--ros-args', '-p', 'target_marker_id:=7',
                     '-p', 'log_dir:=' + str(tmp_path)])
    node = None
    try:
        node = DockingLogger()
        for key, (message_type, _) in STREAMS.items():
            message = message_type()
            if key == 'vision':
                message.id = 7
            if key == 'motor_commands':
                message.data = [1., 2., 3., 4.]
            node.receive(key, message)
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node:
            node.writer.close()
            node.destroy_node()
        rclpy.shutdown()
    with (node.writer.path / 'timeline.csv').open() as file:
        rows = list(csv.DictReader(file))
    assert len(rows) >= 3
    assert all(rows[-1][key + '_received'] == 'true' for key in STREAMS)
    assert rows[-1]['motor_commands_motor_3'] == '4.0'
