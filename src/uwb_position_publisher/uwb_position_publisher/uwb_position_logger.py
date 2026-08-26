"""CSV logger for published UWB position messages."""

import csv
import math
import os
import time
from datetime import datetime
from typing import Optional
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from uwb_interfaces.msg import UwbPosition


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class UwbPositionLogger(Node):
    """Log each published UWB position message to CSV."""

    def __init__(self):
        super().__init__('uwb_position_logger')

        self.declare_parameter('output_dir', '/tmp/uwb_position_logs')
        self.declare_parameter('uwb_position_topic', '/uwb/position')
        self.declare_parameter('flush_every_rows', 20)

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.uwb_position_topic = str(
            self.get_parameter('uwb_position_topic').value
        )
        self.flush_every_rows = int(
            self.get_parameter('flush_every_rows').value
        )
        self._validate_parameters()

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'uwb_position_log_{stamp}.csv',
        )

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'elapsed_sec',
            'position_stamp_sec',
            'device_time_ms',
            'x_m',
            'y_m',
            'valid',
        ])

        self.start_time = time.perf_counter()
        self.row_count = 0

        self.create_subscription(
            UwbPosition,
            self.uwb_position_topic,
            self.uwb_position_callback,
            10,
        )

        self.get_logger().info(
            'logging %s to %s' % (self.uwb_position_topic, self.csv_path)
        )

    def _validate_parameters(self):
        if self.output_dir == '':
            raise ValueError('output_dir must not be empty')
        if self.uwb_position_topic == '':
            raise ValueError('uwb_position_topic must not be empty')
        if self.flush_every_rows <= 0:
            raise ValueError('flush_every_rows must be greater than 0')

    def uwb_position_callback(self, msg: UwbPosition):
        elapsed_sec = time.perf_counter() - self.start_time
        self.writer.writerow([
            '%.4f' % elapsed_sec,
            '%.6f' % stamp_to_sec(msg.header.stamp),
            str(msg.device_time_ms),
            self._format_float(float(msg.x_m)),
            self._format_float(float(msg.y_m)),
            'true' if msg.valid else 'false',
        ])

        self.row_count += 1
        if self.row_count % self.flush_every_rows == 0:
            self.csv_file.flush()

    def _format_float(self, value: float) -> str:
        if not math.isfinite(value):
            return ''
        return '%.6f' % value

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        print('saved log to %s' % self.csv_path)


def main(args: Optional[Sequence[str]] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = UwbPositionLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
