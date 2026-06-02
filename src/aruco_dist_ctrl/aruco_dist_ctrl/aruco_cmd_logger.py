import csv
import os
import time
from datetime import datetime
from typing import Optional

import rclpy
from aruco_interfaces.msg import ArucoDistance
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ArucoCmdLogger(Node):
    def __init__(self):
        super().__init__('aruco_cmd_logger')

        self.declare_parameter('output_dir', '/tmp/aruco_docking_logs')
        self.declare_parameter('log_rate', 20.0)
        self.declare_parameter('flush_every_rows', 20)

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.log_rate = float(self.get_parameter('log_rate').value)
        self.flush_every_rows = int(self.get_parameter('flush_every_rows').value)

        if self.log_rate <= 0.0:
            raise ValueError('log_rate must be greater than 0')
        if self.flush_every_rows <= 0:
            raise ValueError('flush_every_rows must be greater than 0')

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'aruco_cmd_log_{stamp}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'elapsed_sec',
            'aruco_x',
            'aruco_z',
            'aruco_theta',
            'aruco_yaw',
            'cmd_linear_x',
            'cmd_linear_y',
            'cmd_angular_z',
        ])

        self.start_time = time.perf_counter()
        self.row_count = 0

        self.latest_aruco: Optional[ArucoDistance] = None
        self.latest_cmd: Optional[Twist] = None

        self.create_subscription(ArucoDistance, '/aruco/distance', self.aruco_callback, 10)
        self.create_subscription(Twist, '/rov_cmd_vel', self.cmd_callback, 10)
        self.create_timer(1.0 / self.log_rate, self.log_callback)

        self.get_logger().info('logging /aruco/distance and /rov_cmd_vel to %s' % self.csv_path)

    def aruco_callback(self, msg: ArucoDistance):
        self.latest_aruco = msg

    def cmd_callback(self, msg: Twist):
        self.latest_cmd = msg

    def log_callback(self):
        elapsed_sec = time.perf_counter() - self.start_time

        aruco = self.latest_aruco
        cmd = self.latest_cmd

        self.writer.writerow([
            '%.4f' % elapsed_sec,
            self._format_optional(aruco.x if aruco is not None else None),
            self._format_optional(aruco.z if aruco is not None else None),
            self._format_optional(aruco.theta if aruco is not None else None),
            self._format_optional(aruco.yaw if aruco is not None else None),
            self._format_optional(cmd.linear.x if cmd is not None else None),
            self._format_optional(cmd.linear.y if cmd is not None else None),
            self._format_optional(cmd.angular.z if cmd is not None else None),
        ])

        self.row_count += 1
        if self.row_count % self.flush_every_rows == 0:
            self.csv_file.flush()

    def _format_optional(self, value: Optional[float]) -> str:
        if value is None:
            return ''
        return '%.6f' % float(value)

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        print('saved log to %s' % self.csv_path)


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = ArucoCmdLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
