import csv
import math
import os
import time
from collections import deque
from datetime import datetime
from typing import Deque
from typing import Optional

import rclpy
from aruco_interfaces.msg import ArucoDistance
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _mean(values: Deque[float]) -> float:
    return sum(values) / len(values)


class VisionCmdLogger(Node):
    def __init__(self):
        super().__init__('vision_cmd_logger')

        self.declare_parameter('output_dir', '/tmp/vision_docking_logs')
        self.declare_parameter('log_rate', 20.0)
        self.declare_parameter('flush_every_rows', 20)
        self.declare_parameter('optitrack_pose_topic', '/vrpn_mocap/RigidBody_1/pose')
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('position_average_window_size', 5)
        self.declare_parameter('detection_timeout', 0.5)

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.log_rate = float(self.get_parameter('log_rate').value)
        self.flush_every_rows = int(self.get_parameter('flush_every_rows').value)
        self.optitrack_pose_topic = str(self.get_parameter('optitrack_pose_topic').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.position_average_window_size = int(
            self.get_parameter('position_average_window_size').value
        )
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)

        if self.log_rate <= 0.0:
            raise ValueError('log_rate must be greater than 0')
        if self.flush_every_rows <= 0:
            raise ValueError('flush_every_rows must be greater than 0')
        if self.position_average_window_size < 1:
            raise ValueError('position_average_window_size must be greater than or equal to 1')
        if self.detection_timeout <= 0.0:
            raise ValueError('detection_timeout must be greater than 0')

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'vision_cmd_log_{stamp}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'elapsed_sec',
            'aruco_z',
            'aruco_yaw',
            'aruco_normalized_center_error',
            'aruco_z_cos_yaw',
            'aruco_z_sin_yaw',
            'raw_estimated_z',
            'raw_estimated_x',
            'estimated_z_average',
            'estimated_x_average',
            'optitrack_x',
            'optitrack_y',
            'optitrack_z',
            'cmd_linear_x',
            'cmd_linear_y',
            'cmd_angular_z',
        ])

        self.start_time = time.perf_counter()
        self.row_count = 0

        self.latest_aruco: Optional[ArucoDistance] = None
        self.latest_raw_estimated_x: Optional[float] = None
        self.latest_raw_estimated_z: Optional[float] = None
        self.latest_estimated_x_average: Optional[float] = None
        self.latest_estimated_z_average: Optional[float] = None
        self.last_aruco_time: Optional[float] = None
        self.estimated_x_buffer: Deque[float] = deque(
            maxlen=self.position_average_window_size
        )
        self.estimated_z_buffer: Deque[float] = deque(
            maxlen=self.position_average_window_size
        )
        self.latest_optitrack_pose: Optional[PoseStamped] = None
        self.latest_cmd: Optional[Twist] = None

        optitrack_best_effort_qos = QoSProfile(depth=10)
        optitrack_best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(ArucoDistance, '/aruco/distance', self.aruco_callback, 10)
        self.create_subscription(
            PoseStamped,
            self.optitrack_pose_topic,
            self.optitrack_pose_callback,
            optitrack_best_effort_qos,
        )
        self.create_subscription(Twist, '/rov_cmd_vel', self.cmd_callback, 10)
        self.create_timer(1.0 / self.log_rate, self.log_callback)

        self.get_logger().info(
            'logging /aruco/distance, %s, and /rov_cmd_vel to %s'
            % (self.optitrack_pose_topic, self.csv_path)
        )

    def aruco_callback(self, msg: ArucoDistance):
        now = time.perf_counter()
        if (
            self.last_aruco_time is not None
            and (now - self.last_aruco_time) > self.detection_timeout
        ):
            self._clear_position_average()

        self.latest_aruco = msg
        raw_estimated_x, raw_estimated_z = self._estimate_wall_relative_position(
            float(msg.z),
            float(msg.yaw),
        )
        self.estimated_x_buffer.append(raw_estimated_x)
        self.estimated_z_buffer.append(raw_estimated_z)
        self.latest_raw_estimated_x = raw_estimated_x
        self.latest_raw_estimated_z = raw_estimated_z
        self.latest_estimated_x_average = _mean(self.estimated_x_buffer)
        self.latest_estimated_z_average = _mean(self.estimated_z_buffer)
        self.last_aruco_time = now

    def optitrack_pose_callback(self, msg: PoseStamped):
        self.latest_optitrack_pose = msg

    def cmd_callback(self, msg: Twist):
        self.latest_cmd = msg

    def log_callback(self):
        elapsed_sec = time.perf_counter() - self.start_time

        aruco = self.latest_aruco
        optitrack_pose = self.latest_optitrack_pose
        cmd = self.latest_cmd
        optitrack_position = optitrack_pose.pose.position if optitrack_pose is not None else None
        aruco_z_cos_yaw = aruco.z * math.cos(aruco.yaw) if aruco is not None else None
        aruco_z_sin_yaw = aruco.z * math.sin(aruco.yaw) if aruco is not None else None
        if (
            self.last_aruco_time is not None
            and (time.perf_counter() - self.last_aruco_time) > self.detection_timeout
        ):
            self._clear_position_average()

        self.writer.writerow([
            '%.4f' % elapsed_sec,
            self._format_optional(aruco.z if aruco is not None else None),
            self._format_optional(aruco.yaw if aruco is not None else None),
            self._format_optional(
                aruco.normalized_center_error if aruco is not None else None
            ),
            self._format_optional(aruco_z_cos_yaw),
            self._format_optional(aruco_z_sin_yaw),
            self._format_optional(self.latest_raw_estimated_z),
            self._format_optional(self.latest_raw_estimated_x),
            self._format_optional(self.latest_estimated_z_average),
            self._format_optional(self.latest_estimated_x_average),
            self._format_optional(
                optitrack_position.x if optitrack_position is not None else None
            ),
            self._format_optional(
                optitrack_position.y if optitrack_position is not None else None
            ),
            self._format_optional(
                optitrack_position.z if optitrack_position is not None else None
            ),
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

    def _estimate_wall_relative_position(
        self,
        aruco_z: float,
        aruco_yaw: float,
    ) -> tuple[float, float]:
        theta = _wrap_pi(aruco_yaw - self.target_yaw)
        estimated_x = aruco_z * math.sin(theta)
        estimated_z = aruco_z * math.cos(theta)
        return estimated_x, estimated_z

    def _clear_position_average(self):
        self.estimated_x_buffer.clear()
        self.estimated_z_buffer.clear()
        self.latest_raw_estimated_x = None
        self.latest_raw_estimated_z = None
        self.latest_estimated_x_average = None
        self.latest_estimated_z_average = None

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        print('saved log to %s' % self.csv_path)


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = VisionCmdLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
