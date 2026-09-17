"""Subscribe to docking telemetry and record CSV files without commanding the robot."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32MultiArray, String
from uwb_interfaces.msg import UwbPosition, UwbControlError
from zed_interfaces.msg import ZedHeading
from aruco_interfaces.msg import ArucoDistance
from mode_manager_interfaces.msg import ControllerOutput

from .core import CsvWriter, Samples, decode

STREAMS = {
    'cmd_vel': (Twist, '/rov_cmd_vel'),
    'motor_commands': (Float32MultiArray, '/rov/motors'),
    'deadman': (Bool, '/deadman'),
    'uwb_control_error': (UwbControlError, '/uwb/control_error'),
    'uwb': (UwbPosition, '/uwb/position'),
    'uwb_robot_pose': (PoseStamped, '/uwb/robot_pose'),
    'zed_heading': (ZedHeading, '/zed/heading'),
    'optitrack': (PoseStamped, '/vrpn_mocap/RigidBody_1/pose'),
    'vision': (ArucoDistance, '/aruco/distance'),
    'control_state': (String, '/mode_manager/state'),
    'uwb_control_output': (ControllerOutput, '/uwb/control_output'),
    'vision_control_output': (ControllerOutput, '/vision/control_output'),
}


class DockingLogger(Node):
    def __init__(self):
        super().__init__('docking_logger')
        self.writer = None
        self.started = time.monotonic()
        self.params = {}
        self.subscriptions_ = []
        try:
            self.configure()
        except BaseException as exc:
            if self.writer is not None:
                self.writer.close(repr(exc))
            self.destroy_node()
            raise

    def parameter(self, name, default):
        value = self.declare_parameter(
            name, default, ParameterDescriptor(read_only=True)).value
        self.params[name] = value
        return value

    def positive(self, name, default):
        value = self.parameter(name, default)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def configure(self):
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Initial docking_logger supports real time only')
        log_dir = self.parameter('log_dir', '~/docking_logs')
        if not log_dir.strip():
            raise ValueError('log_dir must not be empty')
        name = self.parameter('experiment_name', 'docking')
        note = self.parameter('experiment_note', '')
        target = self.parameter('target_marker_id', -1)
        if target < 0:
            raise ValueError('target_marker_id must be specified and nonnegative')
        rate = self.positive('log_rate', 20.0)
        flush = self.positive('flush_interval_sec', 1.0)
        capacity = self.positive('queue_capacity', 10000)
        stale = self.positive('stale_timeout_sec', 1.0)
        depth = self.positive('qos_depth', 100)
        config_paths = json.loads(self.parameter('config_paths_json', '{}'))
        overrides = json.loads(self.parameter('launch_overrides_json', '{}'))
        snapshots = {}
        for label, path in config_paths.items():
            resolved = Path(path).expanduser().resolve()
            snapshots[label] = {'path': str(resolved), 'content': resolved.read_text(encoding='utf-8')}
        thresholds = {}
        qos_settings = {}
        for key, (message_type, topic_default) in STREAMS.items():
            topic_param = 'optitrack_pose_topic' if key == 'optitrack' else key + '_topic'
            topic = self.parameter(topic_param, topic_default)
            if not topic.strip():
                raise ValueError(f'{topic_param} must not be empty')
            thresholds[key] = self.positive(key + '_stale_timeout_sec', stale)
            reliability = self.parameter(key + '_reliability',
                                         'best_effort' if key == 'optitrack' else 'reliable')
            if reliability not in ('reliable', 'best_effort'):
                raise ValueError(f'Invalid reliability for {key}: {reliability}')
            qos = QoSProfile(depth=depth, durability=DurabilityPolicy.VOLATILE,
                             reliability=ReliabilityPolicy.RELIABLE if reliability == 'reliable'
                             else ReliabilityPolicy.BEST_EFFORT)
            sub = self.create_subscription(message_type, topic,
                                           lambda msg, stream=key: self.receive(stream, msg), qos)
            self.subscriptions_.append(sub)
            qos_settings[key] = {
                'topic': sub.topic_name,
                'type': message_type.__module__.split('.')[0] + '/msg/' + message_type.__name__,
                'reliability': reliability, 'durability': 'volatile', 'depth': depth}
        self.samples = Samples(target, thresholds)
        metadata = {
            'experiment_name': name, 'experiment_note': note, 'parameters': self.params,
            'streams': qos_settings, 'config_snapshots': snapshots,
            'launch_overrides': overrides,
            'external_effective_settings': 'unknown; only supplied configuration is captured',
            'time': {'receive': 'ROS clock nanoseconds', 'elapsed': 'monotonic seconds',
                     'source': 'original message stamp, blank when absent'},
            'coordinates': {'uwb': 'anchor-defined x/y [m]',
                            'uwb_robot_pose': 'robot center in source frame [m], quaternion xyzw; '
                                              'tag offset corrected by publisher',
                            'uwb_control_error': 'controller world/body errors [m], yaw [rad]; '
                                                 'distance is raw world error norm',
                            'zed_heading': 'configured robot yaw [rad/deg]',
                            'optitrack': 'original frame position [m], quaternion xyzw',
                            'vision': 'marker relative to camera: right x, down y, forward z [m]; angles [rad]',
                            'transforms': 'unknown; no coordinate transformation applied',
                            'motor_commands': 'driver input, not measured RPM'},
        }
        self.writer = CsvWriter(log_dir, name, metadata, capacity, flush)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.last_drop_warning = float('-inf')
        self.get_logger().info(f'Recording docking logs: {self.writer.path}')

    def submit(self, key, row):
        if self.writer.error:
            raise RuntimeError(f'Docking log write failed: {self.writer.error}')
        if not self.writer.submit(key, row):
            now = time.monotonic()
            if now - self.last_drop_warning >= 5.0:
                self.get_logger().warning(f'Logging queue full; dropped rows: {self.writer.dropped}')
                self.last_drop_warning = now

    def receive(self, key, msg):
        elapsed = time.monotonic() - self.started
        ros_ns = self.get_clock().now().nanoseconds
        self.submit(key, self.samples.receive(key, decode(key, msg), ros_ns, elapsed))

    def tick(self):
        self.submit('timeline', self.samples.snapshot(
            self.get_clock().now().nanoseconds, time.monotonic() - self.started))


def main(args=None):
    rclpy.init(args=args)
    node = None
    failure = None
    try:
        node = DockingLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        failure = repr(exc)
        if node:
            node.get_logger().error(f'Docking logger failed: {failure}')
        raise
    finally:
        try:
            if node:
                try:
                    node.writer.close(failure)
                    if node.writer.error and failure is None:
                        raise RuntimeError(node.writer.error)
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
