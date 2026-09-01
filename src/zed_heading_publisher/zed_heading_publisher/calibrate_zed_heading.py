import math
import os
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField

from .zed_heading_publisher import calculate_heading
from .zed_heading_publisher import get_axis_parameter
from .zed_heading_publisher import get_axis_value
from .zed_heading_publisher import is_finite


class ZedHeadingCalibrator(Node):
    def __init__(self):
        super().__init__('calibrate_zed_heading')

        self.declare_parameter('mag_topic', '/zed2i/zed_node/imu/mag')
        self.declare_parameter('duration_sec', 30.0)
        self.declare_parameter('min_samples', 20)
        self.declare_parameter('magnetic_field_scale', 1000000.0)
        axis_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('raw_x_axis', 'y', axis_descriptor)
        self.declare_parameter('raw_x_sign', -1.0)
        self.declare_parameter('raw_z_axis', 'x', axis_descriptor)
        self.declare_parameter('raw_z_sign', 1.0)
        self.declare_parameter('progress_log_interval_sec', 2.0)

        self.mag_topic = str(self.get_parameter('mag_topic').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.magnetic_field_scale = float(
            self.get_parameter('magnetic_field_scale').value
        )
        self.raw_x_axis = get_axis_parameter(self.get_parameter('raw_x_axis').value)
        self.raw_x_sign = float(self.get_parameter('raw_x_sign').value)
        self.raw_z_axis = get_axis_parameter(self.get_parameter('raw_z_axis').value)
        self.raw_z_sign = float(self.get_parameter('raw_z_sign').value)
        self.progress_log_interval_sec = float(
            self.get_parameter('progress_log_interval_sec').value
        )
        self._validate_parameters()

        self.sample_count = 0
        self.min_x = math.inf
        self.max_x = -math.inf
        self.min_z = math.inf
        self.max_z = -math.inf
        self.last_raw_x: Optional[float] = None
        self.last_raw_z: Optional[float] = None
        self.start_time = self.get_clock().now()
        self.last_progress_log_time = self.start_time
        self.finished = False

        self.create_subscription(
            MagneticField,
            self.mag_topic,
            self.mag_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'collecting heading calibration samples for %.1f sec from %s; '
            'scale=%.5f raw_x=%+.1f*%s raw_z=%+.1f*%s'
            % (
                self.duration_sec,
                self.mag_topic,
                self.magnetic_field_scale,
                self.raw_x_sign,
                self.raw_x_axis,
                self.raw_z_sign,
                self.raw_z_axis,
            )
        )
        self.get_logger().info(
            'rotate the robot slowly through 360 degrees; keep it at yaw=0 near the end'
        )

    def _validate_parameters(self):
        if self.mag_topic == '':
            raise ValueError('mag_topic must not be empty')
        if self.duration_sec <= 0.0 or not math.isfinite(self.duration_sec):
            raise ValueError('duration_sec must be a finite value greater than 0')
        if self.min_samples < 1:
            raise ValueError('min_samples must be greater than or equal to 1')
        if not math.isfinite(self.magnetic_field_scale):
            raise ValueError('magnetic_field_scale must be finite')
        if not is_finite(
            self.raw_x_sign,
            self.raw_z_sign,
            self.progress_log_interval_sec,
        ):
            raise ValueError('numeric parameters must be finite')
        if self.raw_x_sign == 0.0 or self.raw_z_sign == 0.0:
            raise ValueError('raw_x_sign and raw_z_sign must be non-zero')
        if self.progress_log_interval_sec < 0.0:
            raise ValueError(
                'progress_log_interval_sec must be greater than or equal to 0'
            )

    def mag_callback(self, msg: MagneticField):
        if self.finished:
            return

        raw_x = (
            self.raw_x_sign
            * get_axis_value(msg.magnetic_field, self.raw_x_axis)
            * self.magnetic_field_scale
        )
        raw_z = (
            self.raw_z_sign
            * get_axis_value(msg.magnetic_field, self.raw_z_axis)
            * self.magnetic_field_scale
        )
        if not is_finite(raw_x, raw_z):
            self.get_logger().warn(
                'skipping invalid sample: raw_x=%s raw_z=%s' % (raw_x, raw_z)
            )
            return

        self.sample_count += 1
        self.min_x = min(self.min_x, raw_x)
        self.max_x = max(self.max_x, raw_x)
        self.min_z = min(self.min_z, raw_z)
        self.max_z = max(self.max_z, raw_z)
        self.last_raw_x = raw_x
        self.last_raw_z = raw_z

    def timer_callback(self):
        if self.finished:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if self.progress_log_interval_sec > 0.0:
            progress_elapsed = (now - self.last_progress_log_time).nanoseconds / 1e9
            if progress_elapsed >= self.progress_log_interval_sec:
                self.last_progress_log_time = now
                self._log_progress(elapsed)

        if elapsed >= self.duration_sec:
            self.finished = True
            self._finish()
            rclpy.shutdown()

    def _log_progress(self, elapsed: float):
        remaining = max(0.0, self.duration_sec - elapsed)
        if self.sample_count == 0:
            self.get_logger().info(
                'collecting... samples=0 remaining=%.1f sec' % remaining
            )
            return

        self.get_logger().info(
            'collecting... samples=%d remaining=%.1f sec '
            'raw_x=[%.5f, %.5f] raw_z=[%.5f, %.5f]'
            % (
                self.sample_count,
                remaining,
                self.min_x,
                self.max_x,
                self.min_z,
                self.max_z,
            )
        )

    def _finish(self):
        if self.sample_count == 0 or self.last_raw_x is None or self.last_raw_z is None:
            self.get_logger().error(
                'no samples received for calibration; check mag_topic and ZED data hub'
            )
            return
        if self.sample_count < self.min_samples:
            self.get_logger().warn(
                'sample count is lower than recommended: samples=%d min_samples=%d'
                % (self.sample_count, self.min_samples)
            )

        center_x = (self.min_x + self.max_x) / 2.0
        center_z = (self.min_z + self.max_z) / 2.0
        x_radius = (self.max_x - self.min_x) / 2.0
        z_radius = (self.max_z - self.min_z) / 2.0
        zero_sample = calculate_heading(
            raw_x=self.last_raw_x,
            raw_z=self.last_raw_z,
            center_x=center_x,
            center_z=center_z,
            zero_heading_deg=0.0,
        )
        zero_heading_deg = zero_sample.magnetic_heading_deg

        self.get_logger().info('calibration complete')
        print('', flush=True)
        print('zed_heading_publisher:', flush=True)
        print('  ros__parameters:', flush=True)
        print(f'    mag_topic: {self.mag_topic}', flush=True)
        print(f'    center_x: {center_x:.6f}', flush=True)
        print(f'    center_z: {center_z:.6f}', flush=True)
        print(f'    zero_heading_deg: {zero_heading_deg:.6f}', flush=True)
        print('    publish_rate_hz: 20.0', flush=True)
        print('    frame_id: "zed2i_mag"', flush=True)
        print('    invert_yaw: false', flush=True)
        print(f'    magnetic_field_scale: {self.magnetic_field_scale:.6f}', flush=True)
        print('    diagnostic_log_interval_sec: 1.0', flush=True)
        print(f'    raw_x_axis: "{self.raw_x_axis}"', flush=True)
        print(f'    raw_x_sign: {self.raw_x_sign:.1f}', flush=True)
        print(f'    raw_z_axis: "{self.raw_z_axis}"', flush=True)
        print(f'    raw_z_sign: {self.raw_z_sign:.1f}', flush=True)
        print('', flush=True)
        print('# diagnostics:', flush=True)
        print(f'# samples: {self.sample_count}', flush=True)
        print(f'# raw_x_min: {self.min_x:.6f}', flush=True)
        print(f'# raw_x_max: {self.max_x:.6f}', flush=True)
        print(f'# raw_z_min: {self.min_z:.6f}', flush=True)
        print(f'# raw_z_max: {self.max_z:.6f}', flush=True)
        print(f'# raw_x_radius: {x_radius:.6f}', flush=True)
        print(f'# raw_z_radius: {z_radius:.6f}', flush=True)
        print(
            '# radius_ratio_x_over_z: %.6f'
            % (x_radius / z_radius if z_radius != 0.0 else math.inf),
            flush=True,
        )


def main(args: Optional[list] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = ZedHeadingCalibrator()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
