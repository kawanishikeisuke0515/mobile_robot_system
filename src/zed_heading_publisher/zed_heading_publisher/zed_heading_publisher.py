import math
import os
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from zed_interfaces.msg import ZedHeading

try:
    import pyzed.sl as sl
except ImportError:
    sl = None


def normalize_to_180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class HeadingSample:
    raw_x: float
    raw_z: float
    corrected_x: float
    corrected_z: float
    magnetic_heading_deg: float
    robot_yaw_deg: float
    robot_yaw_rad: float


def calculate_heading(
    raw_x: float,
    raw_z: float,
    center_x: float,
    center_z: float,
    zero_heading_deg: float,
    invert_yaw: bool = False,
) -> HeadingSample:
    corrected_x = raw_x - center_x
    corrected_z = raw_z - center_z

    magnetic_heading_deg = normalize_to_180(
        math.degrees(math.atan2(corrected_x, corrected_z))
    )
    robot_yaw_deg = magnetic_heading_deg - zero_heading_deg
    if invert_yaw:
        robot_yaw_deg = -robot_yaw_deg
    robot_yaw_deg = normalize_to_180(robot_yaw_deg)
    robot_yaw_rad = math.radians(robot_yaw_deg)

    return HeadingSample(
        raw_x=raw_x,
        raw_z=raw_z,
        corrected_x=corrected_x,
        corrected_z=corrected_z,
        magnetic_heading_deg=magnetic_heading_deg,
        robot_yaw_deg=robot_yaw_deg,
        robot_yaw_rad=robot_yaw_rad,
    )


def is_finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


class ZedHeadingPublisher(Node):
    def __init__(self):
        super().__init__('zed_heading_publisher')

        self.declare_parameter('center_x', -2.5354)
        self.declare_parameter('center_z', -10.3439)
        self.declare_parameter('zero_heading_deg', 40.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'zed2i_mag')
        self.declare_parameter('invert_yaw', False)

        self.center_x = float(self.get_parameter('center_x').value)
        self.center_z = float(self.get_parameter('center_z').value)
        self.zero_heading_deg = float(self.get_parameter('zero_heading_deg').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self._validate_parameters()

        self.publisher_ = self.create_publisher(ZedHeading, '/zed/heading', 10)
        self.zed = self._open_zed()
        self.sensors_data = sl.SensorsData()
        self.last_sensor_warn_time = None

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info(
            'center_x=%.5f center_z=%.5f zero_heading_deg=%.5f '
            'publish_rate_hz=%.3f frame_id=%s invert_yaw=%s'
            % (
                self.center_x,
                self.center_z,
                self.zero_heading_deg,
                self.publish_rate_hz,
                self.frame_id,
                self.invert_yaw,
            )
        )
        self.get_logger().info('publishing ZED2i heading on /zed/heading')

    def _validate_parameters(self):
        if not is_finite(self.center_x, self.center_z, self.zero_heading_deg):
            raise ValueError(
                'center_x, center_z, and zero_heading_deg must be finite values'
            )
        if self.publish_rate_hz <= 0.0 or not math.isfinite(self.publish_rate_hz):
            raise ValueError('publish_rate_hz must be a finite value greater than 0')
        if self.frame_id == '':
            raise ValueError('frame_id must not be empty')

    def _open_zed(self):
        if sl is None:
            raise RuntimeError('pyzed.sl is not available; install the ZED SDK')

        zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NONE

        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'failed to open ZED2i: {err}')
        return zed

    def timer_callback(self):
        err = self.zed.get_sensors_data(
            self.sensors_data,
            sl.TIME_REFERENCE.CURRENT,
        )
        if err != sl.ERROR_CODE.SUCCESS:
            self._warn_throttled(f'failed to get ZED sensor data: {err}')
            return

        try:
            mag = self.sensors_data.get_magnetometer_data()
            field = mag.get_magnetic_field_uncalibrated()
            raw_x = float(field[0])
            raw_z = float(field[2])
        except Exception as exc:
            self._warn_throttled(f'failed to read magnetometer data: {exc}')
            return

        if not is_finite(raw_x, raw_z):
            self._warn_throttled(
                f'invalid magnetometer data: raw_x={raw_x} raw_z={raw_z}'
            )
            return

        sample = calculate_heading(
            raw_x=raw_x,
            raw_z=raw_z,
            center_x=self.center_x,
            center_z=self.center_z,
            zero_heading_deg=self.zero_heading_deg,
            invert_yaw=self.invert_yaw,
        )
        self.publisher_.publish(self._build_message(sample))

    def _build_message(self, sample: HeadingSample) -> ZedHeading:
        msg = ZedHeading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.raw_x = sample.raw_x
        msg.raw_z = sample.raw_z
        msg.corrected_x = sample.corrected_x
        msg.corrected_z = sample.corrected_z
        msg.magnetic_heading_deg = sample.magnetic_heading_deg
        msg.robot_yaw_deg = sample.robot_yaw_deg
        msg.robot_yaw_rad = sample.robot_yaw_rad
        msg.valid = True
        return msg

    def _warn_throttled(self, message: str, interval_sec: float = 2.0):
        now = self.get_clock().now()
        if self.last_sensor_warn_time is not None:
            elapsed = (now - self.last_sensor_warn_time).nanoseconds / 1e9
            if elapsed < interval_sec:
                return

        self.last_sensor_warn_time = now
        self.get_logger().warn(message)

    def destroy_node(self):
        if hasattr(self, 'zed') and self.zed is not None:
            self.zed.close()
            self.zed = None
        super().destroy_node()


def main(args: Optional[list] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = ZedHeadingPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
