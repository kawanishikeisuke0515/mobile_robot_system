import math
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from aruco_interfaces.msg import ArucoDistance
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def _debug(message):
    print(f"[aruco_distance_publisher] {message}", flush=True)


def _create_detector_parameters():
    # OpenCV 4.6 on this machine exposes DetectorParameters(), but that path
    # can crash. The factory API is more stable across 4.x releases.
    parameters = cv2.aruco.DetectorParameters_create()

    parameters.minMarkerPerimeterRate = 0.02
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    return parameters


def _get_bool_parameter(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off"):
            return False
    raise ValueError(f"invalid bool parameter value: {value}")


def _marker_yaw_from_rvec(rvec) -> float:
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    marker_z_axis = rotation_matrix[:, 2]

    # Use the marker plane normal to estimate yaw around the camera Y axis.
    # When the marker faces the camera, OpenCV may report the normal along
    # either camera +Z or -Z depending on marker coordinate convention, so use
    # the normal direction that makes the front-facing pose yaw close to zero.
    z_axis_reference = -marker_z_axis[2] if marker_z_axis[2] < 0.0 else marker_z_axis[2]
    return math.atan2(float(marker_z_axis[0]), float(z_axis_reference))


class ArucoDistancePublisher(Node):
    def __init__(self):
        super().__init__("aruco_distance_publisher")
        self.declare_parameter(
            "image_topic",
            "/zed2i/zed_node/rgb/color/rect/image",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/zed2i/zed_node/rgb/color/rect/camera_info",
        )
        self.declare_parameter("use_camera_info", True)
        self.declare_parameter("camera_side", "left")
        self.declare_parameter("marker_length", 0.168)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.use_camera_info = _get_bool_parameter(
            self.get_parameter("use_camera_info").value
        )
        self.camera_side = str(self.get_parameter("camera_side").value)
        self.marker_length = float(self.get_parameter("marker_length").value)
        self._validate_parameters()

        self.publisher_ = self.create_publisher(ArucoDistance, "/aruco/distance", 10)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = _create_detector_parameters()
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_warn_time = None

        if self.use_camera_info:
            self.get_logger().info("waiting for camera calibration from CameraInfo")
        else:
            calib_path = self._resolve_calibration_path(self.camera_side)
            try:
                calib = np.load(calib_path)
                self.camera_matrix = calib["cameraMatrix"]
                self.dist_coeffs = calib["distCoeffs"]
            except Exception as exc:
                raise RuntimeError(
                    f"failed to load calibration file: {calib_path}"
                ) from exc
            self.get_logger().info(f"loaded calibration from {calib_path}")

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = None
        if self.camera_info_topic:
            self.camera_info_subscription = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                qos_profile_sensor_data,
            )

        self.get_logger().info(f"marker length = {self.marker_length:.3f} m")
        self.get_logger().info(f"subscribing image topic: {self.image_topic}")
        if self.camera_info_subscription is not None:
            self.get_logger().info(
                f"subscribing camera info topic: {self.camera_info_topic}"
            )
        self.get_logger().info("publishing marker distance on /aruco/distance")

    def _validate_parameters(self):
        if self.image_topic == "":
            raise ValueError("image_topic must not be empty")
        if not self.use_camera_info and self.camera_side not in ("left", "right"):
            raise ValueError("camera_side must be 'left' or 'right'")
        if self.marker_length <= 0.0:
            raise ValueError("marker_length must be greater than 0")

    def _resolve_calibration_path(self, camera_side: str) -> Path:
        package_dir = Path(__file__).resolve().parent
        candidate_names = [f"calib_result_{camera_side}.npz"]

        share_dir = Path(get_package_share_directory("aruco_distance_publisher"))
        for name in candidate_names:
            local_path = package_dir / name
            if local_path.exists():
                return local_path

            share_path = share_dir / "distance_publisher" / name
            if share_path.exists():
                return share_path

        raise FileNotFoundError(
            f"calibration file for {camera_side} camera was not found in source or install tree"
        )

    def camera_info_callback(self, msg: CameraInfo):
        if not self.use_camera_info:
            return

        camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        dist_coeffs = np.array(msg.d, dtype=np.float64)
        if camera_matrix.shape != (3, 3) or dist_coeffs.size == 0:
            self._warn_throttled("invalid CameraInfo calibration data")
            return
        if (
            not np.all(np.isfinite(camera_matrix))
            or not np.all(np.isfinite(dist_coeffs))
        ):
            self._warn_throttled("non-finite CameraInfo calibration data")
            return

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.get_logger().debug("updated calibration from CameraInfo")

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self._warn_throttled("camera calibration is not available yet")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self._warn_throttled(f"failed to convert image message: {exc}")
            return

        image_width = frame.shape[1]
        self.get_logger().debug(f"image frame shape = {frame.shape}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs,
        )

        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            # OpenCV camera coordinate frame:
            # x = right, y = down, z = forward
            x_m = float(tvec[0])
            y_m = float(tvec[1])
            z_m = float(tvec[2])
            distance_m = math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)
            theta_rad = math.atan2(x_m, z_m)
            yaw_rad = _marker_yaw_from_rvec(rvec)
            marker_corners = corners[i][0]
            center_u = float(np.mean(marker_corners[:, 0]))
            center_v = float(np.mean(marker_corners[:, 1]))
            normalized_center_error = (
                center_u - image_width / 2.0
            ) / (image_width / 2.0)

            self.get_logger().info(
                f"id={int(marker_id)} x={x_m:.3f} y={y_m:.3f} z={z_m:.3f} "
                f"d={distance_m:.3f} theta={theta_rad:.3f} yaw={yaw_rad:.3f} "
                f"u={center_u:.1f} v={center_v:.1f} center_error={normalized_center_error:.3f}"
            )

            msg = ArucoDistance()
            msg.id = int(marker_id)
            msg.x = x_m
            msg.y = y_m
            msg.z = z_m
            msg.distance = distance_m
            msg.theta = theta_rad
            msg.yaw = yaw_rad
            msg.center_u = center_u
            msg.center_v = center_v
            msg.normalized_center_error = normalized_center_error
            self.publisher_.publish(msg)

    def _warn_throttled(self, message: str, interval_sec: float = 2.0):
        now = self.get_clock().now()
        if self.last_warn_time is not None:
            elapsed = (now - self.last_warn_time).nanoseconds / 1e9
            if elapsed < interval_sec:
                return

        self.last_warn_time = now
        self.get_logger().warn(message)


def main(args=None):
    # Keep discovery local by default. This reduces Fast DDS trouble on
    # machines with unusual network interfaces and is enough for local testing.
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")

    _debug("calling rclpy.init()")
    rclpy.init(args=args)
    node = None
    try:
        _debug("constructing ArucoDistancePublisher")
        node = ArucoDistancePublisher()
        _debug("starting spin loop")
        rclpy.spin(node)
    finally:
        _debug("shutting down node")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
