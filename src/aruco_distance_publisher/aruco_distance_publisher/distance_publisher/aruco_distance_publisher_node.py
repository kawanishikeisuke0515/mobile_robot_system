import math
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from aruco_interfaces.msg import ArucoDistance
from rclpy.node import Node


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


def _open_camera():
    # Prefer the default backend first. Some environments crash when V4L2
    # is forced even though plain OpenCV capture works.
    for backend in (cv2.CAP_ANY, cv2.CAP_V4L2):
        cap = cv2.VideoCapture(0, backend)
        if cap.isOpened():
            return cap, backend
        cap.release()

    return None, None


class ArucoDistancePublisher(Node):
    def __init__(self):
        super().__init__("aruco_distance_publisher")
        self.declare_parameter("camera_side", "left")
        self.declare_parameter("marker_length", 0.168)
        self.camera_side = self.get_parameter("camera_side").value
        if self.camera_side not in ("left", "right"):
            raise ValueError("camera_side must be 'left' or 'right'")
        self.marker_length = float(self.get_parameter("marker_length").value)
        if self.marker_length <= 0.0:
            raise ValueError("marker_length must be greater than 0")

        self.publisher_ = self.create_publisher(ArucoDistance, "/aruco/distance", 10)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = _create_detector_parameters()

        calib_path = self._resolve_calibration_path(self.camera_side)
        try:
            calib = np.load(calib_path)
            self.camera_matrix = calib["cameraMatrix"]
            self.dist_coeffs = calib["distCoeffs"]
        except Exception as exc:
            raise RuntimeError(f"failed to load calibration file: {calib_path}") from exc

        self.cap, backend = _open_camera()
        if self.cap is None:
            raise RuntimeError("failed to open camera device 0")

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"loaded calibration from {calib_path}")
        self.get_logger().info(f"using ZED2 {self.camera_side} camera image")
        self.get_logger().info(f"marker length = {self.marker_length:.3f} m")
        self.get_logger().info(f"camera opened with backend id {backend}")
        self.get_logger().info("publishing marker distance on /aruco/distance")

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

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("failed to read frame from camera")
            return

        self.get_logger().debug(f"raw frame shape = {frame.shape}")

        height, width = frame.shape[:2]
        half_width = width // 2

        if self.camera_side == "left":
            frame = frame[:, :half_width]
        else:
            frame = frame[:, half_width:]

        self.get_logger().debug(f"{self.camera_side} frame shape = {frame.shape}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

        if ids is None:
            return

        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs,
        )

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]
            x_m = float(tvec[0])
            y_m = float(tvec[1])
            z_m = float(tvec[2])
            distance_m = math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)
            theta_rad = math.atan2(x_m, z_m)

            self.get_logger().info(
                f"id={int(marker_id)} x={x_m:.3f} y={y_m:.3f} z={z_m:.3f} "
                f"d={distance_m:.3f} theta={theta_rad:.3f}"
            )

            msg = ArucoDistance()
            msg.id = int(marker_id)
            msg.x = x_m
            msg.y = y_m
            msg.z = z_m
            msg.distance = distance_m
            msg.theta = theta_rad
            self.publisher_.publish(msg)

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
