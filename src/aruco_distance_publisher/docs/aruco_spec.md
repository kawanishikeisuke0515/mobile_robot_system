# ArUco Distance Publisher

This package detects ArUco markers and publishes distance information.

## Overview
- Uses OpenCV and ArUco library for marker detection
- Publishes ArucoDistance messages with marker ID and distance

## Topics
- `/aruco/distance`: `aruco_interfaces/msg/ArucoDistance`

## Message Fields
- `id`: detected marker ID
- `x`: marker translation in the camera frame, positive to the right
- `y`: marker translation in the camera frame, positive down
- `z`: marker translation in the camera frame, positive forward (away from the camera)
- `distance`: Euclidean distance to the marker in meters
- `theta`: horizontal marker angle in radians

## Coordinate frame
- `ArucoDistance` is published in the OpenCV camera coordinate frame as returned by `cv2.aruco.estimatePoseSingleMarkers()`.
- In this frame, `x` = right, `y` = down, `z` = forward.

## Nodes
- `aruco_distance_publisher`: Main node for detection and publishing

## Downstream Integration
- `aruco_dist_ctrl/aruco_distance_controller` subscribes to `/aruco/distance`
- The controller publishes `/rov_cmd_vel` for `locomotion_core/rover_velocity`

## Dependencies
- OpenCV
- aruco_interfaces

## Usage
1. Build: `colcon build --packages-select aruco_distance_publisher`
2. Run: `ros2 run aruco_distance_publisher aruco_distance_publisher`
