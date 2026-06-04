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
- `yaw`: marker orientation angle from rvec in radians
- `center_u`: marker center u coordinate in image pixels
- `center_v`: marker center v coordinate in image pixels
- `normalized_center_error`: horizontal marker-center error normalized by half image width

## Pixel Center and Image-Center Error

The publisher computes the marker center from the four detected ArUco corner pixels:

```text
center_u = (u1 + u2 + u3 + u4) / 4
center_v = (v1 + v2 + v3 + v4) / 4
normalized_center_error = (center_u - image_width / 2) / (image_width / 2)
```

`normalized_center_error` is:

- `0.0` when the marker center is horizontally centered in the image
- positive when the marker center is to the image-right side
- negative when the marker center is to the image-left side
- near `+1.0` or `-1.0` when the marker center is close to the horizontal image edge

Downstream controllers use this value to decide whether it is safe to start marker-yaw-based perpendicular alignment without rotating the marker out of view.

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
