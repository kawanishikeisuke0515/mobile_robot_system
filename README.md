# Mobile Robot System

## Overview
This ROS2 workspace contains the mobile robot packages for ArUco marker distance publishing, distance-based docking control, and wheel/motor command output.

## Workspace Layout
- This directory itself is a ROS2 workspace root
- Packages are under `./src`
- Build from this repository root
- Main packages:
  - `aruco_distance_publisher`: detects ArUco markers and publishes marker distance
  - `aruco_dist_ctrl`: subscribes to marker distance and publishes velocity commands
  - `aruco_docking_bringup`: launch files for the full docking stack
  - `locomotion_core`: wheel velocity and motor controller nodes
  - `aruco_interfaces`: custom ArUco message definitions

## Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run Distance Publisher
```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run aruco_distance_publisher aruco_distance_publisher
```

Use the right camera image instead of the default left image:

```bash
ros2 run aruco_distance_publisher aruco_distance_publisher --ros-args -p camera_side:=right
```

Set a different ArUco marker side length in meters:

```bash
ros2 run aruco_distance_publisher aruco_distance_publisher --ros-args -p marker_length:=0.168
```

If `ros2 run` crashes with a segmentation fault, it is often caused by DDS network discovery on the host. `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` forces local-only discovery and avoids that on many machines.

## Inspect Published Distance
```bash
source install/setup.bash
ros2 topic echo /aruco/distance
```

## Run Docking Stack
```bash
source install/setup.bash
ros2 launch aruco_docking_bringup aruco_docking.launch.py
```

## Run Keyboard Teleop
```bash
source install/setup.bash
ros2 launch locomotion_core keyboard_teleop.launch.py
```

This launch publishes keyboard `cmd_vel` commands to `/rov_cmd_vel` and starts:

- `teleop_twist_keyboard`
- `locomotion_core/rover_velocity`
- `locomotion_core/cmd_roboteq`

If keyboard input is not captured by the launch terminal, run teleop in a separate terminal window:

```bash
ros2 launch locomotion_core keyboard_teleop.launch.py teleop_prefix:="xterm -e"
```

## Distance Publisher Node Info
- Node name: aruco_distance_publisher
- Language: Python (rclpy + OpenCV)
- Parameter:
  - `camera_side`: `left` or `right` (`left` by default)
  - `marker_length`: ArUco marker side length in meters (`0.168` by default)

## Camera Input
- Capture source: ZED2 stereo camera
- OpenCV API: cv2.VideoCapture(0, backend)
- Preferred backend: cv2.CAP_ANY
- Fallback backend: cv2.CAP_V4L2
- The ZED2 frame is treated as side-by-side stereo input.
- `camera_side=left` uses the left half of the frame.
- `camera_side=right` uses the right half of the frame.

## ROS2 Output
- Publish topic: /aruco/distance
- Publish data:
  - id
  - x
  - y
  - z
  - distance
  - theta
  - yaw
  - center_u
  - center_v
  - normalized_center_error
  
### Required mapping

In this system, robot forward motion is represented by `linear.x`.

Therefore, ArUco camera-frame values must be remapped before publishing
robot velocity commands:

- ArUco `z` → robot `linear.x` forward/backward motion
- ArUco `x` → robot `linear.y` lateral motion
- ArUco `normalized_center_error` → robot `angular.z` marker-centering motion
- ArUco `yaw` → logged for evaluating docking posture

Example:

cmd.linear.x = forward_cmd   # based on ArUco z
cmd.linear.y = lateral_cmd   # based on -ArUco x
cmd.angular.z = center_cmd   # based on normalized_center_error

The current controller uses `linear.x`, `linear.y`, and `angular.z`.
During PRE_DOCKING and FINAL_DOCKING, `angular.z` is controlled to keep the marker near the image center. Marker yaw is logged so experiments can check whether the robot naturally approaches a perpendicular docking posture.

## Camera Calibration
- Left calibration file path when running from source:
  `src/aruco_distance_publisher/aruco_distance_publisher/distance_publisher/calib_result_left.npz`
- Right calibration file path when running from source:
  `src/aruco_distance_publisher/aruco_distance_publisher/distance_publisher/calib_result_right.npz`
- Calibration file path after install:
  `install/aruco_distance_publisher/share/aruco_distance_publisher/distance_publisher/`
- File format: numpy .npz
- Keys:
  - cameraMatrix
  - distCoeffs
- Load once at startup

## ArUco Settings
- Dictionary: DICT_5X5_50
- Default marker length: 0.168 m

## Processing Flow
1. Open the ZED2 camera using cv2.VideoCapture(0, backend)
2. Read frame
3. Crop the left or right half of the frame based on `camera_side`
4. Convert frame to grayscale
5. Detect ArUco markers
6. Estimate pose using cameraMatrix and distCoeffs
7. For each detected marker:
   - x, y, z = tvec
   - distance = sqrt(x^2 + y^2 + z^2)
   - theta = atan2(x, z)
   - yaw = marker orientation angle from rvec
   - center_u, center_v = average of the four detected marker corner pixels
   - normalized_center_error = (center_u - image_width / 2) / (image_width / 2)
8. Publish result to ROS2 topic

## Error Handling
- If camera open fails: log error and exit
- If calibration file not found: log error and exit
- If no marker detected: do not publish
