# ArUco Distance Controller (v0.4 Moving-Average Docking Control)

## Overview

This node controls the robot's forward/backward, lateral, and yaw motion based on ArUco marker pose information.

The robot is equipped with omni wheels, so position control and posture control can be handled independently through `linear.x`, `linear.y`, and `angular.z`.

The objective is to autonomously dock to a docking station by first moving from the initial pose `P0` to a pre-docking pose `P1`, and then moving from `P1` to the final docking pose `P2`.

This version applies a configurable moving average to the wall-relative translational estimates `estimated_x` and `estimated_z`. The default window size is `1`, which preserves the previous behavior.

---

## Node Name

```text
aruco_distance_controller
```

## Package

```text
aruco_dist_ctrl
```

---

## Subscribed Topic

```text
/aruco/distance
type: aruco_interfaces/msg/ArucoDistance
```

### Expected Fields

```text
x   : marker translation right from the camera in meters
z   : marker translation forward from the camera in meters
theta : bearing angle to the marker center in radians, calculated as atan2(x, z)
yaw : marker orientation angle from rvec in radians
center_u : marker center u coordinate in image pixels
center_v : marker center v coordinate in image pixels
normalized_center_error : horizontal marker-center error normalized by half image width
```

### ArUcoDistance Axis Definition

- `x`: right direction in camera frame
- `y`: down direction in camera frame
- `z`: forward direction in camera frame

The controller uses `z` to compute forward/backward motion, `x` to compute lateral motion, and `normalized_center_error` to compute angular motion.

- `theta`: published and logged as the marker-center bearing, but not used for angular control in this version.
- `yaw`: published and logged to evaluate whether the robot naturally becomes perpendicular to the docking station.
- `normalized_center_error`: used to rotate the robot so the marker stays near the image center.

`aruco_distance_publisher` computes the marker image center from the four detected ArUco corners:

```text
center_u = (u1 + u2 + u3 + u4) / 4
center_v = (v1 + v2 + v3 + v4) / 4
normalized_center_error = (center_u - image_width / 2) / (image_width / 2)
```

`normalized_center_error` is near `0.0` when the marker is horizontally centered in the image. It approaches `+1.0` near the right edge and `-1.0` near the left edge.

This requires adding `center_u`, `center_v`, and `normalized_center_error` to `aruco_interfaces/msg/ArucoDistance.msg` and populating them in `aruco_distance_publisher`.

---

## Published Topic

```text
/rov_cmd_vel
type: geometry_msgs/msg/Twist
```

### Used Fields

```text
linear.x  : forward/backward velocity [m/s]
linear.y  : lateral velocity [m/s]
angular.z : yaw velocity [rad/s]
```

This topic is consumed by `locomotion_core/rover_velocity`.

---

## Control Strategy

The controller has two states:

```text
PRE_DOCKING
    ↓ final approach condition OK
FINAL_DOCKING
```

### Position Estimation And Error Definition

```text
theta = wrap_pi(aruco_yaw - target_yaw)
d = aruco_z

raw_estimated_x = d * sin(theta)
raw_estimated_z = d * cos(theta)

estimated_x = moving_average(raw_estimated_x)
estimated_z = moving_average(raw_estimated_z)

forward_error = estimated_z - target_z
lateral_error = estimated_x - target_x
center_error = normalized_center_error
```

`aruco_theta` is positive when the marker center is to the camera-right side. In this version, `aruco_theta` is published by the distance publisher but not used by the controller.

`theta` in the controller is the marker yaw error, which represents the tilt between the robot and the docking wall. The controller uses `aruco_z` as `d`, assuming that marker-centering angular control keeps the camera approximately facing the marker center.

### Moving Average

The controller smooths only the wall-relative translational estimates:

```text
estimated_x = average(last N raw_estimated_x samples)
estimated_z = average(last N raw_estimated_z samples)
N = position_average_window_size
```

`raw_estimated_x` and `raw_estimated_z` are stored in independent fixed-length buffers. If fewer than `N` samples are available, the controller averages the samples currently held in the buffer.

The moving average is applied before:

- PRE_DOCKING forward/backward control
- PRE_DOCKING lateral control
- PRE_DOCKING to FINAL_DOCKING transition judgment
- FINAL_DOCKING forward control
- FINAL_DOCKING lateral control
- FINAL_DOCKING stop judgment

The image-centering yaw control uses the latest `normalized_center_error` directly. It is not smoothed by this moving average.

If ArUco detection times out, the controller publishes zero velocity and clears the `estimated_x` / `estimated_z` moving-average buffers. When detection resumes, the average restarts from new samples only.

With `position_average_window_size: 1`, the average contains only the latest sample, so the behavior is equivalent to the previous unsmoothed position estimate.

### Angular Control

The angular control target is the marker image-center error. The robot rotates only when the marker center is outside a small deadband.

```text
if abs(center_error) < center_deadband:
    angular_error = 0.0
    angular_enabled = false
else:
    angular_error = -center_error
    angular_enabled = true
```

Meaning:

```text
abs(normalized_center_error) < 0.05:
  marker is close enough to the image center, so stop angular.z

abs(normalized_center_error) >= 0.05:
  rotate to bring the marker center back toward the image center
```

### PRE_DOCKING

The purpose of `PRE_DOCKING` is to move from the initial pose `P0` to the pre-docking pose `P1` while preparing both position and posture for docking.

The controller does not directly command wall-perpendicular yaw in `PRE_DOCKING`. Instead, it keeps the marker centered in the camera image and uses the moving-averaged wall-relative `estimated_x` and `estimated_z` for translational control.

```text
cmd.linear.x = kp_z * forward_error
cmd.linear.y = -kp_x * lateral_error

if angular_enabled:
    cmd.angular.z = kp_center * angular_error
else:
    cmd.angular.z = 0.0
```

### FINAL_DOCKING

The purpose of `FINAL_DOCKING` is to move from the pre-docking pose `P1` to the final docking pose `P2`.

This state continues proportional forward, lateral, and marker image-centering angular correction until the final stop distance is reached.

```text
if estimated_z <= docking_distance:
    cmd.linear.x = 0.0
    cmd.linear.y = 0.0
    cmd.angular.z = 0.0
else:
    cmd.linear.x = kp_z * (estimated_z - docking_distance)
    cmd.linear.y = -kp_x * lateral_error
    if angular_enabled:
        cmd.angular.z = kp_center * angular_error
    else:
        cmd.angular.z = 0.0
```

There is no separate `DOCKED` state. After the final stop condition is reached, the controller keeps publishing zero velocity while staying in `FINAL_DOCKING`.

---

## State Transition Conditions

### PRE_DOCKING to FINAL_DOCKING

The controller switches from `PRE_DOCKING` to `FINAL_DOCKING` only when a recent ArUco detection is available and all alignment errors are inside tolerance:

```text
marker_visible == true
abs(estimated_z - target_z) < z_tolerance
abs(estimated_x - target_x) < x_tolerance
abs(normalized_center_error) < center_deadband
```

In the implementation, `marker_visible == true` means `/aruco/distance` has been received within `detection_timeout`.

`estimated_x` and `estimated_z` in this transition condition are the moving-averaged values.

### FINAL_DOCKING Stop Condition

Once the controller is in `FINAL_DOCKING`, it keeps moving forward while correcting lateral position and marker centering until `estimated_z` reaches the final stop distance:

```text
if estimated_z <= docking_distance:
    cmd.linear.x = 0.0
    cmd.linear.y = 0.0
    cmd.angular.z = 0.0
```

If ArUco detection times out during `FINAL_DOCKING`, the controller publishes zero velocity and returns to `PRE_DOCKING`.

The `estimated_z` used for the final stop condition is the moving-averaged value.

---

## Behavior

```text
PRE_DOCKING:
  estimated_z > target_z    → robot moves forward
  estimated_z < target_z    → robot moves backward
  estimated_x > target_x    → robot moves in negative lateral direction
  estimated_x < target_x    → robot moves in positive lateral direction
  normalized_center_error > center_deadband  → robot rotates to move the marker toward image center
  normalized_center_error < -center_deadband → robot rotates to move the marker toward image center
  abs(normalized_center_error) < center_deadband → angular.z stops

FINAL_DOCKING:
  estimated_z > docking_distance  → robot moves forward with proportional control
  estimated_x is corrected with proportional lateral control
  normalized_center_error is corrected with proportional angular control
  estimated_z <= docking_distance → robot stops
```

---

## Safety Mechanisms

### 1. Tolerance Conditions

```text
if abs(forward_error) < z_tolerance:
    cmd.linear.x = 0.0
if abs(lateral_error) < x_tolerance:
    cmd.linear.y = 0.0
if angular_enabled == false:
    cmd.angular.z = 0.0
elif abs(center_error) < center_deadband:
    cmd.angular.z = 0.0
```

### 2. Detection Timeout

If no ArUco message is received for a specified duration:

```text
cmd.linear.x = 0.0
cmd.linear.y = 0.0
cmd.angular.z = 0.0
```

If detection is lost during `FINAL_DOCKING`, the controller returns to `PRE_DOCKING`.

The moving-average buffers for `estimated_x` and `estimated_z` are cleared when detection times out, so stale position estimates are not reused after detection resumes.

### 3. Velocity Limitation

```text
cmd.linear.x = clamp(cmd.linear.x,
                     -max_forward_speed,
                     +max_forward_speed)
cmd.linear.y = clamp(cmd.linear.y,
                     -max_lateral_speed,
                     +max_lateral_speed)
cmd.angular.z = clamp(cmd.angular.z,
                      -max_angular_speed,
                      +max_angular_speed)
```

### 4. Minimum Translational Speed

If the robot is outside `z_tolerance`, the command keeps at least `min_forward_speed`. If the robot is outside `x_tolerance`, the command keeps at least `min_lateral_speed`. Both preserve the command direction.

Marker-centering control does not use direct minimum angular speed in `PRE_DOCKING`.

```text
if 0.0 < abs(cmd.linear.x) < min_forward_speed:
    cmd.linear.x = sign(cmd.linear.x) * min_forward_speed
if 0.0 < abs(cmd.linear.y) < min_lateral_speed:
    cmd.linear.y = sign(cmd.linear.y) * min_lateral_speed
```

---

## Parameters

```yaml
target_z: 1.3              # pre-docking target distance [m]
kp_z: 2.0                  # forward/backward proportional gain
min_forward_speed: 0.30    # minimum moving speed outside tolerance [m/s]
max_forward_speed: 0.95    # maximum forward/backward speed [m/s]
z_tolerance: 0.01          # acceptable pre-docking distance error [m]
docking_distance: 1.0      # final stop distance [m]
target_x: 0.0              # target lateral offset [m]
kp_x: 0.4                  # lateral proportional gain
min_lateral_speed: 0.30    # minimum lateral speed outside tolerance [m/s]
max_lateral_speed: 0.95    # maximum lateral speed [m/s]
x_tolerance: 0.01          # acceptable lateral error [m]
target_yaw: 0.0            # target marker yaw [rad], used for logging/evaluation
kp_center: 0.3             # marker image-centering proportional gain
center_deadband: 0.05      # normalized center-error deadband
max_angular_speed: 0.5     # maximum yaw speed [rad/s]
position_average_window_size: 1  # moving-average window for estimated_x/z samples
detection_timeout: 0.5     # timeout [sec]
control_rate: 20.0         # control loop frequency [Hz]
```

`position_average_window_size` must be greater than or equal to `1`.

At the default `control_rate` of 20 Hz, one sample corresponds to about 0.05 seconds. A window size of 5 covers about 0.25 seconds of samples, with an approximate effective delay of `(5 - 1) / 2 / 20 = 0.10` seconds for a simple moving average.

`aruco_docking.launch.py` should expose `position_average_window_size` as a launch argument and pass it to `aruco_distance_controller`.

---

## System Integration

```text
aruco_distance_publisher/aruco_distance_publisher
  publishes /aruco/distance
        ↓
aruco_dist_ctrl/aruco_distance_controller
  publishes /rov_cmd_vel
        ↓
locomotion_core/rover_velocity
  publishes rov/motors
        ↓
locomotion_core/cmd_roboteq
  drives Roboteq motor controllers
```

---

## Usage

```bash
colcon build --packages-select aruco_interfaces aruco_distance_publisher aruco_dist_ctrl locomotion_core aruco_docking_bringup
source install/setup.bash

ros2 run aruco_distance_publisher aruco_distance_publisher
ros2 run aruco_dist_ctrl aruco_distance_controller
ros2 launch aruco_docking_bringup aruco_docking.launch.py
```

## Logging ArUco and Command Velocity

Run this logger during experiments to save `/aruco/distance`, the OptiTrack pose, and `/rov_cmd_vel` into one time-aligned CSV file.

```bash
ros2 run aruco_dist_ctrl aruco_cmd_logger --ros-args \
  -p output_dir:=/tmp/aruco_docking_logs \
  -p log_rate:=20.0 \
  -p optitrack_pose_topic:=/vrpn_mocap/RigidBody_1/pose \
  -p target_yaw:=0.0 \
  -p position_average_window_size:=1 \
  -p detection_timeout:=0.5
```

The logger writes:

```text
aruco_cmd_log_<timestamp>.csv
```

Main columns:

```text
elapsed_sec, aruco_z, aruco_yaw, aruco_normalized_center_error,
aruco_z_cos_yaw, aruco_z_sin_yaw,
raw_estimated_z, raw_estimated_x,
estimated_z_average, estimated_x_average,
optitrack_x, optitrack_y, optitrack_z,
cmd_linear_x, cmd_linear_y, cmd_angular_z
```

`raw_estimated_z` and `raw_estimated_x` use the same wall-relative transform as the controller:

```text
theta = wrap_pi(aruco_yaw - target_yaw)
raw_estimated_x = aruco_z * sin(theta)
raw_estimated_z = aruco_z * cos(theta)
```

`estimated_z_average` and `estimated_x_average` are the logger-side moving averages of those raw estimates. To match the controller behavior, run the logger with the same `target_yaw`, `position_average_window_size`, and `detection_timeout` values as `aruco_distance_controller`.

---

## Scope (v0.4)

* Distance-based forward/backward control
* Lateral alignment control
* Marker image-centering angular control
* Moving average for wall-relative `estimated_x` and `estimated_z`
* Two-state docking sequence (`PRE_DOCKING` / `FINAL_DOCKING`)
* Final docking using proportional `linear.x`, `linear.y`, and `angular.z` correction
* Safety stop (tolerance & timeout)

---

## Out of Scope (Future Work)

* PID control
* Multi-marker handling
* Obstacle avoidance
* Post-docking state management

---

## Next Steps

* Validate forward/backward motion
* Validate lateral motion direction
* Validate marker image-centering direction
* Check whether logged `aruco_yaw` naturally approaches the target posture
* Tune `center_deadband`
* Tune `docking_distance`
* Tune `position_average_window_size`
