# ArUco Distance Controller (v0.3 Switching Docking Control)

## Overview

This node controls the robot's forward/backward, lateral, and yaw motion based on ArUco marker pose information.

The robot is equipped with omni wheels, so position control and posture control can be handled independently through `linear.x`, `linear.y`, and `angular.z`.

The objective is to autonomously dock to a docking station by first moving from the initial pose `P0` to a pre-docking pose `P1`, and then moving from `P1` to the final docking pose `P2`.

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

The controller uses `z` to compute forward/backward motion, `x` to compute lateral motion, and both `theta` and `yaw` to compute angular motion.

- `theta`: used to keep the marker in view, especially at long range.
- `yaw`: used to align the robot nearly perpendicular to the docking station, especially at short range.
- `normalized_center_error`: used to decide when yaw alignment can start without losing marker visibility.

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

### Error Definition

```text
forward_error = aruco_z - target_z
lateral_error = aruco_x - target_x
bearing_error = wrap_pi(-aruco_theta)
perpendicular_error = wrap_pi(aruco_yaw - target_yaw)
```

`aruco_theta` is positive when the marker center is to the camera-right side. The controller uses `-aruco_theta` so that the yaw command turns the robot toward the marker center.

### Angular Error Switching

The angular control target switches according to distance and marker visibility margin.

```text
if abs(normalized_center_error) > yaw_align_center_error_threshold:
    angular_error = 0.0
    angular_enabled = false
elif aruco_z <= angular_switch_distance:
    angular_error = perpendicular_error
    angular_enabled = true
else:
    angular_error = bearing_error
    angular_enabled = true
```

Meaning:

```text
abs(normalized_center_error) > 0.4:
  stop angular.z at any distance to avoid losing the marker

abs(normalized_center_error) <= 0.4 and z > 2.0 m:
  prioritize marker visibility using theta

abs(normalized_center_error) <= 0.4 and z <= 2.0 m:
  use yaw so the robot becomes perpendicular to the docking station
```

### PRE_DOCKING

The purpose of `PRE_DOCKING` is to move from the initial pose `P0` to the pre-docking pose `P1` while preparing both position and posture for docking.

When the marker is near the image edge, angular motion is stopped and only translational control continues. This prevents the robot from rotating the marker out of view.

When the marker is near the image center, the controller uses `theta` at long range to keep the marker visible and switches to `yaw` near the docking station to align perpendicular to the wall.

```text
position_error = abs(forward_error) + abs(lateral_error)
weight = yaw_weight_min + (1.0 - yaw_weight_min) / (1.0 + yaw_distance_gain * position_error)

cmd.linear.x = kp_z * forward_error
cmd.linear.y = -kp_x * lateral_error

if angular_enabled:
    cmd.angular.z = kp_yaw * angular_error * weight
else:
    cmd.angular.z = 0.0
```

`weight` approaches `yaw_weight_min` when the robot is far from the target, and approaches `1.0` near the target. This suppresses unnecessary rotation at long range while keeping angular control active.

### FINAL_DOCKING

The purpose of `FINAL_DOCKING` is to move from the pre-docking pose `P1` to the final docking pose `P2`.

This state uses a simple straight final approach from an already aligned pose.

```text
if aruco_z <= docking_distance:
    cmd.linear.x = 0.0
    cmd.linear.y = 0.0
    cmd.angular.z = 0.0
else:
    cmd.linear.x = final_forward_speed
    cmd.linear.y = 0.0
    cmd.angular.z = 0.0
```

There is no separate `DOCKED` state. After the final stop condition is reached, the controller keeps publishing zero velocity while staying in `FINAL_DOCKING`.

---

## State Transition Conditions

### PRE_DOCKING to FINAL_DOCKING

The controller switches from `PRE_DOCKING` to `FINAL_DOCKING` only when a recent ArUco detection is available and all alignment errors are inside tolerance:

```text
marker_visible == true
abs(aruco_z - target_z) < z_tolerance
abs(aruco_x - target_x) < x_tolerance
angular_enabled == true
abs(angular_error) < yaw_tolerance
```

In the implementation, `marker_visible == true` means `/aruco/distance` has been received within `detection_timeout`.

### FINAL_DOCKING Stop Condition

Once the controller is in `FINAL_DOCKING`, it keeps moving straight until the marker reaches the final stop distance:

```text
if aruco_z <= docking_distance:
    cmd.linear.x = 0.0
    cmd.linear.y = 0.0
    cmd.angular.z = 0.0
```

If ArUco detection times out during `FINAL_DOCKING`, the controller publishes zero velocity and returns to `PRE_DOCKING`.

---

## Behavior

```text
PRE_DOCKING:
  aruco_z > target_z    → robot moves forward
  aruco_z < target_z    → robot moves backward
  aruco_x > target_x    → robot moves in negative lateral direction
  aruco_x < target_x    → robot moves in positive lateral direction
  angular error switches according to distance and image-center error
  angular.z stops in the intermediate range when the marker is near the image edge
  angular command is corrected with position-error-based weighting

FINAL_DOCKING:
  aruco_z > docking_distance  → robot moves forward at final_forward_speed
  aruco_z <= docking_distance → robot stops
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
elif abs(angular_error) < yaw_tolerance:
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

Yaw control does not use direct minimum angular speed in `PRE_DOCKING`. Instead, it uses `yaw_weight_min` so angular control is weakened far from the pre-docking target but does not vanish.

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
kp_z: 1.0                  # forward/backward proportional gain
min_forward_speed: 0.3     # minimum moving speed outside tolerance [m/s]
max_forward_speed: 0.95    # maximum forward/backward speed [m/s]
z_tolerance: 0.01          # acceptable pre-docking distance error [m]
docking_distance: 1.0      # final stop distance [m]
final_forward_speed: 0.4   # fixed forward speed in FINAL_DOCKING [m/s]
target_x: 0.0              # target lateral offset [m]
kp_x: 1.0                  # lateral proportional gain
min_lateral_speed: 0.3     # minimum lateral speed outside tolerance [m/s]
max_lateral_speed: 0.95    # maximum lateral speed [m/s]
x_tolerance: 0.01          # acceptable lateral error [m]
target_yaw: 0.0            # target marker yaw [rad]
kp_yaw: 0.3                # yaw proportional gain
min_angular_speed: 0.1     # retained for compatibility; weighted yaw does not use direct minimum speed
max_angular_speed: 0.5     # maximum yaw speed [rad/s]
yaw_tolerance: 0.01        # acceptable yaw error [rad]
yaw_distance_gain: 1.0     # position-error gain for yaw weighting
yaw_weight_min: 0.2        # minimum yaw weight far from target
angular_switch_distance: 2.0       # yaw alignment may start below this distance [m]
yaw_align_center_error_threshold: 0.4  # yaw alignment may start only when abs(normalized_center_error) is below this value
detection_timeout: 0.5     # timeout [sec]
control_rate: 20.0         # control loop frequency [Hz]
```

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

Run this logger during experiments to save `/aruco/distance` and `/rov_cmd_vel` into one time-aligned CSV file.

```bash
ros2 run aruco_dist_ctrl aruco_cmd_logger --ros-args \
  -p output_dir:=/tmp/aruco_docking_logs \
  -p log_rate:=20.0
```

The logger writes:

```text
aruco_cmd_log_<timestamp>.csv
```

Main columns:

```text
elapsed_sec, aruco_x, aruco_z, aruco_theta, aruco_yaw,
aruco_center_u, aruco_center_v, aruco_normalized_center_error,
cmd_linear_x, cmd_linear_y, cmd_angular_z
```

---

## Scope (v0.3)

* Distance-based forward/backward control
* Lateral alignment control
* Distance- and image-center-dependent angular control switching between marker-center bearing and marker yaw
* Two-state docking sequence (`PRE_DOCKING` / `FINAL_DOCKING`)
* Straight final docking using fixed `linear.x`
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
* Validate theta-based yaw motion direction at long range
* Validate yaw-based perpendicular alignment when marker is near image center
* Tune `angular_switch_distance`
* Tune `yaw_align_center_error_threshold`
* Tune `yaw_distance_gain` and `yaw_weight_min`
* Tune `docking_distance` and `final_forward_speed`
