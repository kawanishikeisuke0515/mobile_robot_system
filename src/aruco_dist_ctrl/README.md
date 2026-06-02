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
```

### ArUcoDistance Axis Definition

- `x`: right direction in camera frame
- `y`: down direction in camera frame
- `z`: forward direction in camera frame

The controller uses `z` to compute forward/backward motion, `x` to compute lateral motion, and both `theta` and `yaw` to compute angular motion.

- `theta`: used to keep the marker in view, especially at long range.
- `yaw`: used to align the robot nearly perpendicular to the docking station, especially at short range.

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
bearing_error = wrap_pi(aruco_theta)
perpendicular_error = wrap_pi(aruco_yaw - target_yaw)
```

### Angular Error Switching

The angular control target switches according to distance.

```text
if aruco_z > angular_switch_distance:
    angular_error = bearing_error
else:
    angular_error = perpendicular_error
```

Meaning:

```text
z > 2.0 m  : prioritize marker visibility using theta
z <= 2.0 m : prioritize perpendicular docking posture using yaw
```

### PRE_DOCKING

The purpose of `PRE_DOCKING` is to move from the initial pose `P0` to the pre-docking pose `P1` while preparing both position and posture for docking.

At long range, the controller prioritizes keeping the marker in the camera field of view. Near the docking station, the controller prioritizes becoming perpendicular to the docking station.

```text
position_error = abs(forward_error) + abs(lateral_error)
weight = yaw_weight_min + (1.0 - yaw_weight_min) / (1.0 + yaw_distance_gain * position_error)

cmd.linear.x = kp_z * forward_error
cmd.linear.y = -kp_x * lateral_error
cmd.angular.z = kp_yaw * angular_error * weight
```

`weight` approaches `yaw_weight_min` when the robot is far from the target, and approaches `1.0` near the target. This suppresses unnecessary rotation at long range while keeping angular control active.

### FINAL_DOCKING

The purpose of `FINAL_DOCKING` is to move from the pre-docking pose `P1` to the final docking pose `P2`.

This state is not open-loop. The robot continues to use all three control axes:

- `linear.x`: forward motion toward the docking distance
- `linear.y`: lateral error correction
- `angular.z`: posture error correction and disturbance rejection

```text
if aruco_z <= docking_distance:
    cmd.linear.x = 0.0
    cmd.linear.y = 0.0
    cmd.angular.z = 0.0
else:
    final_forward_error = aruco_z - docking_distance
    cmd.linear.x = clamp(kp_final_z * final_forward_error,
                         0.0,
                         final_forward_speed)
    cmd.linear.y = -kp_x * lateral_error
    cmd.angular.z = kp_yaw * angular_error
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
abs(angular_error) < yaw_tolerance
```

In the implementation, `marker_visible == true` means `/aruco/distance` has been received within `detection_timeout`.

### FINAL_DOCKING Stop Condition

Once the controller is in `FINAL_DOCKING`, it keeps moving with feedback correction until the marker reaches the final stop distance:

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
  angular error switches between theta and yaw according to distance
  angular command is corrected with position-error-based weighting

FINAL_DOCKING:
  aruco_z > docking_distance  → robot moves forward with feedback correction
  lateral and angular errors  → robot keeps correcting with linear.y and angular.z
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
if abs(angular_error) < yaw_tolerance:
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
target_z: 1.5              # pre-docking target distance [m]
kp_z: 1.0                  # forward/backward proportional gain
min_forward_speed: 0.3     # minimum moving speed outside tolerance [m/s]
max_forward_speed: 0.95    # maximum forward/backward speed [m/s]
z_tolerance: 0.01          # acceptable pre-docking distance error [m]
docking_distance: 1.0      # final stop distance [m]
kp_final_z: 0.4            # final docking forward proportional gain
final_forward_speed: 0.4   # maximum forward speed in FINAL_DOCKING [m/s]
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
angular_switch_distance: 2.0  # above this z use theta; below or equal use yaw [m]
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
cmd_linear_x, cmd_linear_y, cmd_angular_z
```

---

## Scope (v0.3)

* Distance-based forward/backward control
* Lateral alignment control
* Distance-dependent angular control switching between marker-center bearing and marker yaw
* Two-state docking sequence (`PRE_DOCKING` / `FINAL_DOCKING`)
* Closed-loop final docking using `linear.x`, `linear.y`, and `angular.z`
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
* Validate yaw-based perpendicular alignment at short range
* Tune `angular_switch_distance`
* Tune `yaw_distance_gain` and `yaw_weight_min`
* Tune `docking_distance` and `final_forward_speed`
