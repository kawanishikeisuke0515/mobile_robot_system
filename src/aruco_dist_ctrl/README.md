# ArUco Distance-Gated Docking Controller

## Overview

This package contains the ArUco docking controller and experiment logger.

`aruco_distance_controller` subscribes to `/aruco/distance` and publishes
`/rov_cmd_vel`. The controller follows a distance-gated state machine instead
of commanding forward, lateral, and yaw motion at full strength at the same
time.

The controller is intended for the phase where the robot is already close
enough for the camera to detect the ArUco marker.

## Topics

Subscribed:

```text
/aruco/distance
type: aruco_interfaces/msg/ArucoDistance
```

Published:

```text
/rov_cmd_vel
type: geometry_msgs/msg/Twist

/aruco_docking/state
type: std_msgs/msg/String
```

Used `Twist` fields:

```text
linear.x   forward/backward command
linear.y   lateral command
angular.z  yaw command
```

## Message Fields

```text
aruco_x        = msg.x
aruco_y        = msg.y
aruco_z        = msg.z, forward-axis distance
aruco_distance = msg.distance, sqrt(x^2 + y^2 + z^2)
aruco_theta    = msg.theta, atan2(x, z)
aruco_yaw      = msg.yaw
```

State transitions use `aruco_distance` for approach gating. Collision safety
uses `aruco_z`.

## State Machine

Detailed design:

```text
docs/docking_state_machine_spec.md
docs/docking_state_machine_spec_ja.md
```

Main sequence:

```text
WAIT_FOR_MARKER
  -> FAR_GUIDED_APPROACH
  -> NEAR_ALIGN
  -> FINAL_APPROACH
  -> DOCKED
```

State behavior:

```text
WAIT_FOR_MARKER:
  stop until a recent marker detection is available

FAR_GUIDED_APPROACH:
  linear.x = far_approach_speed
  angular.z = weak theta_x centering
  linear.y = 0

NEAR_ALIGN:
  linear.x = 0
  linear.y = lateral correction
  angular.z = marker yaw correction

FINAL_APPROACH:
  linear.x = final_approach_speed
  linear.y = 0
  angular.z = 0

HOLD:
  stop for hold_duration, then resume a valid state or wait for marker

DOCKED:
  stop
```

## Docked Condition

The robot enters `DOCKED` only when all final pose errors are within tolerance:

```text
abs(aruco_z - target_z) < z_tolerance
abs(aruco_x - target_x) < x_tolerance
abs(wrap_pi(aruco_yaw - target_yaw)) < yaw_tolerance
```

The controller never commands forward motion when:

```text
aruco_z <= minimum_safe_z
```

## Parameters

```yaml
target_distance: 1.0
align_distance: 1.2
minimum_safe_z: 1.0
align_hysteresis: 0.10

target_x: 0.0
target_z: 1.0
target_yaw: 0.0

x_tolerance: 0.05
z_tolerance: 0.03
yaw_tolerance: 0.06

final_x_realign_threshold: 0.08
final_yaw_realign_threshold: 0.10

theta_x_slow_limit: 0.15
theta_x_stop_limit: 0.25
theta_y_slow_limit: 0.15
theta_y_stop_limit: 0.25

kp_lateral: 0.4
kp_yaw: 0.6
kp_far_center: 0.3

far_approach_speed: 0.3
reduced_far_approach_speed: 0.3
final_approach_speed: 0.3
min_far_center_speed: 0.3
max_far_center_speed: 0.95
min_lateral_align_speed: 0.3
max_lateral_align_speed: 0.95
min_yaw_align_speed: 0.3
max_yaw_align_speed: 0.95

detection_timeout: 0.5
hold_duration: 0.8
control_rate: 20.0
```

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

## Usage

```bash
colcon build --packages-select aruco_interfaces aruco_distance_publisher aruco_dist_ctrl locomotion_core aruco_docking_bringup
source install/setup.bash

ros2 run aruco_distance_publisher aruco_distance_publisher
ros2 run aruco_dist_ctrl aruco_distance_controller
ros2 launch aruco_docking_bringup aruco_docking.launch.py
```

Watch the current docking state in real time:

```bash
ros2 topic echo /aruco_docking/state
```

## Logging ArUco and Command Velocity

Run this logger during experiments to save `/aruco/distance` and `/rov_cmd_vel`
into one time-aligned CSV file.

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
elapsed_sec, aruco_age_sec, cmd_age_sec,
aruco_id, aruco_x, aruco_y, aruco_z, aruco_distance, aruco_theta, aruco_yaw,
cmd_linear_x, cmd_linear_y, cmd_angular_z
```
