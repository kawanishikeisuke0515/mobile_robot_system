# ArUco Distance Controller (v0.1)

## Overview

This node controls the robot's forward/backward, lateral, and yaw motion based on the distance to an ArUco marker.

The objective is to move the robot to a specified target distance, lateral offset, and marker yaw using proportional (P) control.

---

## Node Name

```
aruco_distance_controller
```

## Package

```
aruco_dist_ctrl
```

---

## Subscribed Topic

```
/aruco/distance
type: aruco_interfaces/msg/ArucoDistance
```

### Expected Field

```
x : marker translation right from the camera in meters
z : marker translation forward from the camera in meters
yaw : marker orientation angle from rvec in radians
```

### ArUcoDistance axis definition

- `x`: right direction in camera frame
- `y`: down direction in camera frame
- `z`: forward direction in camera frame

The controller uses `z` to compute forward/backward motion, `x` to compute lateral motion, and `yaw` to compute angular motion.

---

## Published Topic

```
/rov_cmd_vel
type: geometry_msgs/msg/Twist
```

### Used Field

```
linear.x : forward/backward velocity [m/s]
linear.y : lateral velocity [m/s]
angular.z : yaw velocity [rad/s]
```

This topic is consumed by `locomotion_core/rover_velocity`.

---

## Control Strategy

### Error Definition

```
error_z = aruco_z - target_z
error_x = aruco_x - target_x
error_yaw = wrap_pi(aruco_yaw - target_yaw)
```

### Control Law (P Control)

```
cmd.linear.x = Kp_z * error_z
cmd.linear.y = -Kp_x * error_x
cmd.angular.z = Kp_yaw * error_yaw
```

---

## Behavior

```
aruco_z > target_z  → robot moves forward
aruco_z < target_z  → robot moves backward
aruco_x > target_x  → robot moves in negative lateral direction
aruco_x < target_x  → robot moves in positive lateral direction
aruco_yaw > target_yaw  → robot rotates in positive yaw direction
aruco_yaw < target_yaw  → robot rotates in negative yaw direction
```

---

## Safety Mechanisms

### 1. Target Reached Condition

```
if abs(error_z) < z_tolerance:
    cmd.linear.x = 0.0
if abs(error_x) < x_tolerance:
    cmd.linear.y = 0.0
if abs(error_yaw) < yaw_tolerance:
    cmd.angular.z = 0.0
```

---

### 2. Detection Timeout

If no ArUco message is received for a specified duration:

```
cmd.linear.x = 0.0
cmd.linear.y = 0.0
cmd.angular.z = 0.0
```

---

### 3. Velocity Limitation

```
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

### 4. Minimum Moving Speed

If the robot is outside `z_tolerance`, the command keeps at least
`min_forward_speed`. If the robot is outside `x_tolerance`, the command
keeps at least `min_lateral_speed`. If the robot is outside `yaw_tolerance`,
the command keeps at least `min_angular_speed`. All preserve the command direction.

```
if 0.0 < abs(cmd.linear.x) < min_forward_speed:
    cmd.linear.x = sign(cmd.linear.x) * min_forward_speed
if 0.0 < abs(cmd.linear.y) < min_lateral_speed:
    cmd.linear.y = sign(cmd.linear.y) * min_lateral_speed
if 0.0 < abs(cmd.angular.z) < min_angular_speed:
    cmd.angular.z = sign(cmd.angular.z) * min_angular_speed
```

---

## Parameters

```yaml
target_z: 1.0              # target distance [m]
kp_z: 1.0                  # proportional gain
min_forward_speed: 0.3     # minimum moving speed outside tolerance [m/s]
max_forward_speed: 0.95    # maximum speed [m/s]
z_tolerance: 0.01          # acceptable error [m]
target_x: 0.0              # target lateral offset [m]
kp_x: 1.0                  # lateral proportional gain
min_lateral_speed: 0.3     # minimum lateral speed outside tolerance [m/s]
max_lateral_speed: 0.95    # maximum lateral speed [m/s]
x_tolerance: 0.03          # acceptable lateral error [m]
target_yaw: 0.0            # target marker yaw [rad]
kp_yaw: 1.0                # yaw proportional gain
min_angular_speed: 0.1     # minimum yaw speed outside tolerance [rad/s]
max_angular_speed: 0.5     # maximum yaw speed [rad/s]
yaw_tolerance: 0.05        # acceptable yaw error [rad]
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

---

## Scope (v0.1)

* Distance-based forward/backward control
* Lateral alignment control
* Yaw alignment control
* Proportional (P) control
* Safety stop (tolerance & timeout)

---

## Out of Scope (Future Work)

* PID control
* Multi-marker handling
* Obstacle avoidance
* Full docking sequence

---

## Next Steps

* Validate forward/backward motion
* Tune `kp_z` and speed limits
* Validate lateral motion direction
* Validate yaw motion direction
* Introduce state machine (TRACKING / REACHED / LOST)
