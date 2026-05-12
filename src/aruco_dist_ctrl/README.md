# ArUco Distance Controller (v0.1)

## Overview

This node controls the robot's forward/backward motion based on the distance to an ArUco marker.

The objective is to move the robot to a specified target distance using proportional (P) control.

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
z : distance to the ArUco marker [m]
```

---

## Published Topic

```
/rov_cmd_vel
type: geometry_msgs/msg/Twist
```

### Used Field

```
linear.y : forward/backward velocity [m/s]
```

This topic is consumed by `locomotion_core/rover_velocity`.

---

## Control Strategy

### Error Definition

```
error_z = aruco_z - target_z
```

### Control Law (P Control)

```
cmd.linear.y = Kp_z * error_z
```

---

## Behavior

```
aruco_z > target_z  → robot moves forward
aruco_z < target_z  → robot moves backward
```

---

## Safety Mechanisms

### 1. Target Reached Condition

```
if abs(error_z) < z_tolerance:
    cmd.linear.y = 0.0
```

---

### 2. Detection Timeout

If no ArUco message is received for a specified duration:

```
cmd.linear.y = 0.0
```

---

### 3. Velocity Limitation

```
cmd.linear.y = clamp(cmd.linear.y,
                     -max_forward_speed,
                     +max_forward_speed)
```

### 4. Minimum Moving Speed

If the robot is outside `z_tolerance`, the command keeps at least
`min_forward_speed` while preserving the direction.

```
if 0.0 < abs(cmd.linear.y) < min_forward_speed:
    cmd.linear.y = sign(cmd.linear.y) * min_forward_speed
```

---

## Parameters

```yaml
target_z: 0.5               # target distance [m]
kp_z: 0.4                  # proportional gain
min_forward_speed: 0.03    # minimum moving speed outside tolerance [m/s]
max_forward_speed: 0.15    # maximum speed [m/s]
z_tolerance: 0.03          # acceptable error [m]
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
* Proportional (P) control
* Safety stop (tolerance & timeout)

---

## Out of Scope (Future Work)

* Lateral (x-axis) alignment
* Angular control
* PID control
* Multi-marker handling
* Obstacle avoidance
* Full docking sequence

---

## Next Steps

* Validate forward/backward motion
* Tune `kp_z` and speed limits
* Add lateral control (`linear.x`)
* Introduce state machine (TRACKING / REACHED / LOST)
