# ArUco Docking Simulation Interface Contract

Version: 0.1 draft
Date: 2026-06-29
Status: draft

## 1. Purpose

This document defines the ROS 2 interface contract between the ArUco docking controller and any simulation environment.

The same contract should be usable by:

- a lightweight ROS 2 closed-loop simulator
- NVIDIA Omniverse / Isaac Sim
- the real robot system

The controller-facing interface must remain stable so that simulation environments can be replaced without changing `aruco_dist_ctrl/aruco_distance_controller`.

## 2. Boundary

```text
Simulation side
  publishes:   /aruco/distance
  subscribes:  /rov_cmd_vel

Controller side
  subscribes:  /aruco/distance
  publishes:   /rov_cmd_vel
```

Optional interfaces such as `/tf`, `/odom`, `/joint_states`, image topics, and `/clock` may be added later, but they are not required by the current controller.

## 3. Required Topics

### 3.1 `/aruco/distance`

| Item | Value |
| --- | --- |
| Direction | simulation to controller |
| Type | `aruco_interfaces/msg/ArucoDistance` |
| Purpose | ArUco marker observation used by the docking controller |

Required fields for the current controller:

| Field | Unit | Meaning |
| --- | --- | --- |
| `z` | m | Marker forward distance from the camera |
| `yaw` | rad | Marker or wall yaw relative to controller target yaw |
| `normalized_center_error` | normalized | Horizontal marker-center error in the image |

Recommended fields for compatibility:

| Field | Unit | Meaning |
| --- | --- | --- |
| `id` | none | Marker ID |
| `x` | m | Marker lateral position in camera frame |
| `y` | m | Marker vertical position in camera frame |
| `distance` | m | Euclidean distance to marker |
| `theta` | rad | Bearing angle to marker center, typically `atan2(x, z)` |
| `center_u` | px | Marker center image u coordinate |
| `center_v` | px | Marker center image v coordinate |

### 3.2 `/rov_cmd_vel`

| Item | Value |
| --- | --- |
| Direction | controller to simulation |
| Type | `geometry_msgs/msg/Twist` |
| Purpose | Body-frame rover velocity command |

Used fields:

| Field | Unit | Meaning |
| --- | --- | --- |
| `linear.x` | m/s | Forward/backward velocity command |
| `linear.y` | m/s | Lateral velocity command |
| `angular.z` | rad/s | Yaw velocity command |

Unused fields should be ignored by simulation adapters.

## 4. Coordinate And Sign Conventions

### 4.1 Controller velocity frame

`/rov_cmd_vel` is interpreted in the rover body frame.

| Quantity | Positive Direction |
| --- | --- |
| `linear.x` | robot forward |
| `linear.y` | robot left or right according to locomotion implementation; adapter must verify against `locomotion_core` before physics integration |
| `angular.z` | counter-clockwise yaw by ROS convention |

Note: The existing `aruco_distance_controller` publishes `linear.y = -kp_x * lateral_error`. The simulator adapter must use the same lateral sign convention as the real rover path so that simulation behavior matches hardware behavior.

### 4.2 ArUco observation frame

`ArucoDistance` follows the camera-centric convention used by `aruco_distance_publisher`.

| Quantity | Unit | Positive Direction |
| --- | --- | --- |
| `x` | m | camera right |
| `y` | m | camera down |
| `z` | m | camera forward |
| `theta` | rad | positive when marker center is to camera right, calculated as `atan2(x, z)` |
| `normalized_center_error` | normalized | positive when marker center is to the right side of image center |

### 4.3 Image center convention

```text
center_u = image_width / 2   -> normalized_center_error = 0.0
center_u > image_width / 2   -> normalized_center_error > 0.0
center_u < image_width / 2   -> normalized_center_error < 0.0
```

The normalized value should be computed as:

```text
normalized_center_error = (center_u - image_width / 2) / (image_width / 2)
```

### 4.4 Yaw convention

The controller computes wall-relative position as:

```text
theta = wrap_pi(aruco_yaw - target_yaw)
estimated_x = aruco_z * sin(theta)
estimated_z = aruco_z * cos(theta)
```

Simulation adapters must publish `ArucoDistance.yaw` so that this calculation produces the intended wall-relative `estimated_x` and `estimated_z`.

For the first simulation implementation, `target_yaw = 0.0` and marker/wall yaw near `0.0` should represent an aligned docking approach.

## 5. Timing Contract

| Item | Requirement |
| --- | --- |
| Observation publish rate | Should be equal to or higher than controller `control_rate` when possible |
| Controller timeout | Must respect `detection_timeout` behavior by stopping `/aruco/distance` when simulating marker loss |
| Simulation time | If `/clock` is used, all ROS nodes in the simulation launch should use `use_sim_time=true` |
| Wall time mode | Lightweight simulation may use wall time and omit `/clock` |

## 6. Launch Contract

Simulation launch files must not start hardware drivers.

Allowed nodes:

- `aruco_docking_sim` lightweight simulator or Omniverse adapter
- `aruco_dist_ctrl/aruco_distance_controller`
- logging or visualization nodes
- optional `aruco_distance_publisher` when testing camera pipeline mode

Disallowed in simulation launch by default:

- `locomotion_core/cmd_roboteq`
- nodes that open real motor serial devices
- hardware-only camera nodes unless explicitly requested

## 7. Adapter Responsibilities

Any simulation adapter must:

1. Convert simulator world state to `aruco_interfaces/msg/ArucoDistance`.
2. Convert `/rov_cmd_vel` body-frame command to simulator motion input.
3. Document coordinate transforms between simulator world, robot body, camera, and marker frames.
4. Preserve the topic names and message types used by the controller.
5. Provide parameters for initial pose and marker pose.
6. Provide a way to reproduce detection loss.

## 8. Omniverse Notes

Omniverse / Isaac Sim integration should treat this document as the controller-facing contract.

The Omniverse side may use USD, PhysX, RTX cameras, ROS 2 bridge, or custom Python adapters internally. Those details should not leak into `aruco_distance_controller`.

Two integration paths are valid:

```text
Synthetic observation:
  Omniverse state -> adapter -> /aruco/distance

Camera pipeline:
  Omniverse camera -> image topic -> aruco_distance_publisher -> /aruco/distance
```

The synthetic observation path is recommended first because it validates controller behavior before introducing rendering and computer-vision uncertainty.

## 9. Open Questions

| ID | Question |
| --- | --- |
| OQ-001 | Confirm the real rover sign convention for `/rov_cmd_vel.linear.y`. |
| OQ-002 | Define the exact Omniverse world axis mapping to ROS `base_link` and camera frames. |
| OQ-003 | Decide whether lightweight simulation should publish `/tf` and `/odom` in the first implementation. |
| OQ-004 | Decide whether simulation logs should be CSV files, ROS bags, or both. |
| OQ-005 | Decide whether the first Omniverse path should be synthetic `/aruco/distance` or rendered camera images. |
