# Generic Motion Control Specification

This document outlines the motion control specifications for the rover.

## Overview
- Velocity control via Twist messages
- Wheel drive using Roboteq controllers
- Kinematic calculations for rover drive

## Topics
- `/rov_cmd_vel`: Input `geometry_msgs/msg/Twist` commands
- `rov/motors`: Output `std_msgs/msg/Float32MultiArray` motor commands
- `deadman`: Output `std_msgs/msg/Bool` deadman signal

## Used Twist Fields
- `linear.x`: lateral velocity command (robot body frame, positive to the right)
- `linear.y`: forward/backward velocity command (robot body frame, positive forward)
- `angular.z`: yaw velocity command (positive counter-clockwise)

## Coordinate frame
- Twist values are interpreted in the rover body frame.
- `linear.x` is side-to-side motion, `linear.y` is forward/backward motion.
- `linear.z` is not used by the current locomotion controller.

## Nodes
- `rover_velocity`: Converts Twist to RPM
- `cmd_roboteq`: Drives motors via serial

## Upstream Integration
- `aruco_dist_ctrl/aruco_distance_controller` publishes `/rov_cmd_vel`
- Teleop can also publish `/rov_cmd_vel` when manually driving

## Usage
1. Launch wheel control: `ros2 launch locomotion_core wheel_control.launch.py`
2. Run teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rov_cmd_vel`
