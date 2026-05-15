# ArUco Distance-Gated Docking Controller Spec

## Purpose

This document defines an experimental docking controller for the ArUco docking system.

The current scope assumes that the robot is already inside the camera range
where the ArUco marker can be detected. Long-range navigation, UWB navigation,
obstacle avoidance, marker search, and camera power management are future
system-level phases. This controller focuses only on reliable docking once
ArUco observations are available.

The current controller can command forward/backward, lateral, and yaw motion at
the same time. Real-robot tests showed that mixing yaw with translation can make
the robot unstable or freeze. This controller instead enables motion axes based
on distance to the marker:

```text
far from marker:   forward + weak marker-centering yaw
near marker:       allow lateral and yaw correction
final distance:    stop
```

The intent is to keep the first implementation simple. When the robot is far
away, it should not try to solve full pose alignment. Once it is close enough
that the marker will remain visible even when the robot rotates in place, it can
start using yaw and lateral motion.

## Scope

In scope:

```text
- Dock from any pose where the ArUco marker is already detectable.
- Keep the marker visible during docking.
- Avoid collision with the docking station.
- Avoid unstable yaw-plus-forward behavior.
- Stop safely on marker timeout or visibility risk.
```

Out of scope for this controller:

```text
- UWB navigation to the docking area.
- Obstacle detection and avoidance.
- Searching for a marker that is not visible.
- Turning the camera node on and off for power saving.
- Automatic recovery from inside the collision boundary.
```

The expected upstream system behavior is:

```text
UWB / navigation / obstacle phases bring the robot near the docking station.
ArUco marker becomes visible.
This docking controller takes over.
```

## Target Distances

```text
target_distance = 1.0 m
align_distance  = 1.2 m
minimum_safe_z = 1.0 m
```

Terminology:

```text
aruco_distance = msg.distance = sqrt(x^2 + y^2 + z^2)
aruco_z        = msg.z        = forward-axis distance
```

`target_distance` is the final Euclidean docking distance. `align_distance` is
the Euclidean distance at which the controller starts allowing yaw and lateral
correction. `minimum_safe_z` is the hard forward-axis safety boundary.

Do not set `minimum_safe_z` to `align_distance` for this docking sequence.
`align_distance` is the distance where the robot should stop normal far
approach and align its pose. `minimum_safe_z` is the distance where forward
motion becomes unsafe. If both are 1.2 m, the controller would not be allowed to
perform the final forward approach from 1.2 m to the 1.0 m docking distance.

The alignment and target thresholds use the ArUco message `distance` field:

```text
distance = sqrt(x^2 + y^2 + z^2)
```

The exact `align_distance` should be tuned from real-robot tests. It should be
close enough that the marker is reliably detected, but far enough that the robot
still has room to correct yaw and lateral offset before docking.

Collision safety uses `aruco_z`, not `aruco_distance`. The first implementation
should log both `aruco_distance` and `aruco_z`.

## Collision Guard

If the robot is already within the final docking distance, it may be close
enough to hit the docking station. The controller must never command forward
motion inside this boundary.

Hard rule:

```text
if aruco_z <= minimum_safe_z:
    linear.x = 0.0
```

The safety boundary uses `aruco_z`, not `aruco_distance`, because collision risk
is primarily along the robot's forward axis. Euclidean distance can be
misleading when lateral or vertical offset is large.

## Docked Condition

The robot should only enter `DOCKED` when forward distance, lateral offset, and
marker yaw are all inside tolerance.

```text
error_z = aruco_z - target_z
error_x = aruco_x - target_x
error_yaw = wrap_pi(aruco_yaw - target_yaw)

abs(error_z) < z_tolerance
and abs(error_x) < x_tolerance
and abs(error_yaw) < yaw_tolerance -> DOCKED
```

Do not use Euclidean `aruco_distance` alone for the final docked condition. It
can hide forward-axis or lateral alignment errors.

For the first implementation, entering this boundary should transition to
`DOCKED` if pose errors are acceptable, or `HOLD` if pose errors are not
acceptable:

```text
if docked condition is true:
    transition to DOCKED

if aruco_z <= minimum_safe_z
and pose errors are not acceptable:
    transition to HOLD
```

Backing up from this boundary should require an explicit recovery mode or manual
operation. Do not automatically drive backward in the first implementation.

Optional recovery:

```text
BACK_OUT_RECOVERY
```

This recovery can be added after basic docking is stable. It should only be
enabled when the robot is too close to the docking station and pose errors are
not acceptable. The recovery should move backward slowly, without yaw or lateral
motion, until the robot has enough distance to align again.

This recovery must not be enabled unless the upstream system can confirm that
backward motion is safe, or the test operator explicitly enables it.

## Coordinate Assumptions

The controller subscribes to:

```text
/aruco/distance
type: aruco_interfaces/msg/ArucoDistance
```

Relevant fields:

```text
x        marker translation right from the camera [m]
y        marker translation down from the camera [m]
z        marker translation forward from the camera [m]
distance Euclidean distance from the camera to the marker [m]
theta    horizontal marker bearing, atan2(x, z) [rad]
yaw      marker surface yaw estimated from rvec [rad]
```

Robot command mapping:

```text
Twist.linear.x   forward/backward motion
Twist.linear.y   lateral motion
Twist.angular.z  yaw rotation
```

## Visibility Guard

The marker must remain visible throughout docking. The controller computes:

```text
theta_x = atan2(aruco_x, aruco_z)
theta_y = atan2(aruco_y, aruco_z)
```

`theta_x` estimates horizontal image offset. `theta_y` estimates vertical image
offset. The rover cannot directly correct vertical error, but `theta_y` is still
useful as a warning that the marker may leave the camera view.

Suggested guard behavior:

```text
if abs(theta_x) > theta_x_stop_limit or abs(theta_y) > theta_y_stop_limit:
    stop forward motion
    allow only a conservative recovery command or stop

if abs(theta_x) > theta_x_slow_limit or abs(theta_y) > theta_y_slow_limit:
    reduce forward speed
```

The first implementation can simply stop when the stop limit is exceeded. A
later implementation can add a dedicated recovery state.

## States

### WAIT_FOR_MARKER

Stop until a recent ArUco detection is available.

This state is not responsible for long-range marker search. It only waits for
the upstream system or sensor pipeline to provide a valid marker detection.

Command:

```text
linear.x = 0.0
linear.y = 0.0
angular.z = 0.0
```

Transition:

```text
recent marker detection and aruco_z <= minimum_safe_z -> HOLD or DOCKED
recent marker detection and aruco_distance <= align_distance -> NEAR_ALIGN
recent marker detection and aruco_distance > align_distance -> FAR_GUIDED_APPROACH
```

This initial routing is important. The robot may start a docking attempt already
inside `align_distance`; in that case it should not pass through
`FAR_GUIDED_APPROACH`. It should go directly to `NEAR_ALIGN`, unless it is
already inside the forward-axis safety boundary.

### FAR_GUIDED_APPROACH

When the robot is far from the marker, drive forward while applying only weak
marker-centering yaw. Do not command lateral motion or marker-surface yaw
alignment in this range.

This is intentionally simple. At long distance, yaw estimation may be noisy, and
strong rotation can move the marker out of the camera view. The yaw command in
this state must use `theta_x`, not marker `yaw`.

`theta_x` points the camera toward the marker center. Marker `yaw` estimates the
marker surface orientation and should not be used until the robot is close.

Command:

```text
linear.x = far_approach_speed
linear.y = 0.0
angular.z = clamp(kp_far_center * theta_x,
                  -max_far_center_speed,
                  max_far_center_speed)
```

Transitions:

```text
aruco_z <= minimum_safe_z -> HOLD or DOCKED
aruco_distance <= align_distance -> NEAR_ALIGN
visibility stop guard violated -> HOLD
marker lost -> WAIT_FOR_MARKER
```

Optional slow-down:

```text
if visibility slow guard violated:
    linear.x = reduced_far_approach_speed
```

### NEAR_ALIGN

When the robot is close enough that the marker remains visible during local
correction, align lateral offset and marker yaw.

This state can command lateral and yaw motion. It should avoid strong forward
motion, because yaw plus forward motion has caused instability. The first
implementation should keep `linear.x = 0.0` in this state.

Command:

```text
lateral_error = aruco_x - target_x
yaw_error = wrap_pi(aruco_yaw - target_yaw)

linear.x = 0.0
linear.y = clamp(-kp_lateral * lateral_error,
                 -max_lateral_align_speed,
                 max_lateral_align_speed)
angular.z = clamp(kp_yaw * yaw_error,
                  -max_yaw_align_speed,
                  max_yaw_align_speed)
```

Transitions:

```text
aruco_z <= minimum_safe_z
and docked condition is true -> DOCKED

aruco_z <= minimum_safe_z
and docked condition is not true -> HOLD

abs(lateral_error) < x_tolerance
and abs(yaw_error) < yaw_tolerance -> FINAL_APPROACH

aruco_distance > align_distance + align_hysteresis -> FAR_GUIDED_APPROACH
visibility stop guard violated -> HOLD
marker lost -> WAIT_FOR_MARKER
```

If commanding lateral and yaw together still causes freezing, split this state
into two sub-states:

```text
NEAR_LATERAL_ALIGN
NEAR_YAW_ALIGN
```

### FINAL_APPROACH

Drive forward from `align_distance` toward `target_z`.

The robot is expected to be roughly laterally aligned and perpendicular to the
marker before entering this state. Forward speed should be lower than in
`FAR_GUIDED_APPROACH`.

The first implementation should use forward motion only. A later implementation
can add small yaw or lateral hold corrections if logs show it is safe.

Command:

```text
linear.x = final_approach_speed
linear.y = 0.0
angular.z = 0.0
```

Transitions:

```text
docked condition is true -> DOCKED

aruco_z <= minimum_safe_z
and docked condition is not true -> HOLD

if abs(lateral_error) > final_x_realign_threshold
or abs(yaw_error) > final_yaw_realign_threshold:
    return to NEAR_ALIGN

visibility stop guard violated -> HOLD
marker lost -> WAIT_FOR_MARKER
```

### HOLD

Stop briefly when the current state should not continue. `HOLD` is a short
pause, not a permanent state. The first implementation should hold for a bounded
time, then either resume a valid state if marker data is healthy or fall back to
`WAIT_FOR_MARKER`.

Command:

```text
linear.x = 0.0
linear.y = 0.0
angular.z = 0.0
```

Transitions:

```text
hold elapsed < hold_duration -> remain in HOLD
hold elapsed >= hold_duration and marker healthy and docked condition is true -> DOCKED
hold elapsed >= hold_duration and marker healthy and aruco_z <= minimum_safe_z -> HOLD
hold elapsed >= hold_duration and marker healthy and aruco_distance <= align_distance -> NEAR_ALIGN
hold elapsed >= hold_duration and marker healthy and aruco_distance > align_distance -> FAR_GUIDED_APPROACH
hold elapsed >= hold_duration and marker not healthy -> WAIT_FOR_MARKER
```

### BACK_OUT_RECOVERY

Optional state for later testing. This state is not part of the first automatic
implementation.

Use it when the robot is inside `minimum_safe_z`, the pose is not good
enough to call the robot docked, and the operator or upstream supervisor has
confirmed that moving backward is safe.

Command:

```text
linear.x = -back_out_speed
linear.y = 0.0
angular.z = 0.0
```

Transitions:

```text
aruco_distance >= back_out_target_distance -> NEAR_ALIGN
marker lost -> WAIT_FOR_MARKER
operator disables recovery -> HOLD
```

Safety constraints:

```text
back_out_speed must be low
do not command yaw while backing out
do not command lateral motion while backing out
stop immediately on marker timeout or upstream obstacle warning
```

### DOCKED

Stop the robot.

Command:

```text
linear.x = 0.0
linear.y = 0.0
angular.z = 0.0
```

## Initial Parameters

These are starting points for real-robot tests.

```yaml
target_distance: 1.0
align_distance: 1.2
minimum_safe_z: 1.0
align_hysteresis: 0.10
back_out_target_distance: 1.2

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
back_out_speed: 0.05
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

## Safety Requirements

- Stop when ArUco detection times out.
- Never command forward motion when `aruco_z <= minimum_safe_z`.
- Do not automatically drive backward from inside `minimum_safe_z` in the first
  implementation.
- If `BACK_OUT_RECOVERY` is added later, enable it only with operator or
  upstream-supervisor permission.
- Use only weak `theta_x` centering yaw in `FAR_GUIDED_APPROACH`; do not use
  marker `yaw` in the far range.
- Use lower speed in `FINAL_APPROACH` than in `FAR_GUIDED_APPROACH`.
- Do not command forward motion during `NEAR_ALIGN` in the first implementation.
- Treat visibility guard violations as higher priority than pose alignment.
- Keep `HOLD` bounded; after `hold_duration`, either resume a valid state or
  return to `WAIT_FOR_MARKER`.
- Log `/aruco/distance` and `/rov_cmd_vel` during each experiment.

## Validation Plan

Record logs with:

```bash
ros2 run aruco_dist_ctrl aruco_cmd_logger --ros-args \
  -p output_dir:=/tmp/aruco_docking_logs \
  -p log_rate:=20.0
```

Minimum experiments:

```text
1. FAR_GUIDED_APPROACH only
2. FAR_GUIDED_APPROACH + NEAR_ALIGN, with linear.x disabled in NEAR_ALIGN
3. Add FINAL_APPROACH to 1.0 m
4. If NEAR_ALIGN freezes, split it into lateral-only and yaw-only substates
```

For each run, note:

```text
- whether the marker stayed visible
- whether yaw command caused freezing
- whether lateral+yaw together caused freezing
- whether wheel commands saturated
- final aruco_z at stop
- final aruco_distance at stop
- final aruco_x and aruco_y at stop
- final aruco_yaw at stop
```

## Future Work

- Add an upstream navigation supervisor that switches between UWB navigation,
  obstacle handling, ArUco acquisition, and ArUco docking.
- Add camera active/inactive control for power saving after docking behavior is
  stable.
- Add marker pixel center fields to `ArucoDistance` for direct image-margin
  visibility checks.
- Replace hard state transitions with distance-dependent blending for smoother
  continuous docking.
- Add small hold corrections during `FINAL_APPROACH` after logs show they are
  safe.
- Add guarded `BACK_OUT_RECOVERY` once backward safety can be checked.
- Add slew rate limiting to `/rov_cmd_vel`.
- Normalize wheel commands in `rover_velocity` before individual motor limits.
