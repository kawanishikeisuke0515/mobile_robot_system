import math
from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def is_finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def apply_min_speed(value: float, minimum: float) -> float:
    if value == 0.0 or minimum <= 0.0:
        return value
    if abs(value) >= minimum:
        return value
    return math.copysign(minimum, value)


@dataclass(frozen=True)
class PoseControlConfig:
    target_x: float
    target_y: float
    target_yaw: float
    x_tolerance: float
    y_tolerance: float
    yaw_tolerance: float
    kp_x: float
    kp_y: float
    kp_yaw: float
    min_linear_speed: float
    max_linear_speed: float
    min_angular_speed: float
    max_angular_speed: float


@dataclass(frozen=True)
class PoseControlDebug:
    raw_error_world_x: float
    raw_error_world_y: float
    error_world_x: float
    error_world_y: float
    error_body_x: float
    error_body_y: float
    yaw_error: float
    target_reached: bool


@dataclass(frozen=True)
class PoseControlResult:
    linear_x: float
    linear_y: float
    angular_z: float
    debug: PoseControlDebug


def calculate_pose_command(
    current_x: float,
    current_y: float,
    current_yaw: float,
    config: PoseControlConfig,
) -> PoseControlResult:
    raw_error_world_x = config.target_x - current_x
    raw_error_world_y = config.target_y - current_y
    yaw_error = wrap_pi(config.target_yaw - current_yaw)

    target_reached = (
        abs(raw_error_world_x) <= config.x_tolerance
        and abs(raw_error_world_y) <= config.y_tolerance
        and abs(yaw_error) <= config.yaw_tolerance
    )

    error_world_x = (
        0.0
        if abs(raw_error_world_x) <= config.x_tolerance
        else raw_error_world_x
    )
    error_world_y = (
        0.0
        if abs(raw_error_world_y) <= config.y_tolerance
        else raw_error_world_y
    )

    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)
    error_body_x = -sin_yaw * error_world_x + cos_yaw * error_world_y
    error_body_y = cos_yaw * error_world_x + sin_yaw * error_world_y

    linear_x = config.kp_x * error_body_x
    linear_y = config.kp_y * error_body_y
    angular_z = (
        0.0
        if abs(yaw_error) <= config.yaw_tolerance
        else config.kp_yaw * yaw_error
    )

    linear_x = clamp(linear_x, -config.max_linear_speed, config.max_linear_speed)
    linear_y = clamp(linear_y, -config.max_linear_speed, config.max_linear_speed)
    angular_z = clamp(angular_z, -config.max_angular_speed, config.max_angular_speed)

    linear_x = apply_min_speed(linear_x, config.min_linear_speed)
    linear_y = apply_min_speed(linear_y, config.min_linear_speed)
    angular_z = apply_min_speed(angular_z, config.min_angular_speed)

    debug = PoseControlDebug(
        raw_error_world_x=raw_error_world_x,
        raw_error_world_y=raw_error_world_y,
        error_world_x=error_world_x,
        error_world_y=error_world_y,
        error_body_x=error_body_x,
        error_body_y=error_body_y,
        yaw_error=yaw_error,
        target_reached=target_reached,
    )
    return PoseControlResult(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        debug=debug,
    )
