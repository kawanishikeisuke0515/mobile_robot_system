import math

from uwb_position_zed_pose_ctrl.pose_control import (
    PoseControlConfig,
    calculate_pose_command,
    wrap_pi,
)


def default_config(**overrides):
    values = {
        'target_x': 0.0,
        'target_y': 0.0,
        'target_yaw': 0.0,
        'x_tolerance': 0.05,
        'y_tolerance': 0.05,
        'yaw_tolerance': 0.05,
        'kp_x': 1.0,
        'kp_y': 1.0,
        'kp_yaw': 1.0,
        'min_linear_speed': 0.0,
        'max_linear_speed': 0.5,
        'min_angular_speed': 0.0,
        'max_angular_speed': 0.5,
    }
    values.update(overrides)
    return PoseControlConfig(**values)


def test_wrap_pi_maps_positive_pi_to_negative_pi():
    assert wrap_pi(math.pi) == -math.pi


def test_exact_180_degree_yaw_error_rotates_clockwise():
    result = calculate_pose_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=math.pi,
        config=default_config(max_angular_speed=10.0),
    )

    assert result.debug.yaw_error == -math.pi
    assert result.angular_z < 0.0


def test_target_tolerance_publishes_zero_velocity():
    result = calculate_pose_command(
        current_x=0.03,
        current_y=-0.04,
        current_yaw=0.02,
        config=default_config(),
    )

    assert result.debug.target_reached
    assert result.linear_x == 0.0
    assert result.linear_y == 0.0
    assert result.angular_z == 0.0


def test_yaw_positive_90_world_negative_x_is_forward():
    result = calculate_pose_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=math.pi / 2.0,
        config=default_config(target_x=-1.0, max_linear_speed=10.0),
    )

    assert result.debug.error_body_x == 1.0
    assert abs(result.debug.error_body_y) < 1.0e-9
    assert result.linear_x == 1.0
    assert abs(result.linear_y) < 1.0e-9


def test_yaw_positive_90_world_positive_x_is_backward():
    result = calculate_pose_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=math.pi / 2.0,
        config=default_config(target_x=1.0, max_linear_speed=10.0),
    )

    assert result.debug.error_body_x == -1.0
    assert abs(result.debug.error_body_y) < 1.0e-9
    assert result.linear_x == -1.0
    assert abs(result.linear_y) < 1.0e-9


def test_yaw_zero_world_positive_y_is_forward():
    result = calculate_pose_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=0.0,
        config=default_config(target_y=1.0, max_linear_speed=10.0),
    )

    assert result.debug.error_body_x == 1.0
    assert abs(result.debug.error_body_y) < 1.0e-9
    assert result.linear_x == 1.0
    assert abs(result.linear_y) < 1.0e-9


def test_yaw_zero_world_positive_x_is_lateral_positive():
    result = calculate_pose_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=0.0,
        config=default_config(target_x=1.0, max_linear_speed=10.0),
    )

    assert abs(result.debug.error_body_x) < 1.0e-9
    assert result.debug.error_body_y == 1.0
    assert abs(result.linear_x) < 1.0e-9
    assert result.linear_y == 1.0


def test_min_speed_preserves_command_sign():
    result = calculate_pose_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=0.0,
        config=default_config(
            target_y=-0.2,
            kp_x=0.1,
            min_linear_speed=0.05,
            max_linear_speed=1.0,
        ),
    )

    assert result.linear_x == -0.05
