import math
import pytest
from locomotion_closed_loop.control import VelocityControl, body_twist


def test_average_duplicate_and_rollover():
    c = VelocityControl(window=2)
    c.add_sample((1, 2, 3), 1, 0.)
    assert c.measured == (1, 2, 3)
    assert not c.add_sample((9, 9, 9), 1, .1)
    assert c.odom_time == 0.
    c.add_sample((3, 4, 5), 2, .2)
    assert c.measured == (2, 3, 4)
    c.add_sample((5, 6, 7), 3, .3)
    assert c.measured == (4, 5, 6)


@pytest.mark.parametrize('window', [0, -1, 1.5, True])
def test_invalid_window(window):
    with pytest.raises(ValueError):
        VelocityControl(window=window)


def test_disabled_filter():
    c = VelocityControl(filter_enabled=False)
    c.add_sample((1, 2, 3), 1, 0.)
    c.add_sample((4, 5, 6), 2, .1)
    assert c.measured == (4, 5, 6)


def test_three_axis_p_and_allocation_without_15_clip():
    c = VelocityControl(kp=(1, 2, 3), gains=(1, 1, 1), lever=1, radius=1, motor_limit=100)
    c.set_command((10, 5, 2), 0.)
    c.add_sample((2, 1, 1), 1, 0.)
    out = c.output(.1)
    assert out['correction'] == (8, 8, 3)
    assert out['output'] == (36, 0, 10, 26)
    c.limit = 18
    assert c.output(.1)['output'] == (18, 0, 5, 13)


def test_stop_timeouts_and_history_reset():
    c = VelocityControl(motor_limit=100)
    c.set_command((1, 0, 0), 0.)
    c.add_sample((0, 0, 0), 1, 0.)
    assert c.output(.1)['state'] == 'running'
    c.set_command((0, 0, 0), .1)
    assert c.output(.2)['output'] == (0,)*4
    assert len(c.samples) == 1
    assert c.output(.6)['state'] == 'odom_timeout'
    assert not c.samples
    c.add_sample((2, 3, 4), 2, .7)
    assert c.measured == (2, 3, 4)
    assert c.output(.7)['state'] == 'command_timeout'


def test_invalid_and_reversed_samples():
    c = VelocityControl()
    c.add_sample((1, 1, 1), 5, 0.)
    assert not c.add_sample((2, 2, 2), 4, .1)
    assert not c.samples
    c.add_sample((1, 1, 1), 6, .2)
    with pytest.raises(ValueError):
        c.add_sample((float('nan'), 0, 0), 7, .3)
    assert c.measured is None
    with pytest.raises(ValueError):
        c.set_command((float('inf'), 0, 0), .4)
    assert c.command_time is None


def test_transform_rotation_and_camera_offset():
    assert body_twist((0, 1, 0), (0, 0, 1), (0, 0, 0, 1), (1, 0, 0)) == (0, 0, 1)
    assert body_twist((1, 0, 0), (0, 0, 0), (0, 0, math.sqrt(.5), math.sqrt(.5)), (0, 0, 0)) == pytest.approx((0, 1, 0))


def test_unconfigured_limit_inhibits_output():
    c = VelocityControl()
    c.set_command((1, 1, 1), 0.)
    c.add_sample((0, 0, 0), 1, 0.)
    assert c.output(.1)['state'] == 'motor_limit_not_configured'
    assert c.output(.1)['output'] == (0,)*4
