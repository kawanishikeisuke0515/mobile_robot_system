import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mode_manager.core import ModeMachine, Output

ZERO = (0.0, 0.0, 0.0)
MOVE = (0.2, -0.1, 0.05)


def send(m, source, now, **kwargs):
    output = Output(session_id=m.session_id, **kwargs)
    m.receive(source, output, now)
    return output


def waiting():
    m = ModeMachine(stable_time=0.2, min_detections=3)
    assert m.start(0)
    send(m, 'uwb', 0.1, active=True, inputs_valid=True, target_reached=True)
    assert m.tick(0.1) == ZERO
    assert m.state == 'VISION_WAIT'
    return m


def docking():
    m = waiting()
    for now, count in ((0.2, 1), (0.35, 2), (0.5, 3)):
        send(m, 'uwb', now, inputs_valid=True, target_reached=True)
        send(m, 'vision', now, tracking_valid=True, target_marker_id=0,
             detection_sequence=count)
        assert m.tick(now) == ZERO
    assert m.state == 'VISION_DOCKING'
    return m


def test_selected_source_and_old_session_rejection():
    m = ModeMachine()
    m.start(0)
    old = send(m, 'uwb', 0.1, active=True, inputs_valid=True, command=MOVE)
    send(m, 'vision', 0.1, active=True, inputs_valid=True, command=(9., 9., 9.))
    assert m.tick(0.1) == MOVE
    m.stop(0.2)
    m.start(0.3)
    m.receive('uwb', old, 0.4)
    assert m.tick(0.4) == ZERO


def test_detection_heartbeat_alone_cannot_start_vision():
    m = waiting()
    for now in (0.2, 0.4, 0.6):
        send(m, 'uwb', now, inputs_valid=True, target_reached=True)
        send(m, 'vision', now, tracking_valid=True, target_marker_id=0,
             detection_sequence=1)
        m.tick(now)
    assert m.state == 'VISION_WAIT'


def test_wrong_marker_never_starts_vision():
    m = waiting()
    for now in (0.2, 0.4, 0.6):
        send(m, 'uwb', now, inputs_valid=True, target_reached=True)
        send(m, 'vision', now, tracking_valid=True, target_marker_id=99,
             detection_sequence=int(now * 100))
        m.tick(now)
    assert m.state == 'VISION_WAIT'


def test_loss_returns_via_uwb_and_needs_valid_inputs():
    m = docking()
    send(m, 'vision', 0.6, active=True, inputs_valid=True, tracking_valid=True,
         target_marker_id=0, command=MOVE)
    assert m.tick(0.6) == MOVE
    send(m, 'vision', 0.7, active=True, tracking_valid=False)
    assert m.tick(0.7) == ZERO
    assert m.state == 'UWB_RECOVERY'
    send(m, 'uwb', 0.8, active=True, inputs_valid=False, command=MOVE)
    assert m.tick(0.8) == ZERO
    send(m, 'uwb', 0.9, active=True, inputs_valid=True, command=MOVE)
    send(m, 'vision', 0.9, tracking_valid=True, target_marker_id=0)
    assert m.tick(0.9) == MOVE
    assert m.state == 'UWB_RECOVERY'
    send(m, 'uwb', 1., active=True, inputs_valid=True, target_reached=True)
    assert m.tick(1.) == ZERO
    assert m.state == 'VISION_WAIT'


def test_completion_latches_done():
    m = docking()
    send(m, 'vision', 0.6, active=True, inputs_valid=True, tracking_valid=True,
         target_marker_id=0, docking_complete=True, command=MOVE)
    assert m.tick(0.6) == ZERO
    assert m.state == 'DONE'
    send(m, 'vision', 0.7, active=True, tracking_valid=False)
    assert m.tick(0.7) == ZERO
    assert m.state == 'DONE'


def test_loss_takes_priority_over_completion():
    m = docking()
    send(m, 'vision', 0.6, active=True, docking_complete=True)
    m.tick(0.6)
    assert m.state == 'UWB_RECOVERY'


def test_output_timeout_latches_fault_and_requires_start():
    m = ModeMachine()
    m.start(0)
    send(m, 'uwb', 0.1, active=True, inputs_valid=True, command=MOVE)
    assert m.tick(0.7) == ZERO
    assert m.state == 'FAULT'
    send(m, 'uwb', 0.8, active=True, inputs_valid=True, command=MOVE)
    assert m.tick(0.8) == ZERO
    assert m.start(0.9)


def test_pose_drift_keeps_waiting_and_preserves_stable_detection():
    m = waiting()
    session = m.session_id
    for now, count in ((0.2, 1), (0.35, 2), (0.5, 3)):
        send(m, 'uwb', now, inputs_valid=True, target_reached=False)
        send(m, 'vision', now, tracking_valid=True, target_marker_id=0,
             detection_sequence=count)
        assert m.tick(now) == ZERO
        assert m.state == 'VISION_WAIT'
        assert m.session_id == session
    assert m.stable_since == 0.2
    send(m, 'uwb', 0.55, inputs_valid=True, target_reached=True)
    assert m.tick(0.55) == ZERO
    assert m.state == 'VISION_DOCKING'


def test_wait_timeout():
    m = waiting()
    send(m, 'uwb', 30.2, inputs_valid=True, target_reached=True)
    send(m, 'vision', 30.2)
    m.tick(30.2)
    assert m.state == 'FAULT'
    assert m.reason == 'marker wait timeout'


def test_nonfinite_command_and_backward_time():
    m = ModeMachine()
    m.start(0)
    send(m, 'uwb', 0.1, active=True, inputs_valid=True, command=(float('nan'), 0., 0.))
    assert m.tick(0.1) == ZERO
    assert m.state == 'FAULT'
    m.start(1)
    m.tick(1)
    assert m.tick(0.5) == ZERO
    assert m.state == 'FAULT'


@pytest.mark.parametrize('kwargs', [dict(timeout=0), dict(stable_time=float('nan')),
                                   dict(min_detections=1), dict(target_marker_id=-1)])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        ModeMachine(**kwargs)


def test_vision_transport_outage_is_fault_not_recovery():
    m = docking()
    send(m, 'vision', 0.6, active=True, tracking_valid=True, inputs_valid=True,
         target_marker_id=0, command=MOVE)
    assert m.tick(0.6) == MOVE
    assert m.tick(1.2) == ZERO
    assert m.state == 'FAULT'


def test_unobserved_gap_resets_stable_detection():
    m = waiting()
    send(m, 'vision', 0.2, tracking_valid=True, target_marker_id=0, detection_sequence=1)
    send(m, 'vision', 1.0, tracking_valid=True, target_marker_id=0, detection_sequence=9)
    send(m, 'uwb', 1.0, inputs_valid=True, target_reached=True)
    assert m.tick(1.0) == ZERO
    assert m.state == 'VISION_WAIT'


def test_stop_and_restart_while_active():
    m = docking()
    assert not m.start(0.6)
    m.stop(0.7)
    assert m.tick(0.7) == ZERO
    assert m.state == 'IDLE'


def test_auto_start_waits_for_ready_inputs_without_startup_timeout():
    m = ModeMachine(auto_start=True)
    assert m.tick(100) == ZERO
    assert m.state == 'IDLE'
    send(m, 'uwb', 101, inputs_valid=False)
    send(m, 'vision', 101, target_marker_id=0)
    m.tick(101)
    assert m.state == 'IDLE'
    send(m, 'uwb', 102, inputs_valid=True)
    m.tick(102)  # Vision output expired.
    assert m.state == 'IDLE'
    send(m, 'vision', 102, target_marker_id=99)
    m.tick(102)
    assert m.state == 'IDLE'
    send(m, 'vision', 103, target_marker_id=0, tracking_valid=False)
    send(m, 'uwb', 103, inputs_valid=True)
    assert m.tick(103) == ZERO
    assert m.state == 'UWB_APPROACH'
    assert not m.auto_start_pending
    assert m.tick(103.1) == ZERO  # New-session output is required.


def test_auto_start_is_cancelled_by_stop_or_fault():
    for action in ('stop', 'fault', 'done'):
        m = ModeMachine(auto_start=True)
        if action == 'stop':
            m.stop(0)
        else:
            m.transition(action.upper(), 0, action)
        send(m, 'uwb', 1, inputs_valid=True)
        send(m, 'vision', 1, target_marker_id=0)
        assert m.tick(1) == ZERO
        assert not m.auto_start_pending
        assert m.state != 'UWB_APPROACH'
