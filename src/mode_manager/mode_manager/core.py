"""ROS-independent state machine and session-aware command selection."""
from dataclasses import dataclass
import math
from uuid import uuid4


@dataclass(frozen=True)
class Output:
    session_id: str
    active: bool = False
    inputs_valid: bool = False
    target_reached: bool = False
    tracking_valid: bool = False
    docking_complete: bool = False
    target_marker_id: int = -1
    detection_sequence: int = 0
    command: tuple = (0.0, 0.0, 0.0)


class ModeMachine:
    def __init__(self, *, timeout=0.5, stable_time=0.5, min_detections=3,
                 wait_timeout=30.0, target_marker_id=0, auto_start=False):
        for value in (timeout, stable_time, wait_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('timeouts and stable_time must be finite and positive')
        if min_detections < 2 or target_marker_id < 0:
            raise ValueError('min_detections >= 2 and target_marker_id >= 0 required')
        self.timeout = timeout
        self.stable_time = stable_time
        self.min_detections = min_detections
        self.wait_timeout = wait_timeout
        self.target_marker_id = target_marker_id
        self.auto_start_pending = auto_start
        self.state = 'IDLE'
        self.session_id = uuid4().hex
        self.entered = 0.0
        self.outputs = {}
        self.stable_since = None
        self.first_sequence = None
        self.last_tick = None
        self.reason = 'waiting for start'

    def transition(self, state, now, reason):
        self.auto_start_pending = False
        self.state = state
        self.session_id = uuid4().hex
        self.entered = now
        self.outputs.clear()
        self.stable_since = None
        self.first_sequence = None
        self.reason = reason

    def start(self, now):
        if self.state not in ('IDLE', 'DONE', 'FAULT'):
            return False
        self.transition('UWB_APPROACH', now, 'start requested')
        return True

    def stop(self, now):
        self.transition('IDLE', now, 'stop requested')

    @property
    def selected(self):
        if self.state in ('UWB_APPROACH', 'UWB_RECOVERY'):
            return 'uwb'
        if self.state == 'VISION_DOCKING':
            return 'vision'
        return None

    def receive(self, source, output, now):
        if source not in ('uwb', 'vision') or output.session_id != self.session_id:
            return
        previous = self.outputs.get(source)
        self.outputs[source] = (output, now)
        if source == 'vision':
            if previous and (now - previous[1] > self.timeout or
                             output.detection_sequence < previous[0].detection_sequence):
                self.stable_since = self.first_sequence = None
            valid = (output.tracking_valid and
                     output.target_marker_id == self.target_marker_id)
            if not valid:
                self.stable_since = self.first_sequence = None
            elif self.stable_since is None:
                self.stable_since = now
                self.first_sequence = output.detection_sequence

    def recent(self, source, now):
        entry = self.outputs.get(source)
        if entry is None or not 0 <= now - entry[1] <= self.timeout:
            return None
        return entry[0]

    def tick(self, now):
        zero = (0.0, 0.0, 0.0)
        if self.last_tick is not None and now < self.last_tick:
            self.transition('FAULT', now, 'clock moved backwards')
        self.last_tick = now
        if self.state == 'IDLE' and self.auto_start_pending:
            uwb = self.recent('uwb', now)
            vision = self.recent('vision', now)
            if (uwb and uwb.inputs_valid and vision
                    and vision.target_marker_id == self.target_marker_id):
                self.start(now)
                self.reason = 'automatic start: controllers and UWB pose ready'
            return zero
        if self.state in ('IDLE', 'DONE', 'FAULT'):
            return zero
        uwb, vision = self.recent('uwb', now), self.recent('vision', now)
        if vision is None:
            self.stable_since = self.first_sequence = None
        # Allow one timeout for new-session acknowledgement, then latch a fault.
        required = ('uwb', 'vision') if self.state == 'VISION_WAIT' else (self.selected,)
        if now - self.entered > self.timeout and any(
                self.recent(source, now) is None for source in required):
            self.transition('FAULT', now, 'controller output timeout')
            return zero
        if self.state in ('UWB_APPROACH', 'UWB_RECOVERY'):
            if uwb and uwb.active and uwb.inputs_valid and uwb.target_reached:
                self.transition('VISION_WAIT', now, 'handoff pose reached')
                return zero
        elif self.state == 'VISION_WAIT':
            if now - self.entered >= self.wait_timeout:
                self.transition('FAULT', now, 'marker wait timeout')
            elif uwb and uwb.inputs_valid and not uwb.target_reached:
                self.transition('UWB_RECOVERY', now, 'handoff pose drifted')
            elif (uwb and uwb.inputs_valid and uwb.target_reached and vision
                  and vision.tracking_valid
                  and vision.target_marker_id == self.target_marker_id
                  and self.stable_since is not None
                  and now - self.stable_since >= self.stable_time
                  and vision.detection_sequence - self.first_sequence + 1 >= self.min_detections):
                self.transition('VISION_DOCKING', now, 'stable target tracking')
            return zero
        elif self.state == 'VISION_DOCKING':
            if vision and vision.active:
                # Loss takes priority over completion in the same observation.
                if not vision.tracking_valid or vision.target_marker_id != self.target_marker_id:
                    self.transition('UWB_RECOVERY', now, 'target tracking lost')
                    return zero
                if vision.inputs_valid and vision.docking_complete:
                    self.transition('DONE', now, 'docking complete')
                    return zero
        output = self.recent(self.selected, now)
        if output is None or not output.active or not output.inputs_valid:
            return zero
        if not all(math.isfinite(v) for v in output.command):
            self.transition('FAULT', now, 'non-finite command')
            return zero
        return output.command
