"""ROS-independent velocity filtering and proportional wheel control."""
from collections import deque
import math


def finite_vector(values, size):
    values = tuple(float(x) for x in values)
    if len(values) != size or not all(math.isfinite(x) for x in values):
        raise ValueError('Invalid vector')
    return values


def body_twist(linear, angular, rotation, offset):
    """Transform source-origin twist into target-origin twist (SI units)."""
    linear = finite_vector(linear, 3)
    angular = finite_vector(angular, 3)
    x, y, z, w = finite_vector(rotation, 4)
    r = finite_vector(offset, 3)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-12:
        raise ValueError('Invalid quaternion')
    x, y, z, w = (v / norm for v in (x, y, z, w))
    matrix = (
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    )
    def rotate(v):
        return tuple(sum(a*b for a, b in zip(row, v)) for row in matrix)
    v, omega = rotate(linear), rotate(angular)
    cross = (omega[1]*r[2]-omega[2]*r[1],
             omega[2]*r[0]-omega[0]*r[2],
             omega[0]*r[1]-omega[1]*r[0])
    return finite_vector((v[0]-cross[0], v[1]-cross[1], omega[2]), 3)


class VelocityControl:
    def __init__(self, *, kp=(0.1, 0.1, 0.1), gains=(1000., 1000., 30.),
                 window=5, filter_enabled=True, motor_limit=0.,
                 command_timeout=0.5, odom_timeout=0.5,
                 lever=375., radius=63.5):
        if type(window) is not int or window < 1:
            raise ValueError('window must be a positive integer')
        self.kp = finite_vector(kp, 3)
        self.gains = finite_vector(gains, 3)
        scalars = finite_vector((motor_limit, command_timeout, odom_timeout, lever, radius), 5)
        if any(x < 0 for x in self.kp) or scalars[0] < 0 or any(x <= 0 for x in scalars[1:]):
            raise ValueError('Invalid controller limits or geometry')
        self.limit, self.command_timeout, self.odom_timeout, self.lever, self.radius = scalars
        self.samples = deque(maxlen=window if filter_enabled else 1)
        self.command = (0., 0., 0.)
        self.command_time = None
        self.odom_time = None
        self.last_stamp = None
        self.fault = 'waiting_for_odom'

    def invalidate_odom(self, reason):
        self.samples.clear()
        self.odom_time = None
        self.last_stamp = None
        self.fault = reason

    def set_command(self, values, now):
        try:
            self.command = finite_vector(values, 3)
        except ValueError:
            self.command_time = None
            raise
        self.command_time = now

    def add_sample(self, values, stamp_ns, now):
        try:
            values = finite_vector(values, 3)
        except ValueError:
            self.invalidate_odom('invalid_odom')
            raise
        if self.odom_time is not None and now-self.odom_time > self.odom_timeout:
            self.invalidate_odom('odom_timeout')
        if self.last_stamp is not None:
            if stamp_ns == self.last_stamp:
                return False
            if stamp_ns < self.last_stamp:
                self.invalidate_odom('odom_time_reversed')
                return False
        self.samples.append((stamp_ns, values))
        self.last_stamp = stamp_ns
        self.odom_time = now
        self.fault = ''
        return True

    @property
    def measured(self):
        if not self.samples:
            return None
        return tuple(sum(v[i] for _, v in self.samples)/len(self.samples) for i in range(3))

    def output(self, now):
        if self.odom_time is not None and now-self.odom_time > self.odom_timeout:
            self.invalidate_odom('odom_timeout')
        state = self.fault
        if not state:
            if self.command_time is None or now-self.command_time > self.command_timeout:
                state = 'command_timeout'
            elif self.limit == 0:
                state = 'motor_limit_not_configured'
            elif self.command == (0., 0., 0.):
                state = 'stop_command'
        result = dict(state=state or 'running', target=self.command, measured=self.measured,
                      raw_measured=self.samples[-1][1] if self.samples else None,
                      sample_count=len(self.samples),
                      history_span=(self.samples[-1][0]-self.samples[0][0])*1e-9 if self.samples else 0.,
                      error=None, correction=None, raw_output=(0.,)*4, output=(0.,)*4, scale=1.)
        if state:
            return result
        error = tuple(a-b for a, b in zip(self.command, self.measured))
        correction = tuple(k*e for k, e in zip(self.kp, error))
        x, y, yaw = (g*(t+c) for g, t, c in zip(self.gains, self.command, correction))
        w = self.lever*yaw
        raw = ((x+y+w)/self.radius, (x-y-w)/self.radius,
               (x-y+w)/self.radius, (x+y-w)/self.radius)
        if not all(math.isfinite(v) for v in raw):
            result['state'] = 'nonfinite_output'
            return result
        peak = max(abs(v) for v in raw)
        scale = min(1., self.limit/peak) if peak else 1.
        result.update(error=error, correction=correction, raw_output=raw,
                      output=tuple(v*scale for v in raw), scale=scale)
        return result
