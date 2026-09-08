"""Optional managed-mode adapter; standalone controllers retain Twist output."""
import math
import time

from geometry_msgs.msg import Twist
from mode_manager_interfaces.msg import ControllerOutput, ControlRequest


class ControllerLink:
    def __init__(self, node, source, reset):
        self.node, self.reset = node, reset
        node.declare_parameter('managed_mode', False)
        node.declare_parameter('manager_timeout', 0.5)
        self.managed = bool(node.get_parameter('managed_mode').value)
        self.timeout = float(node.get_parameter('manager_timeout').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('manager_timeout must be finite and positive')
        self.session_id = ''
        self.enabled = False
        self.received = None
        self.last_stamp = -1
        self.publisher = node.create_publisher(ControllerOutput, f'/{source}/control_output', 1)
        self.subscription = node.create_subscription(
            ControlRequest, f'/{source}/control_request', self.request, 1)

    def request(self, msg):
        if not self.managed:
            return
        stamp = msg.stamp.sec * 10**9 + msg.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) * 1e-9
        if not msg.session_id or stamp <= self.last_stamp or not 0 <= age <= self.timeout:
            return
        self.last_stamp = stamp
        if msg.session_id != self.session_id or msg.enabled != self.enabled:
            self.reset()
        self.session_id = msg.session_id
        self.enabled = msg.enabled
        self.received = time.monotonic()

    @property
    def active(self):
        if not self.managed:
            return True
        if self.received is None or time.monotonic() - self.received > self.timeout:
            if self.enabled:
                self.enabled = False
                self.reset()
            return False
        return self.enabled

    def publish(self, command, *, inputs_valid=False, target_reached=False,
                tracking_valid=False, docking_complete=False,
                target_marker_id=-1, detection_sequence=0):
        active = self.active
        msg = ControllerOutput()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.session_id = self.session_id
        msg.active = active
        msg.inputs_valid = inputs_valid
        msg.target_reached = target_reached
        msg.tracking_valid = tracking_valid
        msg.docking_complete = docking_complete and active
        msg.target_marker_id = target_marker_id
        msg.detection_sequence = detection_sequence
        msg.command = command if active and inputs_valid else Twist()
        self.publisher.publish(msg)
        if not self.managed:
            self.node.cmd_publisher.publish(command)
