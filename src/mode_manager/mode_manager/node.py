"""ROS transport for the docking mode machine."""
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from mode_manager_interfaces.msg import ControllerOutput, ControlRequest
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import ModeMachine, Output


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        defaults = dict(output_timeout=0.5, stable_detection_time=0.5,
                        min_detections=3, vision_wait_timeout=30.0,
                        target_marker_id=0, control_rate=20.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        def value(key):
            return self.get_parameter(key).value
        rate = float(value('control_rate'))
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('control_rate must be finite and positive')
        self.machine = ModeMachine(
            timeout=float(value('output_timeout')),
            stable_time=float(value('stable_detection_time')),
            min_detections=int(value('min_detections')),
            wait_timeout=float(value('vision_wait_timeout')),
            target_marker_id=int(value('target_marker_id')))
        self.cmd = self.create_publisher(Twist, '/rov_cmd_vel', 1)
        self.status = self.create_publisher(String, '/mode_manager/state', 1)
        self.requests = {}
        self.subscriptions_ = []
        self.last_stamps = {}
        for source in ('uwb', 'vision'):
            self.requests[source] = self.create_publisher(
                ControlRequest, f'/{source}/control_request', 1)
            self.subscriptions_.append(self.create_subscription(
                ControllerOutput, f'/{source}/control_output',
                lambda msg, source=source: self.receive(source, msg), 1))
        self.create_service(Trigger, '~/start', self.start)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_timer(1.0 / rate, self.tick)
        self.last_ros_time = self.get_clock().now().nanoseconds

    def start(self, request, response):
        response.success = self.machine.start(time.monotonic())
        response.message = self.machine.state
        if response.success:
            self.last_stamps.clear()
        self.tick()
        return response

    def stop(self, request, response):
        self.machine.stop(time.monotonic())
        self.last_stamps.clear()
        self.tick()
        response.success = True
        response.message = 'stopped'
        return response

    def receive(self, source, msg):
        stamp = msg.stamp.sec * 10**9 + msg.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if msg.session_id != self.machine.session_id or not 0 <= age <= self.machine.timeout:
            return
        key = (source, msg.session_id)
        if stamp <= self.last_stamps.get(key, -1):
            return
        self.last_stamps[key] = stamp
        command = msg.command
        self.machine.receive(source, Output(
            session_id=msg.session_id, active=msg.active, inputs_valid=msg.inputs_valid,
            target_reached=msg.target_reached, tracking_valid=msg.tracking_valid,
            docking_complete=msg.docking_complete, target_marker_id=msg.target_marker_id,
            detection_sequence=msg.detection_sequence,
            command=(command.linear.x, command.linear.y, command.angular.z)), time.monotonic() - age)

    def tick(self):
        now = time.monotonic()
        ros_now = self.get_clock().now()
        if ros_now.nanoseconds < self.last_ros_time:
            self.machine.transition('FAULT', now, 'ROS clock moved backwards')
        self.last_ros_time = ros_now.nanoseconds
        previous = self.machine.session_id
        velocity = self.machine.tick(now)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = velocity
        self.cmd.publish(cmd)
        if previous != self.machine.session_id:
            self.last_stamps.clear()
            self.get_logger().info(f'{self.machine.state}: {self.machine.reason}')
        for source, publisher in self.requests.items():
            msg = ControlRequest()
            msg.stamp = ros_now.to_msg()
            msg.session_id = self.machine.session_id
            msg.enabled = self.machine.selected == source
            publisher.publish(msg)
        self.status.publish(String(data=self.machine.state))


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init(args=args)
    node = None
    try:
        node = ModeManager()
        rclpy.spin(node)
    finally:
        if node is not None:
            if rclpy.ok():
                node.machine.stop(time.monotonic())
                node.tick()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
