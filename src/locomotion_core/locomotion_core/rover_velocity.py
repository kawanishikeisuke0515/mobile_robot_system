import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String, Bool

import time
import numpy as np
from . import RoverDescription as Rover

class get_move_cmds(Node):

    def __init__(self):
        super().__init__('rover_velocity')

        self.declare_parameter('log_motor_commands', False)
        self.declare_parameter('log_zero_motor_commands', False)
        self.log_motor_commands = bool(self.get_parameter('log_motor_commands').value)
        self.log_zero_motor_commands = bool(self.get_parameter('log_zero_motor_commands').value)

        # Initialize the rover object to define the rover.
        self.rov = Rover.RoverDescription('RobArm', ((205, 170), 63.5))

        # Create a subscriber to receive controller commands
        self.subscription = self.create_subscription(
            Twist,
            '/rov_cmd_vel',
            self.move_cmd_callback,
            5)
        self.subscription  # prevent unused variable warning
        
        # Create the publishers to the roboteq motors.
        timer_period = 0.1  # seconds
        self.pub_du = self.create_publisher(Float32MultiArray, 'rov/motors', 10)
        self.pub_ded = self.create_publisher(Bool, "deadman", 5)
        self.timer = self.create_timer(timer_period, self.drive_unit_callback)
        self.i = 0
        
        # RPM Vector to publish to the roboteq controller.
        self.rpm_vec = [0.0, 0.0, 0.0, 0.0]

    # Computes the velocity vector based on the controller input.
    def move_cmd_callback(self, msg):
        vel_vector = (msg.linear.x*1000, msg.linear.y*1000, 30*msg.angular.z)
        #vel_vector = (msg.linear.x*500, msg.linear.y*500, 5*msg.angular.z/3)
        angular_vel_vector = self.rov.kinematic_equation(vel_vector)
        #briefly changing to 100. rememer to increase back to 600 and multiply by 60
        self.rpm_vec = [max(min(i, 15.0), -15.0) for i in angular_vel_vector]
        #self.rpm_vec = [msg.linear.x, msg.linear.x, msg.linear.x, msg.linear.x]
        if self._should_log_motor_commands():
            self.get_logger().info(f"self.rpm_vec:  {self.rpm_vec}")

    def _should_log_motor_commands(self):
        if not self.log_motor_commands:
            return False
        if self.log_zero_motor_commands:
            return True
        return any(abs(value) > 1e-6 for value in self.rpm_vec)

    # Publishes to the roboteq controller node.
    def drive_unit_callback(self):
        msg = Float32MultiArray()
        # print(self.rpm_vec)
        #msg.data = -[100.0, 100.0, 100.0, 100.0]
        msg.data = self.rpm_vec

        # print(msg.data)
        bool_msg = Bool()
        bool_msg.data = True
        # Publish to all the motor drivers.
        self.pub_ded.publish(bool_msg)
        self.pub_du.publish(msg)
        self.i += 1 

def main(args=None):
    rclpy.init(args=args)

    sub_move_cmds = get_move_cmds()

    rclpy.spin(sub_move_cmds)

    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
