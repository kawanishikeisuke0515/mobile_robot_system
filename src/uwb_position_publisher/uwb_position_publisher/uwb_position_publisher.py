import math
import os
from typing import Optional
from typing import Sequence
from typing import Tuple

import rclpy
from rclpy.node import Node
from uwb_interfaces.msg import UwbDistances
from uwb_interfaces.msg import UwbPosition


def is_finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def trilaterate_2d(
    anchor_1: Tuple[float, float],
    anchor_2: Tuple[float, float],
    anchor_3: Tuple[float, float],
    distance_1: float,
    distance_2: float,
    distance_3: float,
    min_determinant: float,
) -> Optional[Tuple[float, float]]:
    if not is_finite(distance_1, distance_2, distance_3):
        return None

    x1, y1 = anchor_1
    x2, y2 = anchor_2
    x3, y3 = anchor_3

    a11 = 2.0 * (x2 - x1)
    a12 = 2.0 * (y2 - y1)
    a21 = 2.0 * (x3 - x1)
    a22 = 2.0 * (y3 - y1)
    b1 = (
        distance_1 ** 2
        - distance_2 ** 2
        - x1 ** 2
        + x2 ** 2
        - y1 ** 2
        + y2 ** 2
    )
    b2 = (
        distance_1 ** 2
        - distance_3 ** 2
        - x1 ** 2
        + x3 ** 2
        - y1 ** 2
        + y3 ** 2
    )

    determinant = a11 * a22 - a12 * a21
    if abs(determinant) <= min_determinant:
        return None

    x = (b1 * a22 - a12 * b2) / determinant
    y = (a11 * b2 - b1 * a21) / determinant
    return x, y


def build_position_message(
    distances_msg: UwbDistances,
    anchor_1: Tuple[float, float],
    anchor_2: Tuple[float, float],
    anchor_3: Tuple[float, float],
    min_determinant: float,
) -> UwbPosition:
    msg = UwbPosition()
    msg.header = distances_msg.header
    msg.device_time_ms = distances_msg.device_time_ms

    anchors_valid = (
        distances_msg.anchor_1_valid
        and distances_msg.anchor_2_valid
        and distances_msg.anchor_3_valid
    )
    if not anchors_valid:
        msg.x_m = math.nan
        msg.y_m = math.nan
        msg.valid = False
        return msg

    position = trilaterate_2d(
        anchor_1,
        anchor_2,
        anchor_3,
        float(distances_msg.anchor_1_distance_m),
        float(distances_msg.anchor_2_distance_m),
        float(distances_msg.anchor_3_distance_m),
        min_determinant,
    )
    if position is None:
        msg.x_m = math.nan
        msg.y_m = math.nan
        msg.valid = False
        return msg

    msg.x_m = position[0]
    msg.y_m = position[1]
    msg.valid = True
    return msg


class UwbPositionPublisher(Node):
    def __init__(self):
        super().__init__('uwb_position_publisher')

        self.declare_parameter('uwb_distances_topic', '/uwb/distances')
        self.declare_parameter('uwb_position_topic', '/uwb/position')
        self.declare_parameter('anchor_1_x', 0.0)
        self.declare_parameter('anchor_1_y', 0.0)
        self.declare_parameter('anchor_2_x', 1.0)
        self.declare_parameter('anchor_2_y', 0.0)
        self.declare_parameter('anchor_3_x', 0.0)
        self.declare_parameter('anchor_3_y', 1.0)
        self.declare_parameter('min_anchor_determinant', 1.0e-9)

        self.uwb_distances_topic = str(
            self.get_parameter('uwb_distances_topic').value
        )
        self.uwb_position_topic = str(
            self.get_parameter('uwb_position_topic').value
        )
        self.anchor_1 = (
            float(self.get_parameter('anchor_1_x').value),
            float(self.get_parameter('anchor_1_y').value),
        )
        self.anchor_2 = (
            float(self.get_parameter('anchor_2_x').value),
            float(self.get_parameter('anchor_2_y').value),
        )
        self.anchor_3 = (
            float(self.get_parameter('anchor_3_x').value),
            float(self.get_parameter('anchor_3_y').value),
        )
        self.min_anchor_determinant = float(
            self.get_parameter('min_anchor_determinant').value
        )
        self._validate_parameters()

        self.publisher_ = self.create_publisher(
            UwbPosition,
            self.uwb_position_topic,
            10,
        )
        self.create_subscription(
            UwbDistances,
            self.uwb_distances_topic,
            self.uwb_distances_callback,
            10,
        )

        self.get_logger().info(
            'subscribing %s, publishing %s, '
            'anchor_1=(%.3f, %.3f) anchor_2=(%.3f, %.3f) '
            'anchor_3=(%.3f, %.3f) min_anchor_determinant=%.3e'
            % (
                self.uwb_distances_topic,
                self.uwb_position_topic,
                self.anchor_1[0],
                self.anchor_1[1],
                self.anchor_2[0],
                self.anchor_2[1],
                self.anchor_3[0],
                self.anchor_3[1],
                self.min_anchor_determinant,
            )
        )

    def _validate_parameters(self):
        if self.uwb_distances_topic == '':
            raise ValueError('uwb_distances_topic must not be empty')
        if self.uwb_position_topic == '':
            raise ValueError('uwb_position_topic must not be empty')
        if not is_finite(
            self.anchor_1[0],
            self.anchor_1[1],
            self.anchor_2[0],
            self.anchor_2[1],
            self.anchor_3[0],
            self.anchor_3[1],
            self.min_anchor_determinant,
        ):
            raise ValueError(
                'anchor coordinates and min_anchor_determinant must be finite'
            )
        if self.min_anchor_determinant <= 0.0:
            raise ValueError('min_anchor_determinant must be greater than 0')

    def uwb_distances_callback(self, msg: UwbDistances):
        position_msg = build_position_message(
            msg,
            self.anchor_1,
            self.anchor_2,
            self.anchor_3,
            self.min_anchor_determinant,
        )
        self.publisher_.publish(position_msg)


def main(args: Optional[Sequence[str]] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = UwbPositionPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
