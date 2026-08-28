#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class PrismaticTeleop(Node):

    def __init__(self):
        super().__init__('prismatic_teleop')

        # Current joint positions
        # [front_slot, front, back_slot, back]
        self.positions = [0.0, 0.0, 0.0, 0.0]

        # How much each button press moves the joint
        self.step = 0.005

        # Joint limits
        self.limits = [
            (-0.28, 0.195),  # front_slot
            (0.0, 0.05),     # front
            (-0.28, 0.18),   # back_slot
            (0.0, 0.07)      # back
        ]

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/prismatic_controller/commands',
            10
        )

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info("Prismatic teleop started")

    def move_joint(self, index, amount):

        new_position = self.positions[index] + amount

        minimum, maximum = self.limits[index]

        # Keep position inside joint limits
        new_position = max(minimum, min(maximum, new_position))

        self.positions[index] = new_position

    def publish_positions(self):

        msg = Float64MultiArray()
        msg.data = self.positions

        self.publisher.publish(msg)

        self.get_logger().info(
            f"front_slot={self.positions[0]:.3f}, "
            f"front={self.positions[1]:.3f}, "
            f"back_slot={self.positions[2]:.3f}, "
            f"back={self.positions[3]:.3f}"
        )

    def joy_callback(self, msg):

        # D-pad
        #
        # PS5 joy mappings commonly:
        # axis 6 = D-pad left/right
        # axis 7 = D-pad up/down

        dpad_horizontal = msg.axes[6]
        dpad_vertical = msg.axes[7]

        # D-pad UP
        if dpad_vertical > 0.5:
            self.move_joint(0, self.step)

        # D-pad DOWN
        elif dpad_vertical < -0.5:
            self.move_joint(0, -self.step)

        # D-pad RIGHT
        if dpad_horizontal > 0.5:
            self.move_joint(1, self.step)

        # D-pad LEFT
        elif dpad_horizontal < -0.5:
            self.move_joint(1, -self.step)

        # Buttons
        #
        # Typical PS5 mapping:
        # 0 = Cross
        # 1 = Circle
        # 2 = Square
        # 3 = Triangle

        # Triangle
        if msg.buttons[2]:
            self.move_joint(2, self.step)

        # Cross
        if msg.buttons[0]:
            self.move_joint(2, -self.step)

        # Square
        if msg.buttons[3]:
            self.move_joint(3, self.step)

        # Circle
        if msg.buttons[1]:
            self.move_joint(3, -self.step)

        self.publish_positions()


def main(args=None):

    rclpy.init(args=args)

    node = PrismaticTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()