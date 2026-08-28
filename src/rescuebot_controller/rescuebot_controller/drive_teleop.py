#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class DriveTeleop(Node):

    def __init__(self):
        super().__init__('drive_teleop')

        # Scale factors — tune these for your robot
        self.linear_scale = 0.2   # max m/s at full stick
        self.angular_scale = 1.0  # max rad/s at full stick

        # Latest commanded velocities, updated by joy_callback
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publish at 10 Hz, matching twist_mux's navigation timeout of 0.5s
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.get_logger().info("Drive teleop started")

    def joy_callback(self, msg):
        # axis 1 = forward/backward
        # axis 2 = rotation
        self.linear_x = msg.axes[1] * self.linear_scale
        self.angular_z = msg.axes[3] * self.angular_scale

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z

        self.publisher.publish(msg)

        self.get_logger().info(
            f"linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = DriveTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()