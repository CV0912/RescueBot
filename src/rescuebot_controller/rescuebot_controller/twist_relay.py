#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped


class TwistRelayNode(Node):

    def __init__(self):
        super().__init__("twist_relay")

        # Nav2 publishes Twist here
        self.twist_sub = self.create_subscription(
            Twist,
            "/cmd_vel_unstamped",
            self.twist_callback,
            10
        )

        # Mecanum controller receives TwistStamped here
        self.twist_stamped_pub = self.create_publisher(
            TwistStamped,
            "/mecanum_drive_controller/reference",
            10
        )
        self.joy_sub = self.create_subscription(
            TwistStamped,
            "/input_joy/cmd_vel_stamped",
            self.joy_twist_callback,
            10
        )
        self.joy_pub = self.create_publisher(
            Twist,
            "/input_joy/cmd_vel",
            10
        )
    def twist_callback(self, msg):

        stamped_msg = TwistStamped()

        # Add current ROS time
        stamped_msg.header.stamp = self.get_clock().now().to_msg()

        # Copy the Twist
        stamped_msg.twist = msg

        self.twist_stamped_pub.publish(stamped_msg)

    def joy_twist_callback(self, msg):
        twist = Twist()
        twist = msg.twist
        self.joy_pub.publish(twist)
def main(args=None):

    rclpy.init(args=args)

    node = TwistRelayNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()