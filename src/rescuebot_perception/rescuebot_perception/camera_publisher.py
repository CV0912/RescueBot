#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('camera_info_url', '')  # path to a calibration yaml

        device = self.get_parameter('device').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        # match the topic apriltag_node is actually subscribed to
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.bridge = CvBridge()

        self.camera_info_msg = self._build_camera_info(width, height)

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open device: {device}')
        else:
            self.get_logger().info(f'Streaming from {device}')

        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _build_camera_info(self, width, height):
        # PLACEHOLDER intrinsics — replace with a real calibration
        # (see camera_calibration package) or apriltag pose estimates will be wrong
        info = CameraInfo()
        info.width = width
        info.height = height
        fx = fy = float(width)  # rough guess, NOT a real calibration
        cx, cy = width / 2.0, height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame grab failed, retrying...')
            return

        stamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        self.image_pub.publish(msg)

        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = 'camera'
        self.info_pub.publish(self.camera_info_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()