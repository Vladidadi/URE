#!/usr/bin/env python3

"""
Stereo Camera Node for ROS 2
============================

Captures frames from two USB webcams simultaneously, publishes them along with
CameraInfo messages on the exact topics expected by stereo_image_proc:
- left/image_raw
- left/camera_info
- right/image_raw
- right/camera_info

Calibration can be loaded from YAML files or defaults to identity matrices.

Author: [Your Name]
Date: 2025-11-10
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml


def load_camera_info(width: int, height: int, calibration_file: str = None) -> CameraInfo:
    """
    Load or generate a CameraInfo message for a camera.

    Args:
        width (int): Image width.
        height (int): Image height.
        calibration_file (str, optional): Path to YAML calibration file. If None, returns identity intrinsics.

    Returns:
        CameraInfo: Initialized CameraInfo message.
    """
    cam_info = CameraInfo()
    cam_info.width = width
    cam_info.height = height

    if calibration_file:
        with open(calibration_file, 'r') as f:
            calib = yaml.safe_load(f)
        cam_info.k = calib.get('K', [1,0,0,0,1,0,0,0,1])
        cam_info.d = calib.get('D', [0,0,0,0,0])
        cam_info.r = calib.get('R', [1,0,0,0,1,0,0,0,1])
        cam_info.p = calib.get('P', [1,0,0,0,1,0,0,0,1,0,0,0])
        cam_info.distortion_model = calib.get('distortion_model', 'plumb_bob')
    else:
        # Identity/dummy calibration
        cam_info.k = [1,0,0,0,1,0,0,0,1]
        cam_info.d = [0,0,0,0,0]
        cam_info.r = [1,0,0,0,1,0,0,0,1]
        cam_info.p = [1,0,0,0,1,0,0,0,1,0,0,0]
        cam_info.distortion_model = 'plumb_bob'

    return cam_info


class StereoCameraNode(Node):
    def __init__(self, left_calib: str = None, right_calib: str = None):
        super().__init__('stereo_camera_node')
        self.get_logger().info("Initializing Stereo Camera Node...")

        # ROS publishers for images and camera info
        self.left_pub = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, 'stereo/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, 'stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'stereo/right/camera_info', 10)

        # Open USB cameras (adjust indices as needed)
        self.left_cam = cv2.VideoCapture(2)
        self.right_cam = cv2.VideoCapture(4)

        if not self.left_cam.isOpened():
            self.get_logger().error("Failed to open left camera")
            raise RuntimeError("Left camera not available")
        if not self.right_cam.isOpened():
            self.get_logger().error("Failed to open right camera")
            raise RuntimeError("Right camera not available")

        # Image resolution
        self.width = 640
        self.height = 480
        self.left_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.left_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.right_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.right_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Frame rate
        self.frame_rate = 30.0

        self.bridge = CvBridge()

        # Load camera info
        self.left_info_msg = load_camera_info(self.width, self.height, left_calib)
        self.right_info_msg = load_camera_info(self.width, self.height, right_calib)

        # Timer to capture and publish frames
        self.timer = self.create_timer(1.0 / self.frame_rate, self.capture_and_publish)

        self.get_logger().info("Stereo Camera Node initialized successfully.")

    def capture_and_publish(self):
        """Capture frames and publish images and camera info."""
        ret_left, frame_left = self.left_cam.read()
        ret_right, frame_right = self.right_cam.read()

        if not ret_left or not ret_right:
            self.get_logger().warning("Failed to capture frames from cameras")
            return

        # Convert to ROS Image messages
        left_msg = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(frame_right, encoding='bgr8')

        # Set timestamps and frame IDs
        now = self.get_clock().now().to_msg()
        left_msg.header.stamp = now
        left_msg.header.frame_id = 'left_camera'
        right_msg.header.stamp = now
        right_msg.header.frame_id = 'right_camera'

        self.left_info_msg.header = left_msg.header
        self.right_info_msg.header = right_msg.header

        # Publish images and camera info
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.left_info_pub.publish(self.left_info_msg)
        self.right_info_pub.publish(self.right_info_msg)

        self.get_logger().debug("Published synchronized stereo frames")


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode(
        left_calib=None,   # Replace with path to left.yaml if available
        right_calib=None   # Replace with path to right.yaml if available
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Stereo Camera Node...")
    finally:
        node.left_cam.release()
        node.right_cam.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
