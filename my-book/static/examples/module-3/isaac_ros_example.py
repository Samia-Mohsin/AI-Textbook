#!/usr/bin/env python3

"""
Basic Isaac ROS example for perception pipeline
This demonstrates how to use Isaac ROS nodes for perception
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
from cv_bridge import CvBridge
import numpy as np


class IsaacPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_perception_node')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create CV bridge for image processing
        self.bridge = CvBridge()

        # Subscribe to camera topics
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for processed image
        self.processed_image_pub = self.create_publisher(
            Image,
            '/camera/processed_image',
            10
        )

        self.get_logger().info("Isaac Perception Node initialized")

    def image_callback(self, msg):
        """Process incoming images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Example: Apply simple image processing (edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Convert back to ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
            processed_msg.header = msg.header

            # Publish processed image
            self.processed_image_pub.publish(processed_msg)

            self.get_logger().info("Processed image published")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def camera_info_callback(self, msg):
        """Process camera information"""
        self.get_logger().info(f"Camera info received: {msg.width}x{msg.height}")


def main(args=None):
    rclpy.init(args=args)

    perception_node = IsaacPerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()