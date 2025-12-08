#!/usr/bin/env python3

"""
Perception pipeline example for Module 3
Demonstrates object detection and tracking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # Initialize CV bridge and YOLO model
        self.bridge = CvBridge()

        try:
            # Load YOLO model (you may need to download it first)
            self.model = YOLO('yolov8n.pt')
        except:
            self.get_logger().warn("YOLO model not available, using mock detection")
            self.model = None

        # Subscribe to camera topics
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for detections
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        self.get_logger().info("Perception Pipeline initialized")

    def image_callback(self, msg):
        """Process incoming images for object detection"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.model:
                # Run YOLO detection
                results = self.model(cv_image)

                # Create Detection2DArray message
                detections_msg = Detection2DArray()
                detections_msg.header = msg.header

                for result in results:
                    boxes = result.boxes
                    if boxes is not None:
                        for box in boxes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            conf = float(box.conf[0].cpu().numpy())
                            cls = int(box.cls[0].cpu().numpy())

                            # Only include detections with confidence > 0.5
                            if conf > 0.5:
                                detection = self.create_detection(
                                    x1, y1, x2, y2, conf, cls
                                )
                                detections_msg.detections.append(detection)

                # Publish detections
                self.detections_pub.publish(detections_msg)
                self.get_logger().info(f"Published {len(detections_msg.detections)} detections")

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def create_detection(self, x1, y1, x2, y2, conf, cls):
        """Create a Detection2D message from bounding box coordinates"""
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

        detection = Detection2D()

        # Set bounding box
        detection.bbox.center.x = (x1 + x2) / 2.0
        detection.bbox.center.y = (y1 + y2) / 2.0
        detection.bbox.size_x = abs(x2 - x1)
        detection.bbox.size_y = abs(y2 - y1)

        # Set object hypothesis (classification)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(cls)  # Use class ID as string
        hypothesis.hypothesis.score = conf

        detection.results.append(hypothesis)

        return detection


def main(args=None):
    rclpy.init(args=args)

    perception_pipeline = PerceptionPipeline()

    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()