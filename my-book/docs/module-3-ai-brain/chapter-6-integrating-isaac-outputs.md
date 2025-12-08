---
title: "Integrating Isaac outputs with ROS2 controllers"
description: "Connecting Isaac Sim perception outputs to ROS2 control systems for humanoid robotics"
learning_objectives:
  - "Understand Isaac Sim output formats and ROS2 integration"
  - "Implement perception-to-control pipelines using Isaac ROS2 bridge"
  - "Create ROS2 controllers that utilize Isaac Sim outputs"
  - "Validate integrated perception-control systems"
---

# Integrating Isaac outputs with ROS2 controllers

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand Isaac Sim output formats and ROS2 integration
- Implement perception-to-control pipelines using Isaac ROS2 bridge
- Create ROS2 controllers that utilize Isaac Sim outputs
- Validate integrated perception-control systems

## Introduction

The integration of Isaac Sim perception outputs with ROS2 controllers is a critical component of modern humanoid robotics development. Isaac Sim provides high-fidelity simulation with realistic perception outputs (images, depth, segmentation, etc.), while ROS2 offers a mature ecosystem for robotics control and middleware. Connecting these two systems enables the development of perception-driven control systems that can be validated in simulation before deployment on real robots. This chapter will guide you through the technical aspects of integrating Isaac Sim outputs with ROS2-based control systems for humanoid robots.

## Understanding Isaac Sim Output Formats

### Isaac Sim Perception Outputs

Isaac Sim provides various types of perception outputs through its Omniverse platform:

1. **RGB Images**: High-quality color images from virtual cameras
2. **Depth Maps**: Per-pixel depth information
3. **Semantic Segmentation**: Pixel-wise semantic class labels
4. **Instance Segmentation**: Pixel-wise instance identifiers
5. **Bounding Boxes**: 2D and 3D bounding box annotations
6. **Point Clouds**: 3D point cloud data from virtual sensors
7. **Optical Flow**: Motion vectors between frames
8. **Camera Parameters**: Intrinsic and extrinsic camera properties

### Isaac ROS2 Bridge Architecture

The Isaac ROS2 bridge facilitates communication between Isaac Sim and ROS2 systems:

```python
# isaac_ros_integration.py - Isaac Sim to ROS2 bridge integration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
from vision_msgs.msg import Detection2DArray, BoundingBox2D
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class IsaacROSBridge(Node):
    """
    Bridge node that receives Isaac Sim outputs and publishes to ROS2 topics
    """
    def __init__(self):
        super().__init__('isaac_ros_bridge')

        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()

        # Isaac Sim output topics (these would be published by Isaac Sim)
        self.isaac_rgb_sub = self.create_subscription(
            Image,
            '/isaac_sim/rgb/image_raw',
            self.isaac_rgb_callback,
            10
        )

        self.isaac_depth_sub = self.create_subscription(
            Image,
            '/isaac_sim/depth/image_raw',
            self.isaac_depth_callback,
            10
        )

        self.isaac_segmentation_sub = self.create_subscription(
            Image,
            '/isaac_sim/semantic_segmentation',
            self.isaac_segmentation_callback,
            10
        )

        self.isaac_camera_info_sub = self.create_subscription(
            CameraInfo,
            '/isaac_sim/rgb/camera_info',
            self.isaac_camera_info_callback,
            10
        )

        self.isaac_detections_sub = self.create_subscription(
            Detection2DArray,
            '/isaac_sim/detections',
            self.isaac_detections_callback,
            10
        )

        # ROS2 output topics for controllers
        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/rgb/image_raw',
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image_raw',
            10
        )

        self.segmentation_pub = self.create_publisher(
            Image,
            '/camera/semantic_segmentation',
            10
        )

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/camera/detections',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/rgb/camera_info',
            10
        )

        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '/isaac_ros_bridge/status',
            10
        )

        # Store latest data
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_segmentation = None
        self.latest_camera_info = None
        self.latest_detections = None

        # Statistics
        self.message_count = 0
        self.last_message_time = self.get_clock().now()

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Isaac ROS Bridge initialized')

    def isaac_rgb_callback(self, msg):
        """Handle RGB image from Isaac Sim"""
        self.latest_rgb = msg
        self.message_count += 1

        # Republish to ROS2 standard topics
        self.rgb_pub.publish(msg)

        # Process image if needed
        self.process_rgb_image(msg)

    def isaac_depth_callback(self, msg):
        """Handle depth image from Isaac Sim"""
        self.latest_depth = msg
        self.depth_pub.publish(msg)

        # Process depth if needed
        self.process_depth_image(msg)

    def isaac_segmentation_callback(self, msg):
        """Handle segmentation image from Isaac Sim"""
        self.latest_segmentation = msg
        self.segmentation_pub.publish(msg)

        # Process segmentation if needed
        self.process_segmentation_image(msg)

    def isaac_camera_info_callback(self, msg):
        """Handle camera info from Isaac Sim"""
        self.latest_camera_info = msg
        self.camera_info_pub.publish(msg)

    def isaac_detections_callback(self, msg):
        """Handle object detections from Isaac Sim"""
        self.latest_detections = msg
        self.detections_pub.publish(msg)

        # Process detections if needed
        self.process_detections(msg)

    def process_rgb_image(self, msg):
        """Process RGB image from Isaac Sim"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Example processing: resize, normalize, etc.
            processed_image = self.preprocess_image(cv_image)

            # Could publish processed image to other topics
            # processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            # self.processed_image_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def process_depth_image(self, msg):
        """Process depth image from Isaac Sim"""
        try:
            # Convert to numpy array
            depth_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # Example: compute average depth in center region
            h, w = depth_array.shape
            center_region = depth_array[h//4:3*h//4, w//4:3*w//4]
            avg_depth = np.mean(center_region[np.isfinite(center_region)])

            self.get_logger().debug(f'Average depth in center: {avg_depth:.2f}m')

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def process_segmentation_image(self, msg):
        """Process segmentation image from Isaac Sim"""
        try:
            # Convert to numpy array
            seg_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # Example: count different semantic classes
            unique_classes, counts = np.unique(seg_array, return_counts=True)
            class_distribution = dict(zip(unique_classes, counts))

            self.get_logger().debug(f'Semantic class distribution: {class_distribution}')

        except Exception as e:
            self.get_logger().error(f'Error processing segmentation image: {e}')

    def process_detections(self, msg):
        """Process object detections from Isaac Sim"""
        try:
            # Process each detection
            for detection in msg.detections:
                # Could trigger control actions based on detections
                self.handle_detection(detection)

        except Exception as e:
            self.get_logger().error(f'Error processing detections: {e}')

    def preprocess_image(self, image):
        """Preprocess image for downstream processing"""
        # Example preprocessing pipeline
        # Resize image
        # height, width = image.shape[:2]
        # new_width = 640
        # new_height = int(height * new_width / width)
        # resized = cv2.resize(image, (new_width, new_height))

        # Normalize if needed
        # normalized = cv2.normalize(resized, None, 0, 255, cv2.NORM_MINMAX)

        return image  # Return original for now

    def handle_detection(self, detection):
        """Handle individual object detection"""
        # Example: trigger control based on detection
        if detection.results:
            best_result = max(detection.results, key=lambda x: x.hypothesis.score)
            class_id = best_result.hypothesis.class_id
            confidence = best_result.hypothesis.score

            if confidence > 0.8:  # High confidence detection
                self.get_logger().info(f'High confidence detection: {class_id} with confidence {confidence:.2f}')

    def publish_status(self):
        """Publish bridge status"""
        current_time = self.get_clock().now()
        status_msg = String()

        if self.message_count > 0:
            time_diff = (current_time - self.last_message_time).nanoseconds / 1e9
            if time_diff > 0:
                rate = self.message_count / time_diff
                status_msg.data = f'Active: {rate:.2f} Hz, Total: {self.message_count}'
            else:
                status_msg.data = f'Active: messages flowing, Total: {self.message_count}'
        else:
            status_msg.data = 'Waiting for Isaac Sim data'

        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = IsaacROSBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Perception-to-Control Pipelines

### Object Detection to Navigation

```python
# perception_to_control.py - Perception-driven control pipeline
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import List, Dict, Optional
from dataclasses import dataclass

@dataclass
class DetectedObject:
    """Data structure for detected objects"""
    class_id: str
    confidence: float
    bbox_center: tuple  # (x, y) in image coordinates
    bbox_size: tuple    # (width, height)
    distance: float     # Estimated distance from camera

class PerceptionToControlNode(Node):
    """
    Node that processes perception outputs and generates control commands
    """
    def __init__(self):
        super().__init__('perception_to_control')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscription to perception outputs
        self.detections_sub = self.create_subscription(
            Detection2DArray,
            '/camera/detections',
            self.detections_callback,
            10
        )

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Publishers for control commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/humanoid_robot/cmd_vel',
            10
        )

        self.navigation_goal_pub = self.create_publisher(
            PoseStamped,
            '/move_base_simple/goal',
            10
        )

        self.control_status_pub = self.create_publisher(
            String,
            '/perception_control/status',
            10
        )

        # Internal state
        self.latest_detections: List[DetectedObject] = []
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.camera_intrinsics = None

        # Control parameters
        self.avoidance_distance_threshold = 2.0  # meters
        self.target_following_distance = 1.0     # meters
        self.max_linear_speed = 0.5              # m/s
        self.max_angular_speed = 0.5             # rad/s

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Perception-to-Control node initialized')

    def detections_callback(self, msg):
        """Process object detections from Isaac Sim"""
        self.latest_detections = []

        for detection in msg.detections:
            if detection.results:  # Check if detection has results
                # Get best hypothesis
                best_result = max(detection.results, key=lambda x: x.hypothesis.score)

                # Create detected object
                detected_obj = DetectedObject(
                    class_id=best_result.hypothesis.class_id,
                    confidence=best_result.hypothesis.score,
                    bbox_center=(detection.bbox.center.x, detection.bbox.center.y),
                    bbox_size=(detection.bbox.size_x, detection.bbox.size_y),
                    distance=self.estimate_distance(detection)  # Will be 0 initially
                )

                # Only keep high-confidence detections
                if detected_obj.confidence > 0.7:
                    self.latest_detections.append(detected_obj)

    def rgb_callback(self, msg):
        """Store RGB image for processing"""
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')

    def depth_callback(self, msg):
        """Process depth image to estimate object distances"""
        try:
            depth_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # Update distance estimates for detected objects
            for obj in self.latest_detections:
                # Get depth at bounding box center
                center_x = int(obj.bbox_center[0])
                center_y = int(obj.bbox_center[1])

                if 0 <= center_y < depth_array.shape[0] and 0 <= center_x < depth_array.shape[1]:
                    depth_value = depth_array[center_y, center_x]
                    if np.isfinite(depth_value):
                        obj.distance = depth_value

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def estimate_distance(self, detection):
        """Estimate distance based on object size and known dimensions"""
        # This is a simplified approach
        # In practice, you'd use depth information or geometric reasoning
        # For now, return 0 and let depth_callback update it
        return 0.0

    def control_loop(self):
        """Main control loop that processes perceptions and generates commands"""
        if not self.latest_detections:
            # No detections, continue with default behavior or stop
            self.publish_stop_command()
            return

        # Analyze detections to determine control action
        control_cmd = self.analyze_detections_for_control()

        if control_cmd:
            self.cmd_vel_pub.publish(control_cmd)

            # Publish status
            status_msg = String()
            status_msg.data = f'Processed {len(self.latest_detections)} detections'
            self.control_status_pub.publish(status_msg)

    def analyze_detections_for_control(self) -> Optional[Twist]:
        """Analyze detections to generate appropriate control command"""
        # Priority: avoid obstacles, then follow targets, then explore
        avoidance_cmd = self.check_for_obstacle_avoidance()
        if avoidance_cmd:
            return avoidance_cmd

        target_following_cmd = self.check_for_target_following()
        if target_following_cmd:
            return target_following_cmd

        # Default behavior: continue current path or explore
        return self.get_default_behavior()

    def check_for_obstacle_avoidance(self) -> Optional[Twist]:
        """Check if any obstacles require avoidance maneuvers"""
        for obj in self.latest_detections:
            if obj.class_id in ['person', 'car', 'obstacle'] and obj.distance < self.avoidance_distance_threshold:
                # Obstacle detected, generate avoidance command
                cmd = Twist()

                # Simple avoidance: turn away from obstacle
                # Calculate position relative to image center
                image_center_x = 320  # Assuming 640x480 image
                obj_offset = obj.bbox_center[0] - image_center_x

                # Turn away from obstacle
                cmd.angular.z = -0.5 * np.sign(obj_offset)  # Turn away
                cmd.linear.x = 0.2  # Continue moving forward slowly

                self.get_logger().info(f'Avoiding obstacle: {obj.class_id} at {obj.distance:.2f}m')
                return cmd

        return None

    def check_for_target_following(self) -> Optional[Twist]:
        """Check if any targets should be followed"""
        for obj in self.latest_detections:
            if obj.class_id == 'target' and obj.distance > self.target_following_distance:
                # Target detected, generate following command
                cmd = Twist()

                # Move toward target
                image_center_x = 320  # Assuming 640x480 image
                obj_offset = obj.bbox_center[0] - image_center_x

                # Adjust heading toward target
                cmd.angular.z = -0.3 * (obj_offset / image_center_x)  # Proportional control

                # Move forward if target is far
                if obj.distance > self.target_following_distance + 0.5:
                    cmd.linear.x = min(self.max_linear_speed, 0.5 * (obj.distance - self.target_following_distance))

                self.get_logger().info(f'Following target: {obj.class_id} at {obj.distance:.2f}m')
                return cmd

        return None

    def get_default_behavior(self) -> Twist:
        """Default behavior when no specific actions are needed"""
        cmd = Twist()
        # Default: continue straight with slow speed
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0
        return cmd

    def publish_stop_command(self):
        """Publish zero velocity command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionToControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Semantic Segmentation to Navigation

```python
# segmentation_to_navigation.py - Using segmentation for navigation decisions
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Tuple

class SegmentationToNavigationNode(Node):
    """
    Node that uses semantic segmentation for navigation decisions
    """
    def __init__(self):
        super().__init__('segmentation_to_navigation')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Semantic class mappings
        self.semantic_classes = {
            0: 'background',
            1: 'floor',
            2: 'wall',
            3: 'obstacle',
            4: 'person',
            5: 'navigable',
            6: 'trap',
            7: 'goal'
        }

        # Subscription to segmentation
        self.segmentation_sub = self.create_subscription(
            Image,
            '/camera/semantic_segmentation',
            self.segmentation_callback,
            10
        )

        # Camera info for depth calculation
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Control command publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/humanoid_robot/cmd_vel',
            10
        )

        # Navigation goal publisher
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/move_base_simple/goal',
            10
        )

        # Visualization for debugging
        self.vis_pub = self.create_publisher(
            MarkerArray,
            '/segmentation_navigation/visualization',
            10
        )

        # Internal state
        self.latest_segmentation = None
        self.camera_info = None
        self.camera_matrix = None

        # Navigation parameters
        self.min_navigable_area = 5000  # pixels
        self.max_obstacle_ratio = 0.3   # 30% of image
        self.safe_path_width = 100      # pixels

        # Timer for processing
        self.process_timer = self.create_timer(0.2, self.process_segmentation)  # 5 Hz

        self.get_logger().info('Segmentation to Navigation node initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def segmentation_callback(self, msg):
        """Process semantic segmentation image"""
        try:
            # Convert to numpy array
            seg_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # Store for processing
            self.latest_segmentation = seg_array

        except Exception as e:
            self.get_logger().error(f'Error processing segmentation: {e}')

    def process_segmentation(self):
        """Process segmentation for navigation decisions"""
        if self.latest_segmentation is None:
            return

        # Analyze segmentation for navigable areas
        navigable_mask = (self.latest_segmentation == 5)  # navigable class
        obstacle_mask = (self.latest_segmentation == 3)   # obstacle class
        person_mask = (self.latest_segmentation == 4)     # person class

        # Calculate ratios
        total_pixels = self.latest_segmentation.size
        navigable_ratio = np.sum(navigable_mask) / total_pixels
        obstacle_ratio = np.sum(obstacle_mask) / total_pixels
        person_ratio = np.sum(person_mask) / total_pixels

        # Make navigation decision based on segmentation
        cmd = self.make_navigation_decision(
            navigable_mask, obstacle_mask, person_mask,
            navigable_ratio, obstacle_ratio, person_ratio
        )

        if cmd:
            self.cmd_vel_pub.publish(cmd)

        # Publish visualization
        self.publish_visualization(navigable_mask, obstacle_mask, person_mask)

    def make_navigation_decision(self, navigable_mask, obstacle_mask, person_mask,
                               navigable_ratio, obstacle_ratio, person_ratio) -> Twist:
        """Make navigation decision based on segmentation analysis"""
        cmd = Twist()

        # Check if there's enough navigable space
        if navigable_ratio < 0.1:  # Less than 10% is navigable
            # Stop or turn to find more navigable space
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3  # Turn slowly
            self.get_logger().info('Low navigable area, turning to find path')
            return cmd

        # Check for too many obstacles
        if obstacle_ratio > self.max_obstacle_ratio:
            # Obstacle avoidance
            cmd = self.avoid_obstacles(obstacle_mask)
            return cmd

        # Check for people (social navigation)
        if person_ratio > 0.05:  # More than 5% is people
            cmd = self.respect_personal_space(person_mask)
            return cmd

        # Normal navigation - follow navigable areas
        cmd = self.follow_navigable_path(navigable_mask)
        return cmd

    def avoid_obstacles(self, obstacle_mask):
        """Generate obstacle avoidance command"""
        cmd = Twist()

        # Find contours of obstacles
        contours, _ = cv2.findContours(
            obstacle_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            # Find largest obstacle
            largest_contour = max(contours, key=cv2.contourArea)

            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Calculate center of obstacle in image
            obstacle_center_x = x + w // 2
            image_center_x = obstacle_mask.shape[1] // 2

            # Turn away from obstacle
            cmd.angular.z = -0.4 * np.sign(obstacle_center_x - image_center_x)
            cmd.linear.x = 0.1  # Move slowly

        return cmd

    def respect_personal_space(self, person_mask):
        """Generate social navigation command respecting personal space"""
        cmd = Twist()

        # Find person regions
        contours, _ = cv2.findContours(
            person_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            # Calculate centroid of all person regions
            all_points = []
            for contour in contours:
                all_points.extend(contour.reshape(-1, 2))

            if all_points:
                all_points = np.array(all_points)
                centroid_x = np.mean(all_points[:, 0])

                image_center_x = person_mask.shape[1] // 2

                # Move away from people
                cmd.angular.z = -0.3 * np.sign(centroid_x - image_center_x)
                cmd.linear.x = 0.2  # Move at medium speed

        return cmd

    def follow_navigable_path(self, navigable_mask):
        """Generate command to follow navigable path"""
        cmd = Twist()

        # Find navigable path regions
        contours, _ = cv2.findContours(
            navigable_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            # Find the largest navigable area
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate moments to find centroid
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                image_center_x = navigable_mask.shape[1] // 2

                # Adjust heading toward navigable area center
                cmd.angular.z = -0.002 * (cx - image_center_x)  # Proportional control
                cmd.linear.x = 0.3  # Move forward

        return cmd

    def publish_visualization(self, navigable_mask, obstacle_mask, person_mask):
        """Publish visualization markers for debugging"""
        marker_array = MarkerArray()

        # Create markers for different regions
        if navigable_mask.any():
            navigable_marker = Marker()
            navigable_marker.header.frame_id = "camera_link"
            navigable_marker.header.stamp = self.get_clock().now().to_msg()
            navigable_marker.ns = "navigable"
            navigable_marker.id = 1
            navigable_marker.type = Marker.LINE_STRIP
            navigable_marker.action = Marker.ADD
            navigable_marker.pose.orientation.w = 1.0
            navigable_marker.scale.x = 0.02
            navigable_marker.color.r = 0.0
            navigable_marker.color.g = 1.0
            navigable_marker.color.b = 0.0
            navigable_marker.color.a = 0.6

            # Add points for navigable regions (simplified)
            # In practice, you'd extract contour points
            marker_array.markers.append(navigable_marker)

        if obstacle_mask.any():
            obstacle_marker = Marker()
            obstacle_marker.header = navigable_marker.header
            obstacle_marker.ns = "obstacles"
            obstacle_marker.id = 2
            obstacle_marker.type = Marker.LINE_STRIP
            obstacle_marker.action = Marker.ADD
            obstacle_marker.pose.orientation.w = 1.0
            obstacle_marker.scale.x = 0.02
            obstacle_marker.color.r = 1.0
            obstacle_marker.color.g = 0.0
            obstacle_marker.color.b = 0.0
            obstacle_marker.color.a = 0.6

            marker_array.markers.append(obstacle_marker)

        if person_mask.any():
            person_marker = Marker()
            person_marker.header = navigable_marker.header
            person_marker.ns = "people"
            person_marker.id = 3
            person_marker.type = Marker.LINE_STRIP
            person_marker.action = Marker.ADD
            person_marker.pose.orientation.w = 1.0
            person_marker.scale.x = 0.02
            person_marker.color.r = 1.0
            person_marker.color.g = 1.0
            person_marker.color.b = 0.0
            person_marker.color.a = 0.6

            marker_array.markers.append(person_marker)

        if marker_array.markers:
            self.vis_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationToNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS2 Controllers for Humanoid Robotics

### Perception-Based Controllers

```python
# perception_based_controllers.py - Controllers that use Isaac Sim perception outputs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, Imu
from geometry_msgs.msg import Twist, Pose, Point
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import math
from typing import List, Dict, Tuple

class PerceptionBasedBalancer(Node):
    """
    Balancer controller that uses perception outputs for balance
    """
    def __init__(self):
        super().__init__('perception_based_balancer')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.camera_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/humanoid_robot/joint_states',
            self.joint_state_callback,
            10
        )

        # Control command publisher
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/humanoid_robot/joint_commands',
            10
        )

        # Balance status publisher
        self.balance_status_pub = self.create_publisher(
            Float32,
            '/balance/stability_score',
            10
        )

        # Internal state
        self.imu_data = None
        self.joint_states = {}
        self.latest_image = None

        # Balance control parameters
        self.balance_kp = 2.0  # Proportional gain
        self.balance_kd = 0.5  # Derivative gain
        self.max_balance_torque = 50.0  # Nm

        # Timer for balance control
        self.balance_timer = self.create_timer(0.02, self.balance_control_loop)  # 50 Hz

        self.get_logger().info('Perception-Based Balancer initialized')

    def imu_callback(self, msg):
        """Store IMU data for balance control"""
        self.imu_data = {
            'orientation': (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            'angular_velocity': (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            'linear_acceleration': (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        }

    def camera_callback(self, msg):
        """Process camera image for visual feedback"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def joint_state_callback(self, msg):
        """Store joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def balance_control_loop(self):
        """Main balance control loop using perception data"""
        if not self.imu_data:
            return

        # Calculate current orientation error
        orientation_error = self.calculate_orientation_error()

        # Calculate angular velocity for damping
        angular_vel = self.imu_data['angular_velocity']
        angular_speed = math.sqrt(angular_vel[0]**2 + angular_vel[1]**2 + angular_vel[2]**2)

        # Calculate balance correction
        balance_correction = self.balance_kp * orientation_error - self.balance_kd * angular_speed

        # Apply balance correction to joints
        balance_commands = self.calculate_balance_joint_commands(balance_correction)

        # Publish joint commands
        self.publish_joint_commands(balance_commands)

        # Publish stability score
        stability_score = Float32()
        stability_score.data = max(0.0, 1.0 - abs(orientation_error))  # 1.0 = perfectly balanced
        self.balance_status_pub.publish(stability_score)

    def calculate_orientation_error(self):
        """Calculate orientation error from IMU data"""
        # Convert quaternion to roll/pitch
        x, y, z, w = self.imu_data['orientation']

        # Calculate roll and pitch (simplified)
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))

        # For balance, we want to maintain upright position (roll=0, pitch=0)
        # Return combined error
        orientation_error = math.sqrt(roll**2 + pitch**2)
        return orientation_error

    def calculate_balance_joint_commands(self, balance_correction):
        """Calculate joint commands for balance correction"""
        # This is a simplified approach
        # In practice, you'd use inverse kinematics or whole-body control

        commands = JointState()
        commands.header.stamp = self.get_clock().now().to_msg()
        commands.name = []
        commands.position = []
        commands.velocity = []
        commands.effort = []

        # Define which joints to use for balance (typically ankle, hip, torso)
        balance_joints = [
            'left_ankle_pitch', 'left_ankle_roll',
            'right_ankle_pitch', 'right_ankle_roll',
            'left_hip_pitch', 'right_hip_pitch',
            'torso_yaw', 'torso_pitch'
        ]

        # Calculate correction for each balance joint
        for joint_name in balance_joints:
            if joint_name in self.joint_states:
                current_pos = self.joint_states[joint_name]['position']

                # Apply balance correction (scaled appropriately)
                corrected_pos = current_pos - 0.1 * balance_correction

                commands.name.append(joint_name)
                commands.position.append(corrected_pos)
                commands.velocity.append(0.0)  # No specific velocity target
                commands.effort.append(0.0)    # Effort will be computed by controller

        return commands

    def publish_joint_commands(self, commands):
        """Publish joint commands to robot"""
        self.joint_cmd_pub.publish(commands)

class PerceptionGuidedWalker(Node):
    """
    Walking controller guided by perception outputs
    """
    def __init__(self):
        super().__init__('perception_guided_walker')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions
        self.segmentation_sub = self.create_subscription(
            Image,
            '/camera/semantic_segmentation',
            self.segmentation_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/humanoid_robot/joint_states',
            self.joint_state_callback,
            10
        )

        # Control publishers
        self.step_cmd_pub = self.create_publisher(
            Twist,
            '/humanoid_robot/step_command',
            10
        )

        self.gait_param_pub = self.create_publisher(
            JointState,
            '/humanoid_robot/gait_parameters',
            10
        )

        # Internal state
        self.latest_segmentation = None
        self.latest_depth = None
        self.joint_states = {}

        # Walking parameters
        self.step_length = 0.3  # meters
        self.step_height = 0.1  # meters
        self.step_duration = 0.8  # seconds

        # Timer for walking control
        self.walk_timer = self.create_timer(0.1, self.walking_control_loop)  # 10 Hz

        self.get_logger().info('Perception-Guided Walker initialized')

    def segmentation_callback(self, msg):
        """Process segmentation for terrain analysis"""
        try:
            seg_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.latest_segmentation = seg_array
        except Exception as e:
            self.get_logger().error(f'Error processing segmentation: {e}')

    def depth_callback(self, msg):
        """Process depth for step planning"""
        try:
            depth_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.latest_depth = depth_array
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')

    def joint_state_callback(self, msg):
        """Store joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def walking_control_loop(self):
        """Main walking control loop with perception guidance"""
        if self.latest_segmentation is None or self.latest_depth is None:
            return

        # Analyze terrain from segmentation
        terrain_analysis = self.analyze_terrain(self.latest_segmentation)

        # Plan footsteps based on terrain and depth
        step_plan = self.plan_steps(terrain_analysis, self.latest_depth)

        # Generate walking commands
        walk_cmd = self.generate_walk_command(step_plan)

        # Publish walking command
        self.step_cmd_pub.publish(walk_cmd)

        # Adjust gait parameters based on terrain
        gait_params = self.adjust_gait_for_terrain(terrain_analysis)
        self.gait_param_pub.publish(gait_params)

    def analyze_terrain(self, segmentation):
        """Analyze terrain from semantic segmentation"""
        # Identify different terrain types
        floor_mask = (segmentation == 1)  # floor class
        obstacle_mask = (segmentation == 3)  # obstacle class
        stairs_mask = (segmentation == 8)  # stairs class (if defined)
        rough_mask = (segmentation == 9)  # rough terrain class (if defined)

        terrain_analysis = {
            'floor_ratio': np.sum(floor_mask) / segmentation.size,
            'obstacle_ratio': np.sum(obstacle_mask) / segmentation.size,
            'has_stairs': np.any(stairs_mask),
            'roughness': np.sum(rough_mask) / segmentation.size
        }

        return terrain_analysis

    def plan_steps(self, terrain_analysis, depth_image):
        """Plan footsteps based on terrain analysis and depth"""
        # This is a simplified step planning approach
        # In practice, this would use more sophisticated path planning

        step_plan = {
            'step_length': self.step_length,
            'step_height': self.step_height,
            'step_direction': (1, 0),  # Move forward
            'foot_placement': 'center'  # Place foot in center of navigable area
        }

        # Adjust step parameters based on terrain
        if terrain_analysis['obstacle_ratio'] > 0.2:
            # Reduce step length for obstacle avoidance
            step_plan['step_length'] *= 0.7
        elif terrain_analysis['roughness'] > 0.1:
            # Increase step height for rough terrain
            step_plan['step_height'] *= 1.5

        return step_plan

    def generate_walk_command(self, step_plan):
        """Generate walking command from step plan"""
        cmd = Twist()

        # Set linear velocity based on planned step
        cmd.linear.x = step_plan['step_length'] / self.step_duration
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        # Set angular velocity for turning if needed
        # This would be calculated based on desired direction vs current heading
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        return cmd

    def adjust_gait_for_terrain(self, terrain_analysis):
        """Adjust gait parameters based on terrain"""
        gait_params = JointState()
        gait_params.header.stamp = self.get_clock().now().to_msg()
        gait_params.name = []
        gait_params.position = []
        gait_params.velocity = []
        gait_params.effort = []

        # Adjust gait based on terrain type
        if terrain_analysis['roughness'] > 0.1:
            # More cautious gait for rough terrain
            gait_params.name.extend(['stance_duration', 'swing_duration', 'step_height'])
            gait_params.position.extend([0.6, 0.4, 0.15])  # Slower, higher steps
        elif terrain_analysis['has_stairs']:
            # Special gait for stairs
            gait_params.name.extend(['stance_duration', 'swing_duration', 'step_height'])
            gait_params.position.extend([0.5, 0.5, 0.2])  # Shorter, higher steps
        else:
            # Normal gait
            gait_params.name.extend(['stance_duration', 'swing_duration', 'step_height'])
            gait_params.position.extend([0.4, 0.4, 0.1])  # Regular steps

        return gait_params

def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    balancer = PerceptionBasedBalancer()
    walker = PerceptionGuidedWalker()

    # Use multi-threaded executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(balancer)
    executor.add_node(walker)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        balancer.destroy_node()
        walker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Isaac Sim Configuration for ROS2 Integration

### Isaac Sim Extension Setup

```python
# isaac_sim_ros2_extension.py - Isaac Sim extension for ROS2 integration
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.carb import carb_settings_update
from omni.isaac.sensor import Camera, LidarRtx
import numpy as np

class IsaacSimROS2Extension:
    """
    Extension to set up Isaac Sim for ROS2 integration
    """
    def __init__(self):
        self.world = None
        self.robot = None
        self.cameras = []
        self.lidars = []

    def setup_world(self):
        """Set up the Isaac Sim world with ROS2-compatible configuration"""
        # Initialize world
        self.world = World(stage_units_in_meters=1.0)

        # Enable physics
        self.world.scene.enable_physics()

        # Set up default ground plane
        self.world.scene.add_default_ground_plane()

        # Configure physics settings for humanoid simulation
        carb_settings_update("/physics_solver_type", "TGS")
        carb_settings_update("/physics_solver_iterations", 8)
        carb_settings_update("/physics_solver_velocity_iterations", 1)
        carb_settings_update("/physics_solver_position_iterations", 4)

        # Set gravity
        carb_settings_update("/physics/ground_plane_gravity", -9.81)

        self.get_logger().info("Isaac Sim world configured for ROS2 integration")

    def add_humanoid_robot(self, robot_usd_path, position=(0, 0, 1.0)):
        """Add a humanoid robot to the simulation"""
        try:
            # Add robot to stage
            add_reference_to_stage(
                usd_path=robot_usd_path,
                prim_path="/World/HumanoidRobot"
            )

            # Reset world to load robot
            self.world.reset()

            # Get robot as an Articulation object
            self.robot = self.world.scene.get_object("HumanoidRobot")

            if self.robot:
                # Set initial position
                self.robot.set_world_poses(positions=np.array([position]), orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))

                self.get_logger().info(f"Humanoid robot added at position {position}")
            else:
                self.get_logger().error("Failed to get robot as Articulation object")

        except Exception as e:
            self.get_logger().error(f"Error adding humanoid robot: {e}")

    def add_sensors_to_robot(self):
        """Add sensors to the robot for ROS2 integration"""
        if not self.robot:
            self.get_logger().error("No robot available to add sensors to")
            return

        # Add RGB camera to head
        try:
            camera = Camera(
                prim_path="/World/HumanoidRobot/Head/Camera",
                frequency=30,
                resolution=(640, 480)
            )
            camera.set_focal_length(24.0)
            camera.set_horizontal_aperture(20.955)
            camera.set_vertical_aperture(15.2908)
            self.cameras.append(camera)
            self.get_logger().info("RGB camera added to robot head")
        except Exception as e:
            self.get_logger().error(f"Error adding RGB camera: {e}")

        # Add depth camera
        try:
            depth_camera = Camera(
                prim_path="/World/HumanoidRobot/Head/DepthCamera",
                frequency=30,
                resolution=(640, 480)
            )
            depth_camera.set_focal_length(24.0)
            depth_camera.set_horizontal_aperture(20.955)
            depth_camera.set_vertical_aperture(15.2908)
            # Enable depth data
            from omni.replicator.core import _core as replicator
            rep = replicator.get_replicator()
            rep.get()  # Initialize replicator
            self.cameras.append(depth_camera)
            self.get_logger().info("Depth camera added to robot head")
        except Exception as e:
            self.get_logger().error(f"Error adding depth camera: {e}")

        # Add semantic segmentation camera
        try:
            seg_camera = Camera(
                prim_path="/World/HumanoidRobot/Head/SegCamera",
                frequency=30,
                resolution=(640, 480)
            )
            seg_camera.set_focal_length(24.0)
            seg_camera.set_horizontal_aperture(20.955)
            seg_camera.set_vertical_aperture(15.2908)
            self.cameras.append(seg_camera)
            self.get_logger().info("Segmentation camera added to robot head")
        except Exception as e:
            self.get_logger().error(f"Error adding segmentation camera: {e}")

        # Add IMU
        try:
            # IMU is typically added through semantic schemas in Isaac Sim
            head_prim = get_prim_at_path("/World/HumanoidRobot/Head")
            from omni.isaac.core.utils.semantics import add_semantics
            add_semantics(head_prim, "sensor", "imu")
            self.get_logger().info("IMU semantic schema added to robot head")
        except Exception as e:
            self.get_logger().error(f"Error adding IMU: {e}")

        # Add LiDAR
        try:
            lidar = LidarRtx(
                prim_path="/World/HumanoidRobot/Torso/Lidar",
                translation=np.array([0.0, 0.0, 0.5]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                Hz=10,
                points=720,
                channels=16,
                max_range=25.0,
                drift=True,
                visualize=False,
                add_to_stage=True
            )
            self.lidars.append(lidar)
            self.get_logger().info("LiDAR added to robot torso")
        except Exception as e:
            self.get_logger().error(f"Error adding LiDAR: {e}")

    def run_simulation(self):
        """Run the simulation loop"""
        self.world.reset()

        while True:
            # Step the world
            self.world.step(render=True)

            # Process sensor data
            self.process_sensor_data()

            # Check for termination conditions
            if self.should_terminate():
                break

    def process_sensor_data(self):
        """Process sensor data for ROS2 publishing"""
        # This would interface with ROS2 bridge to publish sensor data
        # In practice, you'd have a ROS2 bridge running alongside Isaac Sim
        pass

    def should_terminate(self):
        """Check if simulation should terminate"""
        # Implement termination conditions
        return False

    def get_logger(self):
        """Get logger for the extension"""
        import carb
        return carb.Logger.acquire_default_logger()

def setup_isaac_sim_for_ros2():
    """Main function to set up Isaac Sim for ROS2 integration"""
    extension = IsaacSimROS2Extension()

    # Set up world
    extension.setup_world()

    # Add robot (replace with your robot USD path)
    robot_path = "/path/to/humanoid_robot.usd"  # Replace with actual path
    extension.add_humanoid_robot(robot_path)

    # Add sensors
    extension.add_sensors_to_robot()

    # Run simulation
    extension.run_simulation()

if __name__ == "__main__":
    setup_isaac_sim_for_ros2()
```

## Validation and Testing

### Integration Validation Framework

```python
# validation_framework.py - Validation framework for Isaac Sim to ROS2 integration
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import numpy as np
from cv_bridge import CvBridge

class IsaacSimROS2IntegrationValidator(Node):
    """
    Validation framework for Isaac Sim to ROS2 integration
    """
    def __init__(self):
        super().__init__('integration_validator')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions to validate
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.seg_sub = self.create_subscription(
            Image, '/camera/semantic_segmentation', self.seg_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.info_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/humanoid_robot/cmd_vel', self.cmd_callback, 10
        )

        # Validation results
        self.validation_results = {
            'rgb_received': False,
            'depth_received': False,
            'seg_received': False,
            'info_received': False,
            'cmd_received': False,
            'data_quality': {},
            'timing_stats': {}
        }

        # Timing tracking
        self.message_times = {
            'rgb': [],
            'depth': [],
            'seg': [],
            'info': [],
            'cmd': []
        }

        self.get_logger().info('Integration validator initialized')

    def rgb_callback(self, msg):
        """Validate RGB image quality and timing"""
        self.validation_results['rgb_received'] = True

        # Store timing
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.message_times['rgb'].append(current_time)

        # Validate image quality
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            quality_stats = {
                'height': cv_image.shape[0],
                'width': cv_image.shape[1],
                'mean_intensity': np.mean(cv_image),
                'std_intensity': np.std(cv_image),
                'valid': True
            }
            self.validation_results['data_quality']['rgb'] = quality_stats
        except Exception as e:
            self.validation_results['data_quality']['rgb'] = {'valid': False, 'error': str(e)}

    def depth_callback(self, msg):
        """Validate depth image quality and timing"""
        self.validation_results['depth_received'] = True

        current_time = self.get_clock().now().nanoseconds / 1e9
        self.message_times['depth'].append(current_time)

        try:
            depth_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            quality_stats = {
                'height': depth_array.shape[0],
                'width': depth_array.shape[1],
                'mean_depth': np.mean(depth_array[np.isfinite(depth_array)]),
                'min_depth': np.min(depth_array[np.isfinite(depth_array)]),
                'max_depth': np.max(depth_array[np.isfinite(depth_array)]),
                'valid': True
            }
            self.validation_results['data_quality']['depth'] = quality_stats
        except Exception as e:
            self.validation_results['data_quality']['depth'] = {'valid': False, 'error': str(e)}

    def seg_callback(self, msg):
        """Validate segmentation image quality and timing"""
        self.validation_results['seg_received'] = True

        current_time = self.get_clock().now().nanoseconds / 1e9
        self.message_times['seg'].append(current_time)

        try:
            seg_array = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            unique_vals, counts = np.unique(seg_array, return_counts=True)
            quality_stats = {
                'height': seg_array.shape[0],
                'width': seg_array.shape[1],
                'unique_classes': len(unique_vals),
                'class_distribution': dict(zip(unique_vals, counts)),
                'valid': True
            }
            self.validation_results['data_quality']['seg'] = quality_stats
        except Exception as e:
            self.validation_results['data_quality']['seg'] = {'valid': False, 'error': str(e)}

    def info_callback(self, msg):
        """Validate camera info"""
        self.validation_results['info_received'] = True

        current_time = self.get_clock().now().nanoseconds / 1e9
        self.message_times['info'].append(current_time)

        quality_stats = {
            'width': msg.width,
            'height': msg.height,
            'k_matrix': list(msg.k),
            'p_matrix': list(msg.p),
            'valid': True
        }
        self.validation_results['data_quality']['info'] = quality_stats

    def cmd_callback(self, msg):
        """Validate command reception"""
        self.validation_results['cmd_received'] = True

        current_time = self.get_clock().now().nanoseconds / 1e9
        self.message_times['cmd'].append(current_time)

        quality_stats = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z,
            'valid': True
        }
        self.validation_results['data_quality']['cmd'] = quality_stats

    def calculate_timing_stats(self):
        """Calculate timing statistics"""
        for sensor_type, times in self.message_times.items():
            if len(times) >= 2:
                time_diffs = np.diff(times)
                self.validation_results['timing_stats'][sensor_type] = {
                    'mean_dt': np.mean(time_diffs),
                    'std_dt': np.std(time_diffs),
                    'min_dt': np.min(time_diffs),
                    'max_dt': np.max(time_diffs),
                    'expected_rate': 1.0 / np.mean(time_diffs) if np.mean(time_diffs) > 0 else 0
                }

    def validate_integration(self, timeout=30.0):
        """Perform complete integration validation"""
        start_time = time.time()

        # Wait for messages or timeout
        while time.time() - start_time < timeout:
            if all([
                self.validation_results['rgb_received'],
                self.validation_results['depth_received'],
                self.validation_results['seg_received'],
                self.validation_results['info_received']
            ]):
                break
            time.sleep(0.1)

        # Calculate timing stats
        self.calculate_timing_stats()

        # Generate validation report
        report = self.generate_validation_report()
        return report

    def generate_validation_report(self):
        """Generate comprehensive validation report"""
        report = {
            'summary': {
                'rgb_received': self.validation_results['rgb_received'],
                'depth_received': self.validation_results['depth_received'],
                'seg_received': self.validation_results['seg_received'],
                'info_received': self.validation_results['info_received'],
                'cmd_received': self.validation_results['cmd_received']
            },
            'data_quality': self.validation_results['data_quality'],
            'timing_stats': self.validation_results['timing_stats'],
            'overall_score': self.calculate_overall_score()
        }

        return report

    def calculate_overall_score(self):
        """Calculate overall integration quality score"""
        score = 0
        max_score = 0

        # Data reception (binary)
        for key in ['rgb_received', 'depth_received', 'seg_received', 'info_received']:
            if self.validation_results[key]:
                score += 1
            max_score += 1

        # Data quality (if available)
        for sensor, quality in self.validation_results['data_quality'].items():
            if quality.get('valid', False):
                score += 1
                max_score += 1

        # Timing (if available)
        for sensor, timing in self.validation_results['timing_stats'].items():
            if timing.get('expected_rate', 0) > 10:  # Expect > 10 Hz
                score += 0.5
                max_score += 0.5

        return score / max_score if max_score > 0 else 0.0

def run_integration_validation():
    """Run the integration validation"""
    rclpy.init()
    validator = IsaacSimROS2IntegrationValidator()

    print("Starting Isaac Sim to ROS2 integration validation...")
    print("Ensure Isaac Sim is publishing to the expected topics.")

    # Wait for some data
    time.sleep(5.0)

    # Run validation
    report = validator.validate_integration()

    # Print results
    print("\n=== INTEGRATION VALIDATION REPORT ===")
    print(f"Overall Score: {report['overall_score']:.2f}")
    print(f"Data Reception: {report['summary']}")
    print(f"Timing Stats: {report['timing_stats']}")

    rclpy.shutdown()
    return report

if __name__ == '__main__':
    run_integration_validation()
```

## Practical Exercise: Complete Integration Pipeline

Create a complete integration pipeline:

1. **Set up the Isaac Sim environment**:

```python
# complete_integration_example.py - Complete Isaac Sim to ROS2 integration example
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

class CompleteIntegrationExample:
    def __init__(self):
        # Initialize Isaac Sim
        self.world = World(stage_units_in_meters=1.0)
        self.robot = None

        # Initialize ROS2
        rclpy.init()
        self.ros_node = IntegrationBridgeNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)

    def setup_simulation(self):
        """Set up the complete simulation environment"""
        # Enable physics
        self.world.scene.enable_physics()

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add robot (using a sample robot for demonstration)
        assets_root_path = get_assets_root_path()
        if assets_root_path:
            robot_asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
            add_reference_to_stage(usd_path=robot_asset_path, prim_path="/World/Robot")
        else:
            print("Nucleus asset path unavailable, using simple cube as robot")
            from omni.isaac.core.objects import DynamicCuboid
            self.world.scene.add(
                DynamicCuboid(
                    prim_path="/World/Robot",
                    name="simple_robot",
                    position=np.array([0, 0, 0.5]),
                    size=0.5,
                    color=np.array([0.5, 0.5, 0.5])
                )
            )

        # Reset world
        self.world.reset()

    def run_integration_loop(self):
        """Run the main integration loop"""
        try:
            while True:
                # Step Isaac Sim
                self.world.step(render=True)

                # Spin ROS2 to process callbacks
                self.executor.spin_once(timeout_sec=0.01)

                # Simulate perception outputs from Isaac Sim
                self.simulate_perception_outputs()

        except KeyboardInterrupt:
            print("Integration loop interrupted")

    def simulate_perception_outputs(self):
        """Simulate perception outputs that would normally come from Isaac Sim"""
        # This would normally be handled by Isaac Sim's ROS2 bridge
        # For this example, we'll simulate the data
        pass

    def cleanup(self):
        """Clean up resources"""
        self.ros_node.destroy_node()
        rclpy.shutdown()

class IntegrationBridgeNode(Node):
    """Node that bridges Isaac Sim outputs to ROS2 controllers"""
    def __init__(self):
        super().__init__('integration_bridge')

        # Publishers for Isaac Sim outputs
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)

        # Subscribers for control commands
        self.cmd_sub = self.create_subscription(
            Twist, '/humanoid_robot/cmd_vel', self.cmd_callback, 10
        )

        # Timer for publishing simulated data
        self.pub_timer = self.create_timer(0.1, self.publish_simulated_data)

        # CV Bridge
        self.bridge = CvBridge()

        self.get_logger().info('Integration Bridge Node initialized')

    def cmd_callback(self, msg):
        """Handle control commands from ROS2 controllers"""
        self.get_logger().info(f'Received command: linear={msg.linear}, angular={msg.angular}')

    def publish_simulated_data(self):
        """Publish simulated perception data"""
        # Simulate RGB image
        rgb_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.header.frame_id = 'camera_rgb_optical_frame'
        self.rgb_pub.publish(rgb_msg)

        # Simulate depth image
        depth_image = np.random.uniform(0.1, 10.0, (480, 640)).astype(np.float32)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '32FC1')
        depth_msg.header = rgb_msg.header
        self.depth_pub.publish(depth_msg)

        # Simulate camera info
        info_msg = CameraInfo()
        info_msg.header = rgb_msg.header
        info_msg.width = 640
        info_msg.height = 480
        info_msg.k = [554.256, 0.0, 320.5, 0.0, 554.256, 240.5, 0.0, 0.0, 1.0]  # Approximate Kinect values
        info_msg.p = [554.256, 0.0, 320.5, 0.0, 0.0, 554.256, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info_msg)

def main():
    """Main function to run the complete integration example"""
    integration = CompleteIntegrationExample()

    try:
        integration.setup_simulation()
        integration.run_integration_loop()
    except Exception as e:
        print(f"Error in integration: {e}")
    finally:
        integration.cleanup()

if __name__ == '__main__':
    main()
```

2. **Launch the complete system**:

```python
# launch/integration_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # Isaac ROS Bridge node
    isaac_ros_bridge = Node(
        package='my_humanoid_integration',
        executable='isaac_ros_bridge',
        name='isaac_ros_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/isaac_sim/rgb/image_raw', '/camera/rgb/image_raw'),
            ('/isaac_sim/depth/image_raw', '/camera/depth/image_raw'),
        ]
    )

    # Perception-to-control node
    perception_to_control = Node(
        package='my_humanoid_integration',
        executable='perception_to_control',
        name='perception_to_control',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Segmentation-to-navigation node
    seg_to_nav = Node(
        package='my_humanoid_integration',
        executable='segmentation_to_navigation',
        name='segmentation_to_navigation',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controllers
    balancer = Node(
        package='my_humanoid_integration',
        executable='perception_based_balancer',
        name='perception_based_balancer',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    walker = Node(
        package='my_humanoid_integration',
        executable='perception_guided_walker',
        name='perception_guided_walker',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        isaac_ros_bridge,
        perception_to_control,
        seg_to_nav,
        balancer,
        walker
    ])
```

## Troubleshooting Common Integration Issues

### Topic Mapping Issues
- **Mismatched topic names**: Ensure Isaac Sim outputs match ROS2 controller expectations
- **Message type mismatches**: Verify message types are compatible between systems
- **Frame ID inconsistencies**: Ensure TF frames are properly configured

### Performance Issues
- **High latency**: Optimize sensor publishing rates and network configuration
- **Low bandwidth**: Reduce image resolution or compression settings
- **CPU/GPU overload**: Adjust simulation fidelity and processing rates

### Synchronization Problems
- **Timestamp mismatches**: Ensure both systems use synchronized clocks
- **Buffer overflows**: Implement proper queuing and buffering strategies
- **Data loss**: Add reliability mechanisms for critical data streams

## Summary

In this chapter, we've explored the integration of Isaac Sim outputs with ROS2 controllers for humanoid robotics. We covered understanding Isaac Sim output formats, implementing perception-to-control pipelines, creating ROS2 controllers that utilize Isaac Sim outputs, and validating the integrated systems. The integration between Isaac Sim's high-fidelity simulation and ROS2's control ecosystem enables the development of sophisticated perception-driven control systems that can be validated in simulation before deployment on real robots.

## Next Steps

- Set up Isaac Sim with your specific humanoid robot model
- Configure the ROS2 bridge for your sensor setup
- Implement perception-driven controllers for your specific tasks
- Validate the integration using the framework provided
- Test and refine the perception-control pipeline