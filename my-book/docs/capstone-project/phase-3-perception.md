---
sidebar_position: 4
---

# Phase 3: Perception System with NVIDIA Isaac

## Objectives

In this phase, you will:
- Integrate NVIDIA Isaac ROS perception capabilities
- Implement computer vision algorithms for environment understanding
- Process camera data for object detection and scene analysis
- Integrate sensor fusion for comprehensive environment perception

## Perception System Architecture

The perception system will include:

1. **Camera Processing Pipeline**: RGB and depth image processing
2. **Object Detection**: Identifying objects in the environment
3. **Semantic Segmentation**: Understanding scene composition
4. **SLAM Integration**: Simultaneous Localization and Mapping
5. **Sensor Fusion**: Combining multiple sensor inputs

## Implementation Steps

### Step 1: Install NVIDIA Isaac ROS Packages

First, install the necessary Isaac ROS packages:

```bash
# Install Isaac ROS common packages
sudo apt install ros-humble-isaac-ros-common

# Install Isaac ROS perception packages
sudo apt install ros-humble-isaac-ros-pointcloud-utils
sudo apt install ros-humble-isaac-ros-stereo-image-pipeline
sudo apt install ros-humble-isaac-ros-visual-slam

# Install Isaac ROS detection packages
sudo apt install ros-humble-isaac-ros-dnn-interfaces
sudo apt install ros-humble-isaac-ros-detection-postprocessor
```

### Step 2: Configure Camera and Perception Nodes

Create a perception configuration file (`config/perception.yaml`):

```yaml
camera_node:
  ros__parameters:
    # Camera parameters
    width: 640
    height: 480
    fps: 30
    
    # Calibration parameters
    camera_matrix: [616.17, 0.0, 316.30, 0.0, 616.17, 217.0, 0.0, 0.0, 1.0]
    distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]

object_detection_node:
  ros__parameters:
    # Model parameters
    model_path: "/path/to/detection/model"
    confidence_threshold: 0.5
    nms_threshold: 0.4
    
    # Topic parameters
    image_topic: "/camera/rgb/image_raw"
    detection_topic: "/object_detections"

slam_node:
  ros__parameters:
    # SLAM parameters
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    
    # Optimization parameters
    max_keyframes: 100
    min_translation: 0.1
    min_rotation: 0.2
```

### Step 3: Create Perception Launch File

Create a launch file for the perception system (`launch/perception.launch.py`):

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Set parameters
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'camera_link',
            'enable_slam_2d': True,
            'enable_rectified_pose': True,
            'force_default_publisher_on': True,
            'supported_image_types': ['rgb8', 'bgr8'],
        }],
        remappings=[
            ('/visual_slam/image', '/camera/rgb/image_raw'),
            ('/visual_slam/camera_info', '/camera/rgb/camera_info'),
            ('/visual_slam/imu', '/imu/data'),
        ],
        output='screen'
    )

    # Depth image processing node
    depth_proc_node = Node(
        package='nodelet',
        executable='nodelet',
        name='nodelet_manager',
        arguments=['manager'],
        output='screen'
    )

    depth_image_proc = Node(
        package='nodelet',
        executable='nodelet',
        name='depth_image_proc',
        arguments=['load', 'depth_image_proc/point_cloud_xyzrgb', 'nodelet_manager'],
        remappings=[
            ('rgb/image_rect_color', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth_registered/image_rect', '/camera/depth/image_raw'),
            ('depth_registered/points', '/camera/depth_registered/points'),
        ],
        output='screen'
    )

    # Isaac ROS object detection node
    detection_node = Node(
        package='isaac_ros_detectnet',
        executable='isaac_ros_detectnet',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': 'ssd_mobilenet_v2_coco',
            'confidence_threshold': 0.7,
            'input_image_width': 640,
            'input_image_height': 480,
            'enable_padding': True,
        }],
        remappings=[
            ('/image', '/camera/rgb/image_raw'),
            ('/camera_info', '/camera/rgb/camera_info'),
        ],
        output='screen'
    )

    # Perception processing node
    perception_node = Node(
        package='humanoid_perception',
        executable='perception_processor',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/rgb/image_raw'),
            ('/detections', '/detectnet/detections'),
            ('/slam_pose', '/visual_slam/pose'),
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=use_sim_time),
        depth_proc_node,
        visual_slam_node,
        depth_image_proc,
        detection_node,
        perception_node,
    ])
```

### Step 4: Implement Perception Processing Node

Create a perception processing node (`src/perception_processor.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import message_filters
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs


class PerceptionProcessor(Node):
    def __init__(self):
        super().__init__('perception_processor')
        
        # Create CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Initialize data buffers
        self.latest_image = None
        self.latest_detections = None
        self.latest_slam_pose = None
        self.latest_pointcloud = None
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )
        
        self.slam_pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.slam_pose_callback,
            10
        )
        
        # Publishers for processed data
        self.processed_detections_pub = self.create_publisher(
            Detection2DArray,
            '/processed_detections',
            10
        )
        
        self.obstacle_map_pub = self.create_publisher(
            Image,
            '/obstacle_map',
            10
        )
        
        # Timer for processing loop
        self.process_timer = self.create_timer(0.1, self.process_perception_data)
        
        self.get_logger().info('Perception processor initialized')

    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def detection_callback(self, msg):
        """Handle incoming object detections"""
        self.latest_detections = msg

    def slam_pose_callback(self, msg):
        """Handle SLAM pose updates"""
        self.latest_slam_pose = msg

    def process_perception_data(self):
        """Main processing loop for perception data"""
        if self.latest_image is not None:
            # Process the current image with any available detections
            processed_image = self.annotate_image_with_detections(
                self.latest_image.copy(),
                self.latest_detections
            )
            
            # Convert back to ROS image and publish
            try:
                annotated_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
                annotated_msg.header.stamp = self.get_clock().now().to_msg()
                annotated_msg.header.frame_id = 'camera_link'
                self.obstacle_map_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Error converting processed image: {e}')

        # Process detections if available
        if self.latest_detections is not None:
            # Apply any additional processing to detections
            processed_detections = self.process_detections(self.latest_detections)
            self.processed_detections_pub.publish(processed_detections)

    def annotate_image_with_detections(self, image, detections):
        """Annotate image with bounding boxes and labels"""
        if detections is None:
            return image
        
        for detection in detections.detections:
            # Get bounding box coordinates
            bbox = detection.bbox
            x = int(bbox.center.position.x - bbox.size_x / 2)
            y = int(bbox.center.position.y - bbox.size_y / 2)
            w = int(bbox.size_x)
            h = int(bbox.size_y)
            
            # Draw bounding box
            color = (0, 255, 0)  # Green color
            thickness = 2
            cv2.rectangle(image, (x, y), (x + w, y + h), color, thickness)
            
            # Draw label
            label = ""
            if detection.results and len(detection.results) > 0:
                label = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                
                # Add label text
                label_text = f"{label}: {confidence:.2f}"
                cv2.putText(image, label_text, (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return image

    def process_detections(self, detections):
        """Apply additional processing to detections"""
        processed = Detection2DArray()
        processed.header = detections.header
        
        # Filter detections based on confidence
        for detection in detections.detections:
            # Only keep high-confidence detections
            if detection.results and len(detection.results) > 0:
                confidence = detection.results[0].hypothesis.score
                if confidence > 0.7:  # Confidence threshold
                    processed.detections.append(detection)
        
        return processed

    def transform_point_to_global(self, point, source_frame, target_frame):
        """Transform a point from one frame to another"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            
            # Apply transformation to point
            # This is a simplified version - full implementation would use tf2_geometry_msgs
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)
            return transformed_point
        except Exception as e:
            self.get_logger().error(f'Error transforming point: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    perception_processor = PerceptionProcessor()
    
    try:
        rclpy.spin(perception_processor)
    except KeyboardInterrupt:
        pass
    
    perception_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Create Perception Package

Create a package.xml for the perception package (`humanoid_perception/package.xml`):

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_perception</name>
  <version>1.0.0</version>
  <description>Perception system for humanoid robot using Isaac ROS</description>
  <maintainer email="robotics@todo.com">Humanoid Robotics Team</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>cv_bridge</depend>
  <depend>message_filters</depend>

  <exec_depend>python3-opencv</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

And create a setup.py file:

```python
from setuptools import setup

package_name = 'humanoid_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humanoid Robotics Team',
    maintainer_email='robotics@todo.com',
    description='Perception system for humanoid robot using Isaac ROS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_processor = humanoid_perception.perception_processor:main',
        ],
    },
)
```

### Step 6: Integration Testing

Test the complete perception system:

1. **Launch the perception system:**
```bash
ros2 launch humanoid_perception perception.launch.py
```

2. **Visualize the processed data in RViz:**
```bash
ros2 run rviz2 rviz2
```

3. **Monitor the outputs:**
```bash
# View camera images
ros2 topic echo /camera/rgb/image_raw

# View object detections
ros2 topic echo /detectnet/detections

# View processed detections
ros2 topic echo /processed_detections

# View SLAM pose
ros2 topic echo /visual_slam/pose
```

## Deliverables

- Complete perception pipeline with Isaac ROS integration
- Working object detection and SLAM system
- Processed data visualization
- Documentation of perception parameters and tuning
- Video demonstration of perception system functioning in simulation

## Performance Considerations

### 1. Computational Efficiency
- Optimize neural networks for real-time performance
- Use appropriate image resolutions for processing
- Implement efficient data structures and algorithms

### 2. Accuracy vs. Speed Trade-offs
- Adjust confidence thresholds based on requirements
- Consider different models for different scenarios
- Implement adaptive processing based on scene complexity

## Next Phase

After completing Phase 3, proceed to Phase 4 where you'll implement navigation and planning capabilities for your humanoid robot using Nav2.