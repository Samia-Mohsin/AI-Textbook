---
title: "Sensor simulation: LiDAR, Depth, IMU, RGB"
description: "Simulating various sensors for humanoid robot perception in Gazebo"
learning_objectives:
  - "Implement LiDAR simulation for environment mapping"
  - "Configure depth sensors for 3D perception"
  - "Set up IMU sensors for orientation and balance"
  - "Integrate RGB cameras for visual perception"
---

# Sensor simulation: LiDAR, Depth, IMU, RGB

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement LiDAR simulation for environment mapping
- Configure depth sensors for 3D perception
- Set up IMU sensors for orientation and balance
- Integrate RGB cameras for visual perception

## Introduction

Sensors are the eyes and ears of humanoid robots, providing crucial information about the environment and the robot's state. In simulation, accurate sensor modeling is essential for developing and testing perception algorithms before deployment on real hardware. This chapter will guide you through implementing various sensor types in Gazebo, with a focus on sensors commonly used in humanoid robotics.

## LiDAR Simulation

### Understanding LiDAR in Simulation

LiDAR (Light Detection and Ranging) sensors emit laser beams and measure the time it takes for the light to return after hitting an object. In Gazebo, LiDAR simulation provides accurate distance measurements for mapping and navigation.

### Creating a 2D LiDAR Sensor

```xml
<!-- LiDAR link -->
<link name="laser_link">
  <visual>
    <geometry>
      <cylinder radius="0.02" length="0.04"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.02" length="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
  </inertial>
</link>

<!-- Mount the LiDAR on the robot -->
<joint name="laser_mount_joint" type="fixed">
  <parent link="torso"/>
  <child link="laser_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>  <!-- Position on top of torso -->
</joint>

<!-- Gazebo LiDAR sensor definition -->
<gazebo reference="laser_link">
  <sensor name="laser_scan" type="ray">
    <always_on>true</always_on>
    <update_rate>40</update_rate>
    <ray>
      <!-- Horizontal properties -->
      <scan>
        <horizontal>
          <samples>720</samples>  <!-- Number of beams in 360° -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -π radians = -180° -->
          <max_angle>3.14159</max_angle>   <!-- π radians = 180° -->
        </horizontal>
      </scan>

      <!-- Range properties -->
      <range>
        <min>0.1</min>    <!-- Minimum detectable range -->
        <max>30.0</max>   <!-- Maximum detectable range -->
        <resolution>0.01</resolution>
      </range>

      <!-- Noise properties -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
      </noise>
    </ray>

    <!-- Output topic -->
    <plugin name="laser_controller" filename="gz-sim-ray-system">
      <topic>humanoid_robot/laser_scan</topic>
      <frame_name>laser_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Creating a 3D LiDAR Sensor (Velodyne-style)

```xml
<!-- 3D LiDAR link -->
<link name="velodyne_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
    <material name="dark_grey">
      <color rgba="0.3 0.3 0.3 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
  </inertial>
</link>

<joint name="velodyne_mount_joint" type="fixed">
  <parent link="head"/>
  <child link="velodyne_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="velodyne_link">
  <sensor name="velodyne_sensor" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>  <!-- 16 vertical beams -->
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.2</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="velodyne_controller" filename="gz-sim-ray-system">
      <topic>humanoid_robot/velodyne_points</topic>
      <frame_name>velodyne_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Sensor Simulation

### RGB-D Camera Configuration

Depth sensors provide 3D information about the environment, crucial for navigation and manipulation:

```xml
<!-- Depth camera link -->
<link name="depth_camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.1 0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.1 0.03"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="depth_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="depth_camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="depth_cam">
      <horizontal_fov>1.0472</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="depth_camera_controller" filename="gz-sim-depth-camera-system">
      <topic>humanoid_robot/depth/image_raw</topic>
      <camera_info_topic>humanoid_robot/depth/camera_info</topic>
      <frame_name>depth_camera_link</frame_name>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

### Stereo Camera Configuration

Stereo cameras can also provide depth information:

```xml
<!-- Left camera -->
<link name="left_camera_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="left_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="left_camera_link"/>
  <origin xyz="0.05 0.05 0.05" rpy="0 0 0"/>
</joint>

<!-- Right camera -->
<link name="right_camera_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="right_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="right_camera_link"/>
  <origin xyz="0.05 -0.05 0.05" rpy="0 0 0"/>
</joint>

<!-- Left camera sensor -->
<gazebo reference="left_camera_link">
  <sensor name="left_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="left">
      <horizontal_fov>1.0472</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>30</far>
      </clip>
    </camera>
  </sensor>
</gazebo>

<!-- Right camera sensor -->
<gazebo reference="right_camera_link">
  <sensor name="right_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="right">
      <horizontal_fov>1.0472</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>30</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## IMU Sensor Simulation

### IMU for Balance and Orientation

IMU (Inertial Measurement Unit) sensors provide crucial information for humanoid balance and orientation:

```xml
<!-- IMU link (usually placed at robot's center of mass) -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- At approximate CoM -->
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <!-- Angular velocity noise -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>  <!-- 0.2 mrad/s stddev -->
            <bias_mean>0.00001</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.00001</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.00001</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </z>
      </angular_velocity>

      <!-- Linear acceleration noise -->
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>  <!-- 1.7 cm/s² stddev -->
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_controller" filename="gz-sim-imu-system">
      <topic>humanoid_robot/imu/data</topic>
      <frame_name>imu_link</frame_name>
      <update_rate>100</update_rate>
    </plugin>
  </sensor>
</gazebo>
```

### Multiple IMU Configuration

For humanoid robots, you might want IMUs on different body parts:

```xml
<!-- Head IMU for head orientation -->
<link name="head_imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="head_imu_joint" type="fixed">
  <parent link="head"/>
  <child link="head_imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="head_imu_link">
  <sensor name="head_imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>
    <imu>
      <!-- More accurate for head tracking -->
      <angular_velocity>
        <x><noise type="gaussian"><stddev>1e-4</stddev></noise></x>
        <y><noise type="gaussian"><stddev>1e-4</stddev></noise></y>
        <z><noise type="gaussian"><stddev>1e-4</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><stddev>1e-2</stddev></noise></x>
        <y><noise type="gaussian"><stddev>1e-2</stddev></noise></y>
        <z><noise type="gaussian"><stddev>1e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## RGB Camera Simulation

### Basic RGB Camera

RGB cameras provide visual information for object recognition, navigation, and interaction:

```xml
<!-- RGB camera link -->
<link name="rgb_camera_link">
  <visual>
    <geometry>
      <box size="0.03 0.04 0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.03 0.04 0.03"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.02"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="rgb_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="rgb_camera_link"/>
  <origin xyz="0.06 0 0.02" rpy="0 0 0"/>  <!-- Positioned like eyes -->
</joint>

<gazebo reference="rgb_camera_link">
  <sensor name="rgb_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="gz-sim-camera-system">
      <topic>humanoid_robot/rgb/image_raw</topic>
      <camera_info_topic>humanoid_robot/rgb/camera_info</topic>
      <frame_name>rgb_camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Wide-Angle Camera for Perception

```xml
<!-- Wide-angle camera for broader view -->
<link name="wide_camera_link">
  <inertial>
    <mass value="0.02"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="wide_camera_joint" type="fixed">
  <parent link="torso"/>
  <child link="wide_camera_link"/>
  <origin xyz="0.1 0 0.1" rpy="0 0.2 0"/>  <!-- Slightly upward angle -->
</joint>

<gazebo reference="wide_camera_link">
  <sensor name="wide_camera" type="wideanglecamera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="wide_angle">
      <horizontal_fov>2.0944</horizontal_fov>  <!-- 120 degrees -->
      <image>
        <width>1280</width>
        <height>720</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## Sensor Fusion and Integration

### Creating a Sensor Configuration File

Create a YAML file to configure all sensors for your robot:

```yaml
# config/sensors.yaml
humanoid_robot:
  ros__parameters:
    # LiDAR parameters
    laser_scan:
      frame_id: "laser_link"
      range_min: 0.1
      range_max: 30.0
      angle_min: -3.14159
      angle_max: 3.14159
      angle_increment: 0.00872665  # 0.5 degrees

    # Camera parameters
    rgb_camera:
      image_width: 800
      image_height: 600
      camera_frame_id: "rgb_camera_link"
      distortion_model: "plumb_bob"
      distortion_coefficients:
        k1: 0.0
        k2: 0.0
        p1: 0.0
        p2: 0.0
        k3: 0.0
      projection_coefficients:
        fx: 554.256
        fy: 554.256
        cx: 400.0
        cy: 300.0

    # IMU parameters
    imu:
      frame_id: "imu_link"
      rate: 100.0
      noise_density: 0.0001
      random_walk: 0.0001
```

### Sensor Processing Node

Create a ROS2 node to process sensor data:

```python
# sensors/sensor_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers for all sensors
        self.laser_sub = self.create_subscription(
            LaserScan, 'humanoid_robot/laser_scan', self.laser_callback, 10
        )

        self.rgb_sub = self.create_subscription(
            Image, 'humanoid_robot/rgb/image_raw', self.rgb_callback, 10
        )

        self.depth_sub = self.create_subscription(
            Image, 'humanoid_robot/depth/image_raw', self.depth_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, 'humanoid_robot/imu/data', self.imu_callback, 10
        )

        # Create publishers for processed data
        self.obstacle_pub = self.create_publisher(
            LaserScan, 'processed_obstacles', 10
        )

        self.get_logger().info('Sensor processor initialized')

    def laser_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        # Convert to numpy array
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Simple obstacle detection
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            if min_distance < 1.0:  # Obstacle within 1 meter
                self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')

    def rgb_callback(self, msg):
        """Process RGB camera data"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Example: detect edges using Canny
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Display image (for development)
            cv2.imshow('RGB Camera', cv_image)
            cv2.imshow('Edges', edges)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        """Process depth camera data"""
        try:
            # Convert depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')

            # Example: calculate distance to center point
            h, w = depth_image.shape
            center_depth = depth_image[h//2, w//2]

            if not np.isnan(center_depth) and not np.isinf(center_depth):
                self.get_logger().info(f'Depth at center: {center_depth:.2f}m')

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def imu_callback(self, msg):
        """Process IMU data for balance"""
        # Extract orientation
        orientation = msg.orientation
        # Extract angular velocity
        angular_velocity = msg.angular_velocity
        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration

        # Example: calculate roll and pitch from quaternion
        import math
        # Convert quaternion to roll/pitch (simplified)
        sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z)
        cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
        pitch = math.asin(sinp)

        # Log orientation for monitoring
        self.get_logger().info(f'Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    processor = SensorProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Sensor Features

### Multi-Echo LiDAR Simulation

For more realistic LiDAR simulation with multiple returns:

```xml
<gazebo reference="laser_link">
  <sensor name="multi_echo_lidar" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1081</samples>  <!-- Velodyne HDL-64E resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>64</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>
          <max_angle>0.2618</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.4</min>
        <max>120.0</max>
        <resolution>0.001</resolution>
      </range>
    </ray>
    <!-- Enable multi-echo -->
    <ray>
      <multi_echo>true</multi_echo>
    </ray>
  </sensor>
</gazebo>
```

### Thermal Camera Simulation

For advanced perception applications:

```xml
<gazebo reference="thermal_camera_link">
  <sensor name="thermal_camera" type="thermal_camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="thermal">
      <horizontal_fov>1.0472</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>L8</format>  <!-- Grayscale intensity -->
      </image>
      <clip>
        <near>0.1</near>
        <far>50</far>
      </clip>
      <thermal>
        <temperature_resolution>0.1</temperature_resolution>
        <max_temperature>100</max_temperature>
        <min_temperature>-20</min_temperature>
      </thermal>
    </camera>
  </sensor>
</gazebo>
```

## Practical Exercise: Complete Sensor Suite for Humanoid Robot

Create a complete sensor configuration for a humanoid robot:

1. Add all sensor types to your robot URDF
2. Create a launch file to start the robot with all sensors
3. Write a simple sensor processing node
4. Test the sensors in Gazebo

```python
# launch/sensor_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Sensor processor node
    sensor_processor = Node(
        package='my_humanoid_sensors',
        executable='sensor_processor',
        name='sensor_processor',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz2 for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        sensor_processor,
        rviz
    ])
```

Test the sensor suite:

```bash
# Build your packages
cd ros2_ws
colcon build --packages-select my_humanoid_description my_humanoid_sensors
source install/setup.bash

# Launch the robot with sensors
ros2 launch my_humanoid_sensors sensor_humanoid.launch.py

# Monitor sensor topics
ros2 topic echo /humanoid_robot/laser_scan
ros2 topic echo /humanoid_robot/rgb/image_raw
ros2 topic echo /humanoid_robot/imu/data
```

## Sensor Calibration and Validation

### Checking Sensor Data Quality

```bash
# Check laser scan statistics
ros2 run rqt_plot rqt_plot --topics /humanoid_robot/laser_scan/ranges[0] /humanoid_robot/laser_scan/ranges[100]

# Monitor IMU data
ros2 run rqt_plot rqt_plot --topics /humanoid_robot/imu/data/linear_acceleration.x /humanoid_robot/imu/data/linear_acceleration.y

# Check camera data
ros2 run image_view image_view _image:=/humanoid_robot/rgb/image_raw
```

### Sensor Validation Techniques

1. **Range validation**: Ensure LiDAR ranges are within expected bounds
2. **Image quality**: Check for proper exposure and focus
3. **IMU stability**: Verify IMU readings are stable when robot is stationary
4. **Timing**: Ensure sensors publish at expected rates

## Troubleshooting Common Sensor Issues

### LiDAR Issues
- **No data**: Check if the sensor plugin is loaded
- **Incorrect ranges**: Verify range min/max values
- **Low update rate**: Check update_rate parameter

### Camera Issues
- **Black images**: Verify camera parameters and clipping distances
- **Distorted images**: Check camera calibration parameters
- **Low frame rate**: Reduce image resolution or update rate

### IMU Issues
- **Noisy data**: Adjust noise parameters in URDF
- **Drifting readings**: Verify IMU placement and orientation
- **Wrong units**: Ensure data is published in correct units (m/s², rad/s)

## Summary

In this chapter, we've explored the implementation of various sensors for humanoid robot perception in Gazebo. We covered LiDAR for mapping, depth sensors for 3D perception, IMUs for balance and orientation, and RGB cameras for visual perception. Proper sensor simulation is crucial for developing and testing perception algorithms before deployment on real hardware.

## Next Steps

- Implement a complete sensor suite on your humanoid robot
- Create perception algorithms that use multiple sensor types
- Learn about sensor fusion techniques
- Explore advanced sensor models in Gazebo
- Practice debugging sensor-related issues in simulation