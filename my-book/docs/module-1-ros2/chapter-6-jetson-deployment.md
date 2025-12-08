---
title: "ROS2 + Jetson deployment"
description: "Deploying ROS2 humanoid robot applications on NVIDIA Jetson platforms"
learning_objectives:
  - "Set up ROS2 on NVIDIA Jetson platforms"
  - "Optimize ROS2 applications for edge computing"
  - "Deploy humanoid control systems on Jetson hardware"
  - "Implement efficient communication between Jetson and robot"
---

# ROS2 + Jetson deployment

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up ROS2 on NVIDIA Jetson platforms
- Optimize ROS2 applications for edge computing
- Deploy humanoid control systems on Jetson hardware
- Implement efficient communication between Jetson and robot

## Introduction

NVIDIA Jetson platforms provide powerful, energy-efficient computing solutions for robotics applications. With their integrated GPU capabilities, Jetson devices are ideal for running perception, control, and AI algorithms on humanoid robots. This chapter will guide you through deploying ROS2-based humanoid robot applications on Jetson hardware, covering setup, optimization, and deployment strategies.

## Jetson Platform Overview

The NVIDIA Jetson family includes several models suitable for humanoid robotics:

- **Jetson Nano**: Entry-level, good for basic control and simple perception
- **Jetson TX2**: Mid-range, suitable for more complex algorithms
- **Jetson Xavier NX**: High-performance, ideal for advanced AI and perception
- **Jetson AGX Orin**: Top-tier, for the most demanding applications

### Hardware Specifications for Robotics

When selecting a Jetson platform for humanoid robotics, consider:
- CPU performance for ROS2 node processing
- GPU performance for perception and AI algorithms
- Memory capacity for multiple concurrent processes
- Power consumption for battery operation
- I/O ports for sensors and actuators

## Setting up ROS2 on Jetson

### Installing JetPack

JetPack is NVIDIA's platform software for Jetson devices:

1. Download JetPack from NVIDIA Developer website
2. Use SDK Manager to flash the Jetson device
3. Install base OS (Ubuntu 18.04/20.04 depending on JetPack version)

### Installing ROS2

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Set locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Add to bashrc for persistent setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Installing Jetson-Specific Packages

```bash
# Install Jetson.GPIO for hardware interface
sudo apt install python3-jetson-gpio

# Install camera and multimedia packages
sudo apt install nvidia-jetpack nvidia-jetpack-cuda

# Install additional dependencies
sudo apt install libopencv-dev python3-opencv
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

## Optimizing ROS2 for Jetson

### Resource Management

Jetson devices have limited resources compared to desktop systems. Optimize your ROS2 applications:

```python
# Efficient node implementation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge
import threading
import queue

class OptimizedHumanoidController(Node):
    def __init__(self):
        super().__init__('optimized_humanoid_controller')

        # Use threading for CPU-intensive tasks
        self.processing_queue = queue.Queue(maxsize=2)  # Limit queue size
        self.processing_thread = threading.Thread(target=self.process_sensor_data)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Efficient subscribers with QoS settings
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

        qos_profile = QoSProfile(
            depth=1,  # Minimal history
            durability=QoSDurabilityPolicy.VOLATILE,  # Best effort
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10
        )

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # Control loop at appropriate frequency
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

    def image_callback(self, msg):
        # Only process every Nth image to reduce CPU load
        if self.get_clock().now().nanoseconds % 2 == 0:  # Process every other image
            try:
                self.processing_queue.put_nowait(msg)
            except queue.Full:
                # Drop frame if queue is full
                pass

    def process_sensor_data(self):
        """Run in separate thread to avoid blocking ROS2 main thread"""
        while rclpy.ok():
            try:
                msg = self.processing_queue.get(timeout=0.1)
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

                # Perform vision processing
                processed_data = self.vision_processing(cv_image)

                # Publish results if needed
                # self.vision_pub.publish(processed_data)

            except queue.Empty:
                continue

    def vision_processing(self, image):
        # Efficient vision processing optimized for Jetson
        # Use TensorRT for neural networks when possible
        # Use CUDA-accelerated OpenCV functions
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return gray

    def control_loop(self):
        # Lightweight control logic
        # Offload heavy computation to other threads
        pass
```

### Memory Optimization

```python
# Memory-efficient message handling
class MemoryOptimizedNode(Node):
    def __init__(self):
        super().__init__('memory_optimized_node')

        # Pre-allocate message objects to reduce memory allocation
        self.joint_cmd_msg = Float64MultiArray()
        self.joint_cmd_msg.layout.dim.append(
            Float64MultiArray.layout.data_offset
        )

        # Use numpy arrays for numerical computations
        import numpy as np
        self.joint_targets = np.zeros(12, dtype=np.float64)  # Pre-allocate

        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 1
        )

    def publish_joint_commands(self, commands):
        # Reuse pre-allocated message object
        self.joint_cmd_msg.data = commands
        self.joint_cmd_pub.publish(self.joint_cmd_msg)
```

## Hardware Interface with Jetson

### GPIO and PWM Control

```python
# Hardware interface for actuators
import Jetson.GPIO as GPIO
import time

class JetsonHardwareInterface:
    def __init__(self):
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setwarnings(False)

        # Define pin assignments
        self.servo_pins = [11, 12, 13, 15, 16, 18]  # Example servo pins

        # Setup PWM channels
        self.pwm_channels = []
        for pin in self.servo_pins:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, 50)  # 50Hz for servos
            pwm.start(0)
            self.pwm_channels.append(pwm)

    def set_servo_position(self, servo_index, angle):
        """Set servo position (0-180 degrees)"""
        if 0 <= servo_index < len(self.pwm_channels):
            # Convert angle to duty cycle (2-12% for 0-180°)
            duty_cycle = 2 + (angle / 180.0) * 10
            self.pwm_channels[servo_index].ChangeDutyCycle(duty_cycle)
            time.sleep(0.02)  # Small delay for stability

    def cleanup(self):
        """Clean up GPIO resources"""
        for pwm in self.pwm_channels:
            pwm.stop()
        GPIO.cleanup()
```

### Camera Integration

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('jetson_camera_node')

        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Optimize camera settings for Jetson
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer

        # Publisher
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 1)

        # CV Bridge
        self.bridge = CvBridge()

        # Timer for camera capture
        self.camera_timer = self.create_timer(0.033, self.capture_frame)  # ~30 FPS

    def capture_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(ros_image)

    def destroy_node(self):
        # Release camera resources
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()
```

## Deployment Strategies

### Containerized Deployment with Docker

Create an optimized Dockerfile for Jetson:

```dockerfile
# Use NVIDIA's ROS2 container for Jetson
FROM nvcr.io/nvidia/l4t-ml:r35.2.1-py3

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-ros-core \
    ros-humble-ros-base \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 environment
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy application code
COPY . /app
WORKDIR /app

# Set up entrypoint
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "humanoid_control", "robot.launch.py"]
```

### ROS2 Launch Configuration for Jetson

```python
# launch/jetson_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    # Optimize for Jetson hardware
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='jetson_humanoid',
        description='Name of the robot'
    )

    # Set environment variables for Jetson optimization
    set_cuda_env = SetEnvironmentVariable(
        name='CUDA_VISIBLE_DEVICES',
        value='0'
    )

    # Humanoid controller node with optimized parameters
    humanoid_controller = Node(
        package='humanoid_control',
        executable='humanoid_controller',
        name='humanoid_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'control_frequency': 100.0},  # Adjust based on Jetson capability
            {'max_joint_velocity': 2.0},
            {'cpu_efficient_mode': True}  # Enable optimizations
        ],
        # Reduce CPU usage
        on_exit=None,
        respawn=False,
        respawn_delay=5.0
    )

    # Camera node
    camera_node = Node(
        package='humanoid_vision',
        executable='jetson_camera',
        name='jetson_camera',
        parameters=[
            {'camera_width': 640},
            {'camera_height': 480},
            {'camera_fps': 30}
        ]
    )

    # Balance controller
    balance_controller = Node(
        package='humanoid_control',
        executable='balance_controller',
        name='balance_controller',
        parameters=[
            {'control_frequency': 200.0},  # Higher frequency for balance
            {'enable_imu_filter': True}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_name,
        set_cuda_env,
        humanoid_controller,
        camera_node,
        balance_controller
    ])
```

## Communication Optimization

### Efficient Data Transfer

```python
# Efficient data structures for communication
import numpy as np
import struct

class EfficientDataTransfer:
    @staticmethod
    def pack_joint_data(joint_positions, joint_velocities, joint_efforts):
        """Pack joint data into compact binary format"""
        # Use struct for efficient binary packing
        data = b''
        data += struct.pack('I', len(joint_positions))  # Number of joints

        for pos, vel, eff in zip(joint_positions, joint_velocities, joint_efforts):
            data += struct.pack('ddd', pos, vel, eff)  # 3 doubles per joint

        return data

    @staticmethod
    def unpack_joint_data(binary_data):
        """Unpack joint data from binary format"""
        num_joints = struct.unpack('I', binary_data[:4])[0]
        joint_data = []

        offset = 4  # Skip the count
        for i in range(num_joints):
            pos, vel, eff = struct.unpack('ddd', binary_data[offset:offset+24])
            joint_data.append((pos, vel, eff))
            offset += 24

        return joint_data

# Use with custom message types or TCP communication
```

### Real-time Considerations

```python
import threading
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RealTimeController(Node):
    def __init__(self):
        super().__init__('realtime_controller')

        # Use QoS settings for real-time performance
        rt_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', rt_qos
        )

        # Use dedicated thread for time-critical operations
        self.rt_thread = threading.Thread(target=self.realtime_loop)
        self.rt_thread.daemon = True
        self.rt_thread.start()

    def realtime_loop(self):
        """Time-critical control loop"""
        period = 0.01  # 100 Hz
        while rclpy.ok():
            start_time = time.time()

            # Time-critical operations here
            self.execute_control_algorithm()

            # Maintain consistent timing
            elapsed = time.time() - start_time
            sleep_time = max(0, period - elapsed)
            time.sleep(sleep_time)

    def execute_control_algorithm(self):
        # Real-time critical control code
        pass
```

## Practical Exercise: Deploy a Simple Humanoid Controller on Jetson

Create a complete deployment example:

1. Create a launch file optimized for Jetson: `launch/jetson_humanoid.launch.py`

2. Create a resource monitor node to track system performance:

```python
import psutil
import GPUtil
from std_msgs.msg import String
import json

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')

        self.system_status_pub = self.create_publisher(
            String, 'system_status', 1
        )

        self.monitor_timer = self.create_timer(1.0, self.monitor_system)

    def monitor_system(self):
        # Monitor CPU, memory, GPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent

        gpus = GPUtil.getGPUs()
        gpu_load = gpus[0].load if gpus else 0

        status = {
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'gpu_load': gpu_load,
            'timestamp': self.get_clock().now().nanoseconds
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.system_status_pub.publish(status_msg)
```

3. Build and deploy to Jetson:
```bash
# Cross-compile or build directly on Jetson
cd ros2_ws
colcon build --packages-select humanoid_control humanoid_vision --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Launch the robot
ros2 launch humanoid_control jetson_humanoid.launch.py
```

## Performance Monitoring and Tuning

Monitor your deployed system:

```bash
# Check CPU usage
htop

# Check GPU usage
sudo tegrastats  # Jetson-specific

# Monitor ROS2 topics
ros2 topic hz /joint_states

# Check system resources
nvidia-smi
```

## Summary

In this chapter, we've covered the essential aspects of deploying ROS2 humanoid robot applications on NVIDIA Jetson platforms. We explored setup procedures, optimization techniques, hardware interfaces, and deployment strategies. Proper deployment on Jetson hardware enables humanoid robots to run perception, control, and AI algorithms efficiently at the edge.

## Next Steps

- Set up a Jetson development environment
- Deploy the example controller to a Jetson device
- Monitor and optimize performance based on real hardware
- Explore Jetson-specific libraries like TensorRT for AI acceleration
- Learn about power management for battery-powered humanoid robots