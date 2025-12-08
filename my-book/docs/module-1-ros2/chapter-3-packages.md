---
title: "Packages, Launch files, Parameters"
description: "Understanding ROS2 packages, launch systems, and parameter management"
learning_objectives:
  - "Create and structure ROS2 packages effectively"
  - "Use launch files to manage complex node configurations"
  - "Manage parameters for flexible node configuration"
  - "Implement parameter callbacks for dynamic reconfiguration"
---

# Packages, Launch files, Parameters

## Learning Objectives

By the end of this chapter, you will be able to:
- Create and structure ROS2 packages effectively
- Use launch files to manage complex node configurations
- Manage parameters for flexible node configuration
- Implement parameter callbacks for dynamic reconfiguration

## Introduction

ROS2 packages are the fundamental unit of code organization in ROS. They contain nodes, libraries, and other resources needed to perform specific functions. Launch files allow you to start multiple nodes with specific configurations simultaneously, while parameters provide a flexible way to configure node behavior without recompiling code. This chapter will guide you through these essential concepts for building modular and configurable humanoid robot systems.

## ROS2 Package Structure

A typical ROS2 package follows a standard structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml            # Package metadata
├── setup.py               # Python build configuration
├── setup.cfg              # Installation configuration
├── my_robot_package/      # Python module
│   ├── __init__.py
│   └── robot_nodes.py
├── launch/                # Launch files
│   └── robot.launch.py
├── config/                # Configuration files
│   └── robot_params.yaml
├── test/                  # Test files
└── resource/              # Additional resources
```

### Creating a Package

To create a new package:

```bash
# For Python packages
ros2 pkg create --build-type ament_python my_humanoid_controller

# For C++ packages
ros2 pkg create --build-type ament_cmake my_humanoid_controller
```

The `package.xml` file contains metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_humanoid_controller</name>
  <version>0.0.0</version>
  <description>Humanoid robot controller package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Launch Files

Launch files allow you to start multiple nodes with specific configurations. Here's a basic example:

```python
# launch/humanoid_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Create nodes
    controller_node = Node(
        package='my_humanoid_controller',
        executable='controller_node',
        name='humanoid_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'control_frequency': 100.0},
            {'max_joint_velocity': 2.0}
        ],
        output='screen'
    )

    sensor_processor_node = Node(
        package='my_humanoid_controller',
        executable='sensor_processor_node',
        name='sensor_processor',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'sensor_timeout': 0.1}
        ],
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        controller_node,
        sensor_processor_node
    ])
```

### Advanced Launch File Features

Launch files support many advanced features:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include other launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Group nodes with common namespace
    with_namespace = GroupAction(
        actions=[
            PushRosNamespace('humanoid_robot'),
            Node(
                package='my_humanoid_controller',
                executable='controller_node',
                name='controller'
            ),
            Node(
                package='my_humanoid_controller',
                executable='sensor_processor',
                name='sensor_processor'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        with_namespace
    ])
```

## Parameter Management

Parameters provide a flexible way to configure nodes without recompiling. ROS2 supports multiple ways to manage parameters.

### Parameter Declaration in Nodes

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Declare parameters with default values
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('max_joint_velocity', 2.0)
        self.declare_parameter('kp', 1.0)  # Proportional gain
        self.declare_parameter('ki', 0.1)  # Integral gain
        self.declare_parameter('kd', 0.05) # Derivative gain

        # Get parameter values
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value

        # Create PID controller with parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Create parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp' and param.type_ == Parameter.Type.DOUBLE:
                self.kp = param.value
                self.get_logger().info(f'Updated kp to: {self.kp}')
            elif param.name == 'ki' and param.type_ == Parameter.Type.DOUBLE:
                self.ki = param.value
                self.get_logger().info(f'Updated ki to: {self.ki}')
            elif param.name == 'kd' and param.type_ == Parameter.Type.DOUBLE:
                self.kd = param.value
                self.get_logger().info(f'Updated kd to: {self.kd}')
        return SetParametersResult(successful=True)
```

### Parameter Files

Parameters can be stored in YAML files:

```yaml
# config/humanoid_params.yaml
humanoid_controller:
  ros__parameters:
    control_frequency: 100.0
    max_joint_velocity: 2.0
    kp: 1.0
    ki: 0.1
    kd: 0.05
    joint_limits:
      - name: "hip_joint"
        min: -1.57
        max: 1.57
      - name: "knee_joint"
        min: 0.0
        max: 2.0
```

## Practical Exercise: Complete Humanoid Controller Package

Create a complete package structure for a humanoid controller:

1. Create the package:
```bash
cd ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_controller
```

2. Create the main controller node in `humanoid_controller/humanoid_controller/controller_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Declare parameters
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('joint_names', [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist'
        ])

        # Get parameters
        self.control_frequency = self.get_parameter('control_frequency').value
        self.joint_names = self.get_parameter('joint_names').value

        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, 'joint_trajectory', self.trajectory_callback, 10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)

        # Current joint states
        self.current_positions = {name: 0.0 for name in self.joint_names}

        self.get_logger().info('Humanoid controller initialized')

    def trajectory_callback(self, msg):
        # Process trajectory commands
        if len(msg.points) > 0:
            target_point = msg.points[0]
            for i, joint_name in enumerate(msg.joint_names):
                if joint_name in self.current_positions:
                    self.current_positions[joint_name] = target_point.positions[i]

    def control_loop(self):
        # Publish current joint states
        msg = JointState()
        msg.name = list(self.current_positions.keys())
        msg.position = list(self.current_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Create the launch file `humanoid_controller/launch/humanoid_controller.launch.py`

4. Update the `setup.py` file to make the node executable

5. Build the package:
```bash
cd ros2_ws
colcon build --packages-select humanoid_controller
```

## Summary

In this chapter, we've covered the essential aspects of ROS2 package organization, launch files, and parameter management. These concepts are crucial for building modular, configurable, and maintainable humanoid robot systems. Launch files allow you to manage complex multi-node systems, while parameters provide flexibility without recompilation.

## Next Steps

- Create your own package with multiple nodes
- Experiment with different parameter configurations
- Learn about ROS2 lifecycle nodes for more complex state management
- Explore composition of nodes for better performance