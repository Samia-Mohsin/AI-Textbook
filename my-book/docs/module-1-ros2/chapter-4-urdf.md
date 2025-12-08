---
title: "URDF: Joints, Links, Sensors"
description: "Understanding Universal Robot Description Format for humanoid robot modeling"
learning_objectives:
  - "Create URDF models for humanoid robots"
  - "Define joints, links, and their properties"
  - "Add sensors to robot models"
  - "Validate and visualize URDF models"
---

# URDF: Joints, Links, Sensors

## Learning Objectives

By the end of this chapter, you will be able to:
- Create URDF models for humanoid robots
- Define joints, links, and their properties
- Add sensors to robot models
- Validate and visualize URDF models

## Introduction

The Universal Robot Description Format (URDF) is an XML format used in ROS to describe robots. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), inertial properties, visual representations, and collision models. For humanoid robots, URDF is essential for simulation, visualization, and control. This chapter will guide you through creating comprehensive URDF models for humanoid robots.

## URDF Structure

A URDF file is an XML document that describes a robot's structure. Here's the basic structure:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link can have:
- Visual properties (how it looks)
- Collision properties (how it interacts physically)
- Inertial properties (mass, center of mass, inertia tensor)

### Visual Properties

```xml
<link name="torso">
  <visual>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/torso.stl"/>
    </geometry>
    <material name="light_grey">
      <color rgba="0.7 0.7 0.7 1.0"/>
    </material>
  </visual>
</link>
```

### Collision Properties

```xml
<link name="torso">
  <collision>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.15 0.4"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties

```xml
<link name="torso">
  <inertial>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <mass value="5.0"/>
    <inertia
      ixx="0.1" ixy="0.0" ixz="0.0"
      iyy="0.1" iyz="0.0"
      izz="0.1"/>
  </inertial>
</link>
```

## Joints

Joints define how links connect and move relative to each other. Common joint types include:

- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No movement (rigid connection)
- `floating`: 6 DOF movement (rarely used)
- `planar`: Movement on a plane

### Joint Examples

```xml
<!-- Hip joint (revolute) -->
<joint name="left_hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.1 -0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>

<!-- Knee joint (revolute) -->
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="2.0" effort="100" velocity="3.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>

<!-- Fixed joint for sensors -->
<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

## Sensors in URDF

Sensors are represented as special links with Gazebo plugins:

```xml
<!-- IMU sensor -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## Complete Humanoid URDF Example

Here's a simplified example of a humanoid torso with head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.15"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Additional links and joints would continue similarly... -->

</robot>
```

## Xacro for Complex Models

For complex humanoid robots, Xacro (XML Macros) helps reduce redundancy:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_width" value="0.2" />
  <xacro:property name="base_length" value="0.3" />
  <xacro:property name="base_height" value="0.1" />

  <!-- Macro for creating a simple link -->
  <xacro:macro name="simple_link" params="name xyz size mass color">
    <link name="${name}">
      <visual>
        <origin xyz="${xyz}"/>
        <geometry>
          <box size="${size}"/>
        </geometry>
        <material name="${color}">
          <color rgba="0.7 0.7 0.7 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="${xyz}"/>
        <geometry>
          <box size="${size}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_link name="base" xyz="0 0 0" size="0.3 0.2 0.1" mass="1.0" color="base_color"/>

</robot>
```

## Practical Exercise: Create a Simple Humanoid Model

Create a URDF file for a simple humanoid robot with:
1. A torso
2. A head
3. Two arms with shoulder and elbow joints
4. Two legs with hip and knee joints

1. Create the URDF file in `ros2_ws/src/humanoid_description/urdf/humanoid.urdf`

2. Create a launch file to display the robot:
```xml
<!-- In ros2_ws/src/humanoid_description/launch/display.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find the URDF file
    urdf_path = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'humanoid.urdf'
    )

    # Launch joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path, 'r').read()}]
    )

    # Launch RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz
    ])
```

3. Build and run:
```bash
cd ros2_ws
colcon build --packages-select humanoid_description
source install/setup.bash
ros2 launch humanoid_description display.launch.py
```

## Validation and Visualization

To validate your URDF:
```bash
# Check for XML syntax errors
xmllint --noout your_robot.urdf

# Check URDF validity
check_urdf your_robot.urdf

# Visualize in RViz
ros2 run rviz2 rviz2
```

## Summary

In this chapter, we've explored the fundamentals of URDF for humanoid robot modeling. We covered links, joints, sensors, and how to create complex models using Xacro. URDF is crucial for simulation, visualization, and control of humanoid robots in ROS2 environments.

## Next Steps

- Create your own humanoid URDF model
- Add realistic meshes for better visualization
- Include sensor models for perception
- Test your model in Gazebo simulation
- Learn about kinematic chains and inverse kinematics