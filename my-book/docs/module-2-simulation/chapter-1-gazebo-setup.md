---
title: "Gazebo setup"
description: "Setting up Gazebo simulation environment for humanoid robotics"
learning_objectives:
  - "Install and configure Gazebo for humanoid robot simulation"
  - "Understand Gazebo's architecture and components"
  - "Create basic simulation worlds"
  - "Integrate Gazebo with ROS2 for robot simulation"
---

# Gazebo setup

## Learning Objectives

By the end of this chapter, you will be able to:
- Install and configure Gazebo for humanoid robot simulation
- Understand Gazebo's architecture and components
- Create basic simulation worlds
- Integrate Gazebo with ROS2 for robot simulation

## Introduction

Gazebo is a powerful 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. For humanoid robotics, Gazebo enables safe testing of control algorithms, perception systems, and behaviors before deployment on real hardware. This chapter will guide you through setting up Gazebo for humanoid robot simulation and integrating it with ROS2.

## Installing Gazebo

### Installing Gazebo Garden (Recommended)

Gazebo Garden is the latest version with improved performance and features:

```bash
# Update package list
sudo apt update

# Install Gazebo Garden
sudo apt install gazebo-garden

# Install ROS2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-dev
```

### Alternative: Installing Classic Gazebo

If you prefer the classic Gazebo (version 11):

```bash
# Install classic Gazebo
sudo apt install gazebo libgazebo-dev

# Install ROS2 Gazebo classic packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-plugins
```

## Gazebo Architecture

Gazebo consists of several key components:

1. **Gazebo Server (gzserver)**: The physics simulation engine
2. **Gazebo Client (gzclient)**: The graphical user interface
3. **Gazebo Transport**: Communication layer between components
4. **Model Database**: Repository of pre-built models
5. **Plugins**: Extensible functionality for robots and sensors

### Core Concepts

- **World**: The simulation environment containing models, lighting, and physics properties
- **Model**: A robot or object with links, joints, and plugins
- **Link**: A rigid body with visual, collision, and inertial properties
- **Joint**: Connection between links with defined motion constraints
- **Plugin**: Code that extends Gazebo's functionality

## Basic Gazebo Usage

### Launching Gazebo

```bash
# Launch Gazebo with empty world
gazebo

# Launch Gazebo with a specific world
gazebo worlds/willowgarage.world

# Launch with GUI disabled (for headless simulation)
gzserver worlds/empty.world
```

### Gazebo Client Commands

```bash
# Launch only the GUI client
gzclient

# Connect to a running server
gzclient --verbose
```

## Creating a Basic World

Create a simple world file `my_humanoid_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Gazebo with ROS2 Integration

### Required ROS2 Packages

```bash
# Essential Gazebo-ROS2 packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-plugins
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

### Launching Gazebo with ROS2

Create a launch file `launch/gazebo_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    world = LaunchConfiguration('world')
    launch_gui = LaunchConfiguration('launch_gui')

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Choose one of the world files from `/usr/share/gazebo/worlds`'
    )

    declare_launch_gui_cmd = DeclareLaunchArgument(
        'launch_gui',
        default_value='True',
        description='Whether to launch the Gazebo GUI'
    )

    # Include Gazebo launch file
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
        }.items()
    )

    # Conditionally launch Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(launch_gui)
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_launch_gui_cmd,
        gazebo_server,
        gazebo_client
    ])
```

## Creating a Humanoid Robot Model for Gazebo

### URDF with Gazebo Plugins

To use your humanoid robot in Gazebo, you need to add Gazebo-specific tags to your URDF:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include the basic humanoid URDF -->
  <xacro:include filename="$(find my_humanoid_description)/urdf/humanoid.urdf.xacro"/>

  <!-- Gazebo plugins for ROS control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_humanoid_control)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- Example of a joint with Gazebo-specific properties -->
  <gazebo reference="left_hip_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>

</robot>
```

### Controller Configuration

Create a controller configuration file `config/humanoid_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: position_controllers/JointGroupPositionController

    right_leg_controller:
      type: position_controllers/JointGroupPositionController

    left_arm_controller:
      type: position_controllers/JointGroupPositionController

    right_arm_controller:
      type: position_controllers/JointGroupPositionController

left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint

right_leg_controller:
  ros__parameters:
    joints:
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint

left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint
      - left_wrist_joint

right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_joint
      - right_elbow_joint
      - right_wrist_joint
```

## Launching Robot in Gazebo

Create a launch file to spawn your robot in Gazebo:

```python
# launch/spawn_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_path = LaunchConfiguration('robot_description_path')
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_robot_description_path = DeclareLaunchArgument(
        'robot_description_path',
        default_value=PathJoinSubstitution([
            get_package_share_directory('my_humanoid_description'),
            'urdf',
            'humanoid.urdf'
        ]),
        description='Path to robot description file'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Choose one of the world files from `/usr/share/gazebo/worlds`'
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
        launch_arguments={
            'world': world,
            'verbose': 'false',
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', robot_description_path])
        }]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1.0'  # Spawn 1m above ground
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_description_path,
        declare_world,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## Advanced Gazebo Features

### Custom Plugins

Create a custom plugin for specialized robot behavior:

```cpp
// custom_humanoid_plugin.cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class HumanoidPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the model pointer for convenience
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&HumanoidPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model
      this->model->SetLinearVel(ignition::math::Vector3d(0.01, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HumanoidPlugin)
}
```

### Sensor Simulation

Add sensors to your robot for perception capabilities:

```xml
<!-- IMU sensor -->
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

<!-- Camera sensor -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Practical Exercise: Basic Humanoid in Gazebo

1. Create a simple humanoid URDF model with basic joints
2. Add Gazebo plugins for ROS2 control
3. Create a launch file to spawn the robot in Gazebo
4. Test the simulation by publishing joint commands

```bash
# Build your packages
cd ros2_ws
colcon build --packages-select my_humanoid_description my_humanoid_control
source install/setup.bash

# Launch the simulation
ros2 launch my_humanoid_control spawn_humanoid.launch.py

# In another terminal, send joint commands
ros2 topic pub /left_leg_controller/commands std_msgs/Float64MultiArray '{data: [0.1, 0.2, 0.05]}'
```

## Troubleshooting Common Issues

### Performance Issues
- Reduce physics update rate for better performance
- Simplify collision meshes
- Use less complex visual meshes
- Reduce the number of sensors if needed

### Physics Issues
- Adjust joint friction and damping parameters
- Verify inertial properties are realistic
- Check joint limits are properly set
- Ensure proper mass distribution

### ROS2 Integration Issues
- Verify controller configuration files
- Check topic names and message types
- Confirm robot description is properly loaded
- Validate TF tree is complete

## Summary

In this chapter, we've covered the fundamentals of setting up Gazebo for humanoid robot simulation. We explored installation, basic usage, ROS2 integration, and how to create simulation-ready robot models. Gazebo provides a safe and efficient environment for testing humanoid robot behaviors before deployment on real hardware.

## Next Steps

- Install Gazebo on your development system
- Create a simple humanoid model for simulation
- Integrate your robot with ROS2 control systems
- Experiment with different physics configurations
- Learn about more advanced simulation features like terrain and complex environments