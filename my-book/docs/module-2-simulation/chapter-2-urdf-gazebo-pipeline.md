---
title: "URDF → Gazebo pipeline (SDF, controllers, sensors)"
description: "Creating the complete pipeline from URDF to Gazebo simulation with controllers and sensors"
learning_objectives:
  - "Understand the URDF to SDF conversion process"
  - "Integrate controllers for robot simulation"
  - "Add sensors to simulated robots"
  - "Validate and troubleshoot the simulation pipeline"
---

# URDF → Gazebo pipeline (SDF, controllers, sensors)

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the URDF to SDF conversion process
- Integrate controllers for robot simulation
- Add sensors to simulated robots
- Validate and troubleshoot the simulation pipeline

## Introduction

The pipeline from URDF (Unified Robot Description Format) to Gazebo simulation involves several key transformations and additions to make your robot model fully functional in simulation. While URDF describes the physical structure of your robot, Gazebo requires additional information for physics simulation, control interfaces, and sensor modeling. This chapter will guide you through creating a complete pipeline that transforms your URDF into a fully functional simulated humanoid robot.

## Understanding URDF to SDF Conversion

### The Conversion Process

When you load a URDF file into Gazebo, it gets automatically converted to SDF (Simulation Description Format). This conversion is handled by Gazebo's built-in parser, but you can also manually convert:

```bash
# Convert URDF to SDF manually
gz sdf -p my_robot.urdf > my_robot.sdf

# Or use the legacy tool
urdf_to_sdf my_robot.urdf > my_robot.sdf
```

### Key Differences Between URDF and SDF

| URDF | SDF |
|------|-----|
| XML-based | XML-based |
| Primarily for ROS | Simulation-focused |
| Joint limits defined in URDF | Can be enhanced in SDF |
| No physics engine config | Full physics config |
| No plugin support | Rich plugin support |
| Simple material definitions | Complex material properties |

## Extending URDF for Gazebo Simulation

### Gazebo-Specific Tags

To make your URDF work properly in Gazebo, you need to add Gazebo-specific tags:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include your basic robot definition -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific tags for the link -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>  <!-- Friction coefficient -->
    <mu2>0.2</mu2>  <!-- Secondary friction coefficient -->
    <kp>1000000.0</kp>  <!-- ODE stiffness coefficient -->
    <kd>100.0</kd>      <!-- ODE damping coefficient -->
    <self_collide>false</self_collide>
    <gravity>true</gravity>
  </gazebo>

  <!-- Joint definition -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0.15 0 -0.05" rpy="0 0 0"/>
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
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Gazebo-specific tags for the joint -->
  <gazebo reference="base_to_wheel">
    <provideFeedback>true</provideFeedback>
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>

</robot>
```

### Advanced Gazebo Properties

You can add more sophisticated properties to enhance simulation:

```xml
<gazebo reference="wheel_link">
  <!-- Custom material with PBR properties -->
  <material>
    <ambient>0.3 0.3 0.3 1.0</ambient>
    <diffuse>0.7 0.7 0.7 1.0</diffuse>
    <specular>0.5 0.5 0.5 1.0</specular>
    <emissive>0.0 0.0 0.0 1.0</emissive>
  </material>

  <!-- Contact properties for collision handling -->
  <collision>
    <max_contacts>10</max_contacts>
    <surface>
      <contact>
        <ode>
          <kp>1e+7</kp>
          <kd>1.0</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
          <fdir1>0 0 1</fdir1>
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

## Integrating ROS2 Controllers

### Setting up ros2_control

The `ros2_control` framework provides a standardized way to control simulated and real robots. To integrate it with Gazebo:

1. **Install ros2_control packages:**
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control
```

2. **Add ros2_control tags to your URDF:**

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">

  <!-- Your robot definition -->
  <link name="torso">
    <!-- ... link definition ... -->
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <!-- ... joint definition ... -->
  </joint>

  <!-- ros2_control interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <!-- Joint interfaces -->
    <joint name="left_hip_joint">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="left_knee_joint">
      <command_interface name="position">
        <param name="min">-0.1</param>
        <param name="max">2.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <!-- Add more joints as needed -->
  </ros2_control>

  <!-- Gazebo plugin for ros2_control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_humanoid_control)/config/humanoid_controllers.yaml</parameters>
      <ros>
        <namespace>/humanoid_robot</namespace>
      </ros>
    </plugin>
  </gazebo>

</robot>
```

### Controller Configuration File

Create a controller configuration file that matches your robot's structure:

```yaml
# config/humanoid_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    # Joint State Broadcaster (always needed)
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # Individual joint position controllers
    left_leg_controller:
      type: position_controllers/JointGroupPositionController

    right_leg_controller:
      type: position_controllers/JointGroupPositionController

    left_arm_controller:
      type: position_controllers/JointGroupPositionController

    right_arm_controller:
      type: position_controllers/JointGroupPositionController

    # Individual joint controllers for fine control
    left_hip_position_controller:
      type: position_controllers/JointTrajectoryController

    # Add more controllers as needed

# Left leg controller configuration
left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint

# Right leg controller configuration
right_leg_controller:
  ros__parameters:
    joints:
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint

# Left arm controller configuration
left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint
      - left_wrist_joint

# Right arm controller configuration
right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_joint
      - right_elbow_joint
      - right_wrist_joint

# Joint trajectory controller for smooth motion
left_hip_position_controller:
  ros__parameters:
    joints:
      - left_hip_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      left_hip_joint: { trajectory: 0.05, goal: 0.01 }
```

## Adding Sensors to Simulated Robots

### Camera Sensors

Add a camera to your robot for vision-based tasks:

```xml
<!-- Camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<!-- Camera joint -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo sensor definition -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.089</horizontal_fov> <!-- 62.4 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
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
      <camera_name>humanoid_robot/camera</camera_name>
      <frame_name>camera_link</frame_name>
      <min_depth>0.05</min_depth>
      <max_depth>300.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensors

Add an IMU for balance and orientation:

```xml
<!-- IMU link -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<!-- Fixed joint to attach IMU to torso -->
<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo IMU sensor -->
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
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
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
      <body_name>imu_link</body_name>
      <update_rate>100</update_rate>
      <gaussian_noise>0.001</gaussian_noise>
      <frame_id>imu_link</frame_id>
    </plugin>
  </sensor>
</gazebo>
```

### Force/Torque Sensors

Add force/torque sensors to joints for contact detection:

```xml
<!-- Gazebo plugin for joint force/torque sensor -->
<gazebo>
  <plugin name="left_ankle_ft_sensor" filename="gz-sim-joint-force-torque-system">
    <joint_name>left_ankle_joint</joint_name>
    <topic>left_ankle/ft_sensor</topic>
  </plugin>
</gazebo>
```

## Complete Example: Humanoid Robot URDF for Gazebo

Here's a complete example of a humanoid robot URDF ready for Gazebo simulation:

```xml
<?xml version="1.0"?>
<robot name="my_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base torso link -->
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

  <!-- Gazebo properties for torso -->
  <gazebo reference="torso">
    <material>Gazebo/White</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

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

  <!-- Left Leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="-0.05 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="3.0"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.0" effort="50" velocity="3.0"/>
  </joint>

  <!-- Add more links and joints for the rest of the humanoid... -->

  <!-- IMU sensor -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- ros2_control interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <!-- All joint interfaces -->
    <joint name="left_hip_joint">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <command_interface name="velocity">
        <param name="min">-3.0</param>
        <param name="max">3.0</param>
      </command_interface>
      <command_interface name="effort">
        <param name="min">-50.0</param>
        <param name="max">50.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="left_knee_joint">
      <command_interface name="position">
        <param name="min">0.0</param>
        <param name="max">2.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <!-- Add interfaces for all other joints -->
  </ros2_control>

  <!-- Gazebo plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_humanoid_control)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo sensor -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><stddev>2e-4</stddev></noise></x>
          <y><noise type="gaussian"><stddev>2e-4</stddev></noise></y>
          <z><noise type="gaussian"><stddev>2e-4</stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><stddev>1.7e-2</stddev></noise></x>
          <y><noise type="gaussian"><stddev>1.7e-2</stddev></noise></y>
          <z><noise type="gaussian"><stddev>1.7e-2</stddev></noise></z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

</robot>
```

## Launching the Complete Simulation

Create a launch file to bring up your robot in Gazebo:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_path = LaunchConfiguration('robot_description_path')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_robot_description_path = DeclareLaunchArgument(
        'robot_description_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_humanoid_description'),
            'urdf',
            'humanoid.urdf.xacro'
        ]),
        description='Path to robot description file'
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            robot_description_path,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Spawn robot in Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'  # Start 0.5m above ground
        ],
        output='screen',
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Robot controller spawners
    left_leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_leg_controller'],
    )

    right_leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_leg_controller'],
    )

    # Delay rviz start after joint_state_broadcaster and robot_controller spawners
    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz2_node],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_description_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        left_leg_controller_spawner,
        right_leg_controller_spawner,
    ])
```

## Validation and Troubleshooting

### Common Issues and Solutions

1. **Robot falls through the ground:**
   - Check inertial properties (mass, center of mass, inertia tensor)
   - Verify that collision meshes are properly defined
   - Ensure proper gravity settings in the world file

2. **Controllers don't respond:**
   - Check that joint names in controller config match URDF
   - Verify ros2_control hardware interface is loaded
   - Confirm controller manager is running

3. **Sensors not publishing data:**
   - Check sensor topic names
   - Verify sensor plugins are loaded
   - Confirm Gazebo simulation is running

### Validation Commands

```bash
# Check if controllers are loaded
ros2 control list_controllers

# Check robot state topics
ros2 topic list | grep joint

# Monitor joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames

# Test controller commands
ros2 topic pub /left_leg_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.1, 0.2, 0.05]}'
```

## Practical Exercise: Create Your Own URDF to Gazebo Pipeline

1. Create a simple humanoid URDF with at least 6 joints
2. Add ros2_control interfaces to all joints
3. Include an IMU sensor
4. Create a controller configuration file
5. Test the pipeline by launching the robot in Gazebo
6. Verify that you can command joint positions

```bash
# Build your packages
cd ros2_ws
colcon build --packages-select my_humanoid_description my_humanoid_control
source install/setup.bash

# Launch the simulation
ros2 launch my_humanoid_control humanoid_gazebo.launch.py

# Test joint control
ros2 topic pub /left_leg_controller/commands std_msgs/Float64MultiArray '{data: [0.1, 0.5, 0.0]}'
```

## Summary

In this chapter, we've explored the complete pipeline from URDF to functional Gazebo simulation. We covered adding Gazebo-specific tags, integrating ros2_control, adding sensors, and validating the simulation. The URDF to Gazebo pipeline is crucial for humanoid robotics development, enabling safe testing of control algorithms and perception systems.

## Next Steps

- Create your own complete humanoid robot model
- Experiment with different controller types
- Add more sophisticated sensors to your robot
- Learn about Gazebo worlds and environments
- Explore advanced physics properties and contact modeling