---
sidebar_position: 2
---

# Phase 1: Robot Design & Simulation Setup

## Objectives

In this phase, you will:
- Design the physical structure of your humanoid robot
- Create a URDF model with appropriate joints, links, and sensors
- Set up the Gazebo simulation environment
- Validate the robot model in simulation

## Robot Design Specifications

Your humanoid robot should include the following components:

- **Torso**: Main body with mounting points for sensors and actuators
- **Head**: With cameras for vision and possibly other sensors
- **Arms**: Two arms with shoulders, elbows, and wrists
- **Hands**: Simplified grippers or more complex manipulators
- **Legs**: Two legs with hips, knees, and ankles for locomotion
- **Sensors**: RGB-D camera, IMU, force/torque sensors, joint position/velocity sensors

## URDF Model Creation

Create a detailed URDF model following these steps:

1. **Define Robot Links**: Each physical component (limbs, torso) should be a link with appropriate mass, inertia properties, and visual/collision meshes.

2. **Define Joints**: Connect the links with appropriate joint types (revolute for most joints, fixed for sensors).

3. **Add Sensors**: Include camera, IMU, and other sensor models in your URDF.

4. **Add Transmission Elements**: Define how actuators connect to joints.

## Implementation Steps

### Step 1: Create Robot Package Structure

```
humanoid_description/
├── CMakeLists.txt
├── package.xml
├── meshes/
│   ├── base/
│   ├── head/
│   ├── arms/
│   ├── legs/
│   └── hands/
├── urdf/
│   ├── humanoid.urdf.xacro
│   ├── head.xacro
│   ├── arm.xacro
│   ├── leg.xacro
│   └── hand.xacro
└── launch/
    └── spawn_robot.launch.py
```

### Step 2: Define the Base Link

Start with defining the main torso/base link of your robot with appropriate physical properties:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
  </link>
</robot>
```

### Step 3: Add Sensors

Include necessary sensors for perception and control:

```xml
<!-- RGB-D Camera -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.1 0.04"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.1 0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>
```

### Step 4: Gazebo Integration

Add Gazebo-specific plugins and materials:

```xml
<!-- Gazebo material -->
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>
</gazebo>

<!-- Camera plugin -->
<gazebo reference="camera_link">
  <sensor type="depth" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="camera">
      <horizontal_fov>1.047</horizontal_fov>
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
      <frame_name>camera_link</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>100</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

## Validation Steps

1. **URDF Validation**: Use `check_urdf` to validate your robot model
2. **Visual Inspection**: Use RViz to visualize the robot model
3. **Simulation Testing**: Spawn the robot in Gazebo and verify proper functioning

## Deliverables

- Complete URDF model of your humanoid robot
- Launch file to spawn the robot in Gazebo
- Documentation of design decisions and key parameters
- Screenshots of the robot model in RViz and Gazebo

## Common Issues and Troubleshooting

- **Inertia Issues**: Ensure all links have properly defined inertia properties
- **Joint Limits**: Set appropriate position and velocity limits for joints
- **Mesh Loading**: Verify that mesh files are accessible from the URDF
- **Gazebo Plugins**: Ensure all sensor plugins are correctly configured

## Next Phase

After completing Phase 1, proceed to Phase 2 where you'll implement basic locomotion controls for your humanoid robot.