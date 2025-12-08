---
title: "Physics: gravity, collisions, contacts"
description: "Understanding and configuring physics simulation for humanoid robots in Gazebo"
learning_objectives:
  - "Configure physics engines for humanoid robot simulation"
  - "Understand collision detection and contact modeling"
  - "Tune physics parameters for realistic humanoid behavior"
  - "Troubleshoot physics-related simulation issues"
---

# Physics: gravity, collisions, contacts

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure physics engines for humanoid robot simulation
- Understand collision detection and contact modeling
- Tune physics parameters for realistic humanoid behavior
- Troubleshoot physics-related simulation issues

## Introduction

Physics simulation is the foundation of realistic robot behavior in Gazebo. For humanoid robots, proper physics configuration is critical for stable walking, balance control, and interaction with the environment. This chapter will guide you through understanding and configuring the physics engine, collision detection, and contact modeling to achieve realistic humanoid robot simulation.

## Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different characteristics:

### ODE (Open Dynamics Engine)

ODE is the default physics engine in Gazebo and is well-suited for humanoid robotics:

```xml
<!-- World file with ODE physics configuration -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics name="ode_physics" type="ode">
      <!-- Time step - smaller = more accurate but slower -->
      <max_step_size>0.001</max_step_size>

      <!-- Real-time update rate - how many physics steps per second -->
      <real_time_update_rate>1000</real_time_update_rate>

      <!-- Real-time factor - 1.0 means real-time, < 1.0 means faster than real-time -->
      <real_time_factor>1.0</real_time_factor>

      <!-- ODE-specific parameters -->
      <ode>
        <!-- Solver type: quick or world -->
        <solver>
          <type>quick</type>
          <iters>10</iters>  <!-- Number of iterations for constraint solver -->
          <sor>1.3</sor>     <!-- Successive Over-Relaxation parameter -->
        </solver>

        <!-- Constraints parameters -->
        <constraints>
          <cfm>0.000001</cfm>    <!-- Constraint Force Mixing -->
          <erp>0.2</erp>         <!-- Error Reduction Parameter -->
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

### Bullet Physics Engine

Bullet can be used for certain applications:

```xml
<physics name="bullet_physics" type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <real_time_factor>1.0</real_time_factor>

  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iterations>50</iterations>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
    </constraints>
  </bullet>
</physics>
```

## Gravity Configuration

Gravity is fundamental to humanoid robot simulation. The default gravity on Earth is 9.8 m/s²:

```xml
<!-- World with custom gravity -->
<world name="humanoid_world">
  <!-- Custom gravity vector (x, y, z) -->
  <gravity>0 0 -9.8</gravity>

  <!-- For different gravity (e.g., moon simulation) -->
  <!-- <gravity>0 0 -1.62</gravity> -->

  <!-- Include ground plane and other models -->
  <include>
    <uri>model://ground_plane</uri>
  </include>
</world>
```

### Gravity Considerations for Humanoid Robots

For humanoid robots, gravity significantly affects:
- Balance and stability
- Joint loading and torques
- Walking dynamics
- Contact forces

## Collision Detection and Contact Modeling

### Collision Properties

Collision properties define how objects interact physically:

```xml
<link name="left_foot">
  <collision name="collision">
    <geometry>
      <box>
        <size>0.15 0.1 0.05</size>  <!-- Foot-sized box -->
      </box>
    </geometry>

    <!-- Contact properties -->
    <surface>
      <contact>
        <ode>
          <!-- Stiffness and damping for contact joints -->
          <kp>1000000000000.0</kp>  <!-- Stiffness (P for proportional) -->
          <kd>1000000000000.0</kd>  <!-- Damping (D for derivative) -->

          <!-- Maximum velocity for contact joints -->
          <max_vel>100.0</max_vel>

          <!-- Minimum depth before contact constraints are applied -->
          <min_depth>0.001</min_depth>
        </ode>
      </contact>

      <!-- Friction properties -->
      <friction>
        <ode>
          <!-- Primary friction coefficient -->
          <mu>0.8</mu>

          <!-- Secondary friction coefficient -->
          <mu2>0.8</mu2>

          <!-- Direction of friction 2 -->
          <fdir1>1 0 0</fdir1>

          <!-- Slip coefficients -->
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>
      </friction>

      <!-- Bounce properties -->
      <bounce>
        <restitution_coefficient>0.01</restitution_coefficient>
        <threshold>100000</threshold>
      </bounce>
    </surface>
  </collision>
</link>
```

### Contact Sensors

For detailed contact information, you can add contact sensors:

```xml
<gazebo reference="left_foot">
  <sensor name="left_foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <contact>
      <collision>left_foot_collision</collision>
    </contact>
    <plugin name="left_foot_contact_plugin" filename="gz-sim-contact-system">
      <topic>left_foot/contacts</topic>
    </plugin>
  </sensor>
</gazebo>
```

## Joint Physics Properties

Joints in humanoid robots need careful physics configuration:

```xml
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="2.0" effort="100" velocity="3.0"/>

  <!-- Dynamics properties for the joint -->
  <dynamics damping="1.0" friction="0.1" spring_reference="0" spring_stiffness="0"/>
</joint>

<!-- Gazebo-specific joint properties -->
<gazebo reference="left_knee_joint">
  <!-- Provide feedback about joint forces -->
  <provideFeedback>true</provideFeedback>

  <!-- Use implicit spring damper for more stable simulation -->
  <implicitSpringDamper>1</implicitSpringDamper>

  <!-- ODE-specific joint properties -->
  <ode>
    <limit>
      <cfm>0.0</cfm>  <!-- Constraint Force Mixing for limits -->
      <erp>0.2</erp>  <!-- Error Reduction Parameter for limits -->
    </limit>
    <spring_reference>0.0</spring_reference>
    <spring_stiffness>0.0</spring_stiffness>
  </ode>
</gazebo>
```

## Inertial Properties

Proper inertial properties are crucial for realistic physics:

```xml
<link name="torso">
  <inertial>
    <!-- Mass in kg -->
    <mass value="5.0"/>

    <!-- Origin of the inertial reference frame -->
    <origin xyz="0 0 0.2" rpy="0 0 0"/>

    <!-- Inertia matrix -->
    <!-- For a box: Ixx = m/12 * (h² + d²), etc. -->
    <inertia
      ixx="0.2083" ixy="0.0" ixz="0.0"
      iyy="0.2917" iyz="0.0"
      izz="0.2917"/>
  </inertial>
</link>

<!-- More complex inertia calculation for irregular shapes -->
<link name="head">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <!-- For a sphere: Ixx = Iyy = Izz = 2/5 * m * r² -->
    <inertia
      ixx="0.004" ixy="0.0" ixz="0.0"
      iyy="0.004" iyz="0.0"
      izz="0.004"/>
  </inertial>
</link>
```

## Physics Tuning for Humanoid Robots

### Walking and Balance Considerations

For humanoid robots, physics tuning affects stability significantly:

```xml
<!-- Physics configuration optimized for humanoid walking -->
<physics name="humanoid_optimized" type="ode">
  <!-- Smaller time step for better stability -->
  <max_step_size>0.0005</max_step_size>

  <!-- Higher update rate for more responsive simulation -->
  <real_time_update_rate>2000</real_time_update_rate>

  <real_time_factor>1.0</real_time_factor>

  <ode>
    <solver>
      <type>quick</type>
      <!-- More iterations for better constraint solving -->
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>

    <constraints>
      <!-- Lower ERP for more accurate contact constraints -->
      <cfm>1e-5</cfm>
      <erp>0.1</erp>
      <contact_max_correcting_vel>10</contact_max_correcting_vel>
      <!-- Small surface layer to ensure contacts are detected -->
      <contact_surface_layer>0.0005</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Foot Contact Configuration

For stable walking, foot contact properties are critical:

```xml
<gazebo reference="left_foot">
  <collision name="left_foot_collision">
    <max_contacts>10</max_contacts>
    <surface>
      <contact>
        <ode>
          <!-- High stiffness for solid foot contact -->
          <kp>1000000000000.0</kp>
          <kd>1000000000000.0</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.0001</min_depth>
        </ode>
      </contact>

      <!-- High friction for good grip -->
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
          <fdir1>0 0 1</fdir1>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

## Advanced Physics Features

### Soft Contacts

For more realistic contact simulation:

```xml
<!-- In the world file -->
<physics name="soft_contacts" type="ode">
  <ode>
    <constraints>
      <!-- Soft contacts for more realistic interaction -->
      <contact_surface_layer>0.002</contact_surface_layer>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
    </constraints>
  </ode>
</physics>
```

### Joint Limits with Spring-Dampers

For more realistic joint behavior:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="0.5" effort="10" velocity="2.0"/>
  <dynamics damping="0.5" friction="0.1" spring_reference="-0.5" spring_stiffness="100"/>
</joint>
```

## Physics Debugging and Visualization

### Visualizing Contact Forces

You can visualize contact forces in Gazebo:

```xml
<!-- Add this to your world file -->
<world name="contact_debug_world">
  <!-- Enable contact visualization -->
  <physics name="debug_physics" type="ode">
    <ode>
      <show_contacts>true</show_contacts>
    </ode>
  </physics>
</world>
```

### Debugging Parameters

Use these parameters to identify physics issues:

```xml
<!-- In your URDF/Gazebo tags -->
<gazebo reference="problematic_link">
  <!-- Enable COM visualization -->
  <visualize_com>true</visualize_com>

  <!-- Enable inertia visualization -->
  <visualize_inertia>true</visualize_inertia>

  <!-- Enable contact visualization -->
  <visualize_contacts>true</visualize_contacts>
</gazebo>
```

## Practical Exercise: Physics Tuning for Humanoid Balance

Create a simple humanoid model and tune physics parameters for stable balance:

1. Create a simplified humanoid URDF with basic links and joints
2. Configure physics parameters for stable standing
3. Test balance by applying small disturbances
4. Adjust parameters to improve stability

```xml
<!-- balance_test.urdf.xacro -->
<?xml version="1.0"?>
<robot name="balance_test" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="PI" value="3.1415926535897931"/>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Left leg -->
  <link name="left_hip">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_hip"/>
    <origin xyz="-0.05 -0.1 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="3.0"/>
  </joint>

  <!-- Add more joints and links for complete model -->

  <!-- Gazebo physics properties -->
  <gazebo reference="torso">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="left_hip">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <!-- ros2_control interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="left_hip_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <!-- Add other joints -->
  </ros2_control>

  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_humanoid_control)/config/balance_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

## Performance Considerations

### Optimizing Physics Simulation

For better performance with humanoid robots:

1. **Reduce time step only when necessary**: Smaller time steps improve accuracy but reduce performance
2. **Adjust solver iterations**: More iterations = more accurate but slower
3. **Simplify collision meshes**: Use simpler shapes for collision detection
4. **Limit contact points**: Use `max_contacts` to limit the number of contact points
5. **Use fixed joints where possible**: Fixed joints are computationally cheaper than revolute joints

### Physics Parameters for Different Scenarios

| Scenario | Time Step | Solver Iters | ERP | CFM |
|----------|-----------|--------------|-----|-----|
| Stable walking | 0.001 | 20-50 | 0.1-0.2 | 1e-6 |
| Fast simulation | 0.002 | 10-20 | 0.2-0.5 | 1e-5 |
| Precise control | 0.0005 | 50-100 | 0.05-0.1 | 1e-7 |

## Troubleshooting Common Physics Issues

### Robot Falls Through Ground

**Causes and Solutions:**
- **Inertial properties too small**: Increase mass values
- **Collision meshes not defined**: Add proper collision elements
- **Physics parameters too loose**: Decrease ERP, increase KP/KD
- **Time step too large**: Reduce max_step_size

### Robot Jittering or Vibrating

**Causes and Solutions:**
- **Stiffness too high**: Lower KP values in contact parameters
- **Solver iterations too low**: Increase solver iterations
- **Time step too large**: Reduce time step
- **Inconsistent units**: Verify all units are consistent (SI units)

### Unstable Walking

**Causes and Solutions:**
- **Friction too low**: Increase mu values
- **Center of mass too high**: Redesign robot with lower CoM
- **Control frequency too low**: Increase controller update rate
- **Physics update rate too low**: Increase real_time_update_rate

## Summary

In this chapter, we've explored the critical aspects of physics simulation for humanoid robots in Gazebo. We covered physics engines, gravity configuration, collision detection, contact modeling, and parameter tuning. Proper physics configuration is essential for realistic humanoid robot behavior, especially for tasks like walking and balance control.

## Next Steps

- Experiment with different physics parameter sets
- Create a stable walking controller using the physics model
- Add more complex contact sensors to your robot
- Learn about advanced physics features like soft body simulation
- Explore physics debugging tools for identifying issues