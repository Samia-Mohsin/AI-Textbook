---
title: "Interactive humanoid testing environments"
description: "Creating and using interactive environments for humanoid robot testing in simulation"
learning_objectives:
  - "Design interactive testing environments for humanoid robots"
  - "Implement dynamic obstacles and interactive elements"
  - "Create standardized test scenarios and benchmarks"
  - "Evaluate humanoid robot performance in simulation"
---

# Interactive humanoid testing environments

## Learning Objectives

By the end of this chapter, you will be able to:
- Design interactive testing environments for humanoid robots
- Implement dynamic obstacles and interactive elements
- Create standardized test scenarios and benchmarks
- Evaluate humanoid robot performance in simulation

## Introduction

Interactive testing environments are crucial for developing and validating humanoid robots in simulation before deploying them in the real world. These environments allow for safe, repeatable testing of various capabilities including locomotion, manipulation, navigation, and human-robot interaction. This chapter will guide you through creating comprehensive testing environments that challenge humanoid robots in realistic scenarios.

## Designing Interactive Testing Environments

### Environment Categories

Testing environments can be categorized based on the capabilities they test:

1. **Locomotion Environments**: Focus on walking, balance, and terrain navigation
2. **Manipulation Environments**: Focus on grasping, object interaction, and dexterity
3. **Navigation Environments**: Focus on path planning, obstacle avoidance, and mapping
4. **Interaction Environments**: Focus on human-robot interaction and social behaviors

### Basic Environment Framework

Let's start with a basic framework for creating interactive environments:

```xml
<!-- interactive_environment.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="interactive_humanoid_test">
    <!-- Include basic world elements -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics configuration -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>1e-5</cfm>
          <erp>0.1</erp>
        </constraints>
      </ode>
    </physics>

    <!-- Test arena -->
    <model name="test_arena">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="arena_floor">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1000</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Lighting configuration -->
    <light name="arena_light" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>100</range>
      </attenuation>
      <direction>-0.3 -0.3 -1</direction>
    </light>
  </world>
</sdf>
```

## Locomotion Testing Environments

### Flat Terrain Challenge

```xml
<!-- flat_terrain_test.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="flat_terrain_test">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Start platform -->
    <model name="start_platform">
      <pose>-5 0 0.05 0 0 0</pose>
      <link name="platform">
        <collision name="collision">
          <geometry>
            <box><size>2 2 0.1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>2 2 0.1</size></box>
          </geometry>
          <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Goal platform -->
    <model name="goal_platform">
      <pose>5 0 0.05 0 0 0</pose>
      <link name="platform">
        <collision name="collision">
          <geometry>
            <box><size>2 2 0.1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>2 2 0.1</size></box>
          </geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Waypoint markers -->
    <model name="waypoint_1" type="waypoint">
      <pose>-2.5 0 0.5 0 0 0</pose>
      <link name="marker">
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>1</length></cylinder></geometry>
          <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="waypoint_2" type="waypoint">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="marker">
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>1</length></cylinder></geometry>
          <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="waypoint_3" type="waypoint">
      <pose>2.5 0 0.5 0 0 0</pose>
      <link name="marker">
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>1</length></cylinder></geometry>
          <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Obstacle Course Environment

```xml
<!-- obstacle_course.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="obstacle_course">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Start area -->
    <model name="start_area">
      <pose>-8 0 0.02 0 0 0</pose>
      <static>true</static>
      <link name="floor">
        <collision><geometry><box><size>4 4 0.04</size></box></geometry></collision>
        <visual><geometry><box><size>4 4 0.04</size></box></geometry>
          <material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      </link>
    </model>

    <!-- Obstacles -->
    <!-- Static obstacles -->
    <model name="obstacle_1">
      <pose>-4 1 0.5 0 0 0</pose>
      <link name="obstacle">
        <collision><geometry><box><size>0.5 0.5 1</size></box></geometry></collision>
        <visual><geometry><box><size>0.5 0.5 1</size></box></geometry>
          <material><ambient>0.8 0.4 0.2 1</ambient><diffuse>0.8 0.4 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>10</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
      </link>
    </model>

    <model name="obstacle_2">
      <pose>-4 -1 0.5 0 0 0</pose>
      <link name="obstacle">
        <collision><geometry><box><size>0.5 0.5 1</size></box></geometry></collision>
        <visual><geometry><box><size>0.5 0.5 1</size></box></geometry>
          <material><ambient>0.8 0.4 0.2 1</ambient><diffuse>0.8 0.4 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>10</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
      </link>
    </model>

    <!-- Dynamic obstacle (animated) -->
    <model name="moving_obstacle">
      <pose>0 0 0.2 0 0 0</pose>
      <link name="moving_part">
        <collision><geometry><cylinder><radius>0.3</radius><length>0.4</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.3</radius><length>0.4</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material>
        </visual>
        <inertial><mass>5</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
      </link>

      <!-- Gazebo plugin for animation -->
      <gazebo>
        <plugin name="mover" filename="libgazebo_ros_p3d.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>100</updateRate>
          <bodyName>moving_part</bodyName>
          <topicName>moving_obstacle/pose</topicName>
          <gaussianNoise>0.0</gaussianNoise>
          <frameName>world</frameName>
        </plugin>
      </gazebo>
    </model>

    <!-- Goal area -->
    <model name="goal_area">
      <pose>8 0 0.02 0 0 0</pose>
      <static>true</static>
      <link name="floor">
        <collision><geometry><box><size>4 4 0.04</size></box></geometry></collision>
        <visual><geometry><box><size>4 4 0.04</size></box></geometry>
          <material><ambient>0.8 0 0 1</ambient><diffuse>0.8 0 0 1</diffuse></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertial></inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Manipulation Testing Environments

### Object Manipulation Arena

```xml
<!-- manipulation_arena.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="manipulation_arena">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Workbench -->
    <model name="workbench">
      <pose>0 0 0.8 0 0 0</pose>
      <link name="table">
        <collision><geometry><box><size>2 1 0.8</size></box></geometry></collision>
        <visual><geometry><box><size>2 1 0.8</size></box></geometry>
          <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz></inertia></inertial>
      </link>
    </model>

    <!-- Objects to manipulate -->
    <model name="red_block">
      <pose>-0.5 0.3 1.05 0 0 0</pose>
      <link name="block">
        <collision><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>0.1 0.1 0.1</size></box></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      </link>
    </model>

    <model name="blue_cylinder">
      <pose>0.5 -0.3 1.05 0 0 0</pose>
      <link name="cylinder">
        <collision><geometry><cylinder><radius>0.05</radius><length>0.1</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.05</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material>
        </visual>
        <inertial><mass>0.3</mass><inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0001</izz></inertia></inertial>
      </link>
    </model>

    <model name="green_sphere">
      <pose>0 0.3 1.05 0 0 0</pose>
      <link name="sphere">
        <collision><geometry><sphere><radius>0.05</radius></sphere></geometry></collision>
        <visual><geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>0.2 0.8 0.2 1</ambient><diffuse>0.2 0.8 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>0.2</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      </link>
    </model>

    <!-- Target locations -->
    <model name="target_red">
      <pose>-0.8 -0.4 0.85 0 0 0</pose>
      <static>true</static>
      <link name="target">
        <visual><geometry><box><size>0.15 0.15 0.01</size></box></geometry>
          <material><ambient>0.8 0.2 0.2 0.3</ambient><diffuse>0.8 0.2 0.2 0.3</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="target_blue">
      <pose>0.8 0.4 0.85 0 0 0</pose>
      <static>true</static>
      <link name="target">
        <visual><geometry><box><size>0.15 0.15 0.01</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 0.3</ambient><diffuse>0.2 0.2 0.8 0.3</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="target_green">
      <pose>0 -0.4 0.85 0 0 0</pose>
      <static>true</static>
      <link name="target">
        <visual><geometry><box><size>0.15 0.15 0.01</size></box></geometry>
          <material><ambient>0.2 0.8 0.2 0.3</ambient><diffuse>0.2 0.8 0.2 0.3</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Navigation Testing Environments

### Maze Environment

```xml
<!-- maze_environment.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="maze_environment">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Maze walls -->
    <model name="maze_wall_1">
      <pose>0 5 0.5 0 0 0</pose>
      <static>true</static>
      <link name="wall">
        <collision><geometry><box><size>10 0.2 1</size></box></geometry></collision>
        <visual><geometry><box><size>10 0.2 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz></inertia></inertial>
      </link>
    </model>

    <model name="maze_wall_2">
      <pose>5 0 0.5 0 0 1.5708</pose>  <!-- Rotated 90 degrees -->
      <static>true</static>
      <link name="wall">
        <collision><geometry><box><size>10 0.2 1</size></box></geometry></collision>
        <visual><geometry><box><size>10 0.2 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz></inertia></inertial>
      </link>
    </model>

    <!-- More maze walls would be added here -->
    <model name="maze_wall_3">
      <pose>-3 3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="wall">
        <collision><geometry><box><size>4 0.2 1</size></box></geometry></collision>
        <visual><geometry><box><size>4 0.2 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
        <inertial><mass>40</mass><inertia><ixx>4</ixx><iyy>4</iyy><izz>4</izz></inertia></inertial>
      </link>
    </model>

    <model name="maze_wall_4">
      <pose>3 -3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="wall">
        <collision><geometry><box><size>4 0.2 1</size></box></geometry></collision>
        <visual><geometry><box><size>4 0.2 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
        <inertial><mass>40</mass><inertia><ixx>4</ixx><iyy>4</iyy><izz>4</izz></inertia></inertial>
      </link>
    </model>

    <!-- Start position marker -->
    <model name="start_marker">
      <pose>-4.5 4.5 0.05 0 0 0</pose>
      <static>true</static>
      <link name="marker">
        <visual><geometry><cylinder><radius>0.3</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Goal position marker -->
    <model name="goal_marker">
      <pose>4.5 -4.5 0.05 0 0 0</pose>
      <static>true</static>
      <link name="marker">
        <visual><geometry><cylinder><radius>0.3</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0.8 0 0 1</ambient><diffuse>0.8 0 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Dynamic obstacles -->
    <model name="moving_target">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="target">
        <collision><geometry><sphere><radius>0.3</radius></sphere></geometry></collision>
        <visual><geometry><sphere><radius>0.3</radius></geometry>
          <material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material>
        </visual>
        <inertial><mass>2</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
      </link>

      <!-- Plugin for movement -->
      <plugin name="waypoint_follower" filename="libgazebo_ros_p3d.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>100</updateRate>
        <bodyName>target</bodyName>
      </plugin>
    </model>
  </world>
</sdf>
```

## Dynamic and Interactive Elements

### Creating Interactive Objects

```xml
<!-- interactive_objects.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="interactive_objects">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Door that can be opened -->
    <model name="interactive_door">
      <pose>0 0 1 0 0 0</pose>
      <link name="door_frame">
        <collision><geometry><box><size>2 0.1 2</size></box></geometry></collision>
        <visual><geometry><box><size>2 0.1 2</size></box></geometry>
          <material><ambient>0.3 0.2 0.1 1</ambient><diffuse>0.3 0.2 0.1 1</diffuse></material>
        </visual>
        <inertial><mass>50</mass><inertia><ixx>5</ixx><iyy>5</iyy><izz>5</izz></inertia></inertial>
      </link>

      <link name="door_panel">
        <collision><geometry><box><size>0.9 0.05 1.8</size></box></geometry></collision>
        <visual><geometry><box><size>0.9 0.05 1.8</size></box></geometry>
          <material><ambient>0.5 0.3 0.2 1</ambient><diffuse>0.5 0.3 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>10</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      </link>

      <joint name="door_hinge" type="revolute">
        <parent>door_frame</parent>
        <child>door_panel</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit><lower>-1.57</lower><upper>1.57</upper></limit>
        </axis>
        <pose>0.5 0 0 0 0 0</pose>
      </joint>

      <!-- Gazebo plugin for interaction -->
      <gazebo reference="door_panel">
        <mu1>0.8</mu1>
        <mu2>0.8</mu2>
      </gazebo>
    </model>

    <!-- Button that can be pressed -->
    <model name="interactive_button">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="button_base">
        <collision><geometry><cylinder><radius>0.2</radius><length>0.1</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.2</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>5</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
      </link>

      <link name="button_top">
        <collision><geometry><cylinder><radius>0.15</radius><length>0.05</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.15</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.8 0 0 1</ambient><diffuse>0.8 0 0 1</diffuse></material>
        </visual>
        <inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      </link>

      <joint name="button_joint" type="prismatic">
        <parent>button_base</parent>
        <child>button_top</child>
        <axis><xyz>0 0 1</xyz></axis>
        <limit><lower>0</lower><upper>0.05</upper></limit>
      </joint>

      <!-- Gazebo plugin for button press detection -->
      <gazebo reference="button_top">
        <mu1>0.5</mu1>
        <mu2>0.5</mu2>
      </gazebo>
    </model>

    <!-- Switch that can be toggled -->
    <model name="interactive_switch">
      <pose>-2 0 0.5 0 0 0</pose>
      <link name="switch_base">
        <collision><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>0.1 0.1 0.1</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      </link>

      <link name="switch_handle">
        <collision><geometry><box><size>0.05 0.02 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>0.05 0.02 0.1</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
        <inertial><mass>0.1</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      </link>

      <joint name="switch_joint" type="revolute">
        <parent>switch_base</parent>
        <child>switch_handle</child>
        <axis><xyz>0 1 0</xyz></axis>
        <limit><lower>-0.5</lower><upper>0.5</upper></limit>
      </joint>
    </model>
  </world>
</sdf>
```

## Environment Management and Testing Framework

### Creating a Test Environment Manager

```python
# env_manager/env_test_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from gazebo_msgs.msg import ModelState
import xml.etree.ElementTree as ET
import os

class EnvironmentTestManager(Node):
    def __init__(self):
        super().__init__('environment_test_manager')

        # Service clients
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.set_state_cli = self.create_client(SetEntityState, '/set_entity_state')

        # Publishers for test control
        self.test_status_pub = self.create_publisher(String, 'test_status', 10)
        self.test_result_pub = self.create_publisher(String, 'test_result', 10)

        # Subscribers for test commands
        self.test_command_sub = self.create_subscription(
            String, 'test_command', self.test_command_callback, 10
        )

        # Timer for test monitoring
        self.test_timer = self.create_timer(1.0, self.monitor_test_progress)

        # Test state
        self.current_test = None
        self.test_active = False
        self.test_start_time = None

        self.get_logger().info('Environment Test Manager initialized')

    def test_command_callback(self, msg):
        """Handle test commands"""
        command = msg.data
        self.get_logger().info(f'Received test command: {command}')

        if command.startswith('start_'):
            test_name = command.split('_', 1)[1]
            self.start_test(test_name)
        elif command == 'stop_test':
            self.stop_test()
        elif command == 'reset_test':
            self.reset_test()
        elif command.startswith('configure_'):
            self.configure_test(command)

    def start_test(self, test_name):
        """Start a specific test environment"""
        if self.test_active:
            self.get_logger().warn('Test already active, stopping current test')
            self.stop_test()

        self.current_test = test_name
        self.test_active = True
        self.test_start_time = self.get_clock().now()

        # Load and spawn the appropriate environment
        self.load_test_environment(test_name)

        # Publish test status
        status_msg = String()
        status_msg.data = f'Test {test_name} started'
        self.test_status_pub.publish(status_msg)

        self.get_logger().info(f'Started test: {test_name}')

    def stop_test(self):
        """Stop the current test"""
        if not self.test_active:
            return

        test_duration = (self.get_clock().now() - self.test_start_time).nanoseconds / 1e9
        self.get_logger().info(f'Stopped test {self.current_test}, duration: {test_duration:.2f}s')

        # Clean up test environment
        self.cleanup_test_environment()

        self.test_active = False
        self.current_test = None

        # Publish test status
        status_msg = String()
        status_msg.data = 'Test stopped'
        self.test_status_pub.publish(status_msg)

    def reset_test(self):
        """Reset the current test"""
        if self.current_test:
            self.stop_test()
            self.start_test(self.current_test)

    def load_test_environment(self, test_name):
        """Load and spawn objects for the specified test"""
        if test_name == 'locomotion':
            self.load_locomotion_test()
        elif test_name == 'manipulation':
            self.load_manipulation_test()
        elif test_name == 'navigation':
            self.load_navigation_test()
        elif test_name == 'interaction':
            self.load_interaction_test()
        else:
            self.get_logger().error(f'Unknown test: {test_name}')

    def load_locomotion_test(self):
        """Load locomotion test environment"""
        # Spawn start platform
        self.spawn_object('start_platform', Pose(position=Point(x=-5.0, y=0.0, z=0.05)))

        # Spawn goal platform
        self.spawn_object('goal_platform', Pose(position=Point(x=5.0, y=0.0, z=0.05)))

        # Spawn obstacles
        self.spawn_object('obstacle_1', Pose(position=Point(x=-2.0, y=1.0, z=0.5)))
        self.spawn_object('obstacle_2', Pose(position=Point(x=-2.0, y=-1.0, z=0.5)))

    def load_manipulation_test(self):
        """Load manipulation test environment"""
        # Spawn workbench
        self.spawn_object('workbench', Pose(position=Point(x=0.0, y=0.0, z=0.8)))

        # Spawn objects to manipulate
        self.spawn_object('red_block', Pose(position=Point(x=-0.5, y=0.3, z=1.05)))
        self.spawn_object('blue_cylinder', Pose(position=Point(x=0.5, y=-0.3, z=1.05)))
        self.spawn_object('green_sphere', Pose(position=Point(x=0.0, y=0.3, z=1.05)))

        # Spawn target locations
        self.spawn_object('target_red', Pose(position=Point(x=-0.8, y=-0.4, z=0.85)))
        self.spawn_object('target_blue', Pose(position=Point(x=0.8, y=0.4, z=0.85)))
        self.spawn_object('target_green', Pose(position=Point(x=0.0, y=-0.4, z=0.85)))

    def load_navigation_test(self):
        """Load navigation test environment"""
        # For navigation, we might load a maze world file
        # In practice, this would involve spawning multiple maze elements
        self.get_logger().info('Loading navigation test environment')

    def load_interaction_test(self):
        """Load interaction test environment"""
        # Spawn interactive objects
        self.spawn_object('interactive_door', Pose(position=Point(x=0.0, y=0.0, z=1.0)))
        self.spawn_object('interactive_button', Pose(position=Point(x=2.0, y=0.0, z=0.5)))
        self.spawn_object('interactive_switch', Pose(position=Point(x=-2.0, y=0.0, z=0.5)))

    def spawn_object(self, object_name, pose):
        """Spawn an object in the simulation"""
        # This is a simplified example - in practice, you'd need to load the model definition
        # and call the spawn service
        self.get_logger().info(f'Spawning {object_name} at {pose}')

    def cleanup_test_environment(self):
        """Clean up the test environment"""
        # Delete all spawned objects
        # In practice, you'd need to track all spawned objects and delete them
        self.get_logger().info('Cleaning up test environment')

    def monitor_test_progress(self):
        """Monitor the progress of the current test"""
        if not self.test_active or not self.current_test:
            return

        # Calculate test duration
        current_time = self.get_clock().now()
        test_duration = (current_time - self.test_start_time).nanoseconds / 1e9

        # Check test-specific conditions
        if self.current_test == 'locomotion':
            self.check_locomotion_progress(test_duration)
        elif self.current_test == 'manipulation':
            self.check_manipulation_progress(test_duration)
        elif self.current_test == 'navigation':
            self.check_navigation_progress(test_duration)

    def check_locomotion_progress(self, duration):
        """Check progress of locomotion test"""
        # In practice, this would check robot position relative to goal
        if duration > 120:  # 2 minutes timeout
            self.get_logger().warn('Locomotion test timeout')
            self.stop_test()

    def check_manipulation_progress(self, duration):
        """Check progress of manipulation test"""
        # In practice, this would check object positions relative to targets
        if duration > 300:  # 5 minutes timeout
            self.get_logger().warn('Manipulation test timeout')
            self.stop_test()

    def check_navigation_progress(self, duration):
        """Check progress of navigation test"""
        # In practice, this would check robot path and goal achievement
        if duration > 180:  # 3 minutes timeout
            self.get_logger().warn('Navigation test timeout')
            self.stop_test()

    def configure_test(self, command):
        """Configure test parameters"""
        # Parse configuration command and adjust test parameters
        self.get_logger().info(f'Configuring test with: {command}')

def main(args=None):
    rclpy.init(args=args)
    manager = EnvironmentTestManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Standardized Test Scenarios and Benchmarks

### Creating Test Benchmark Definitions

```python
# benchmarks/humanoid_benchmarks.py
import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Tuple
import numpy as np

class TestCategory(Enum):
    LOCOMOTION = "locomotion"
    MANIPULATION = "manipulation"
    NAVIGATION = "navigation"
    INTERACTION = "interaction"
    BALANCE = "balance"

@dataclass
class TestMetrics:
    """Metrics for evaluating test performance"""
    success: bool = False
    completion_time: float = 0.0
    efficiency: float = 0.0  # 0-1 scale
    accuracy: float = 0.0    # 0-1 scale
    energy_consumption: float = 0.0
    safety_score: float = 0.0  # 0-1 scale
    error_count: int = 0

class HumanoidBenchmarks:
    """Standardized benchmarks for humanoid robot testing"""

    def __init__(self):
        self.benchmarks = {
            TestCategory.LOCOMOTION: [
                self.walk_10m_straight,
                self.walk_10m_curved,
                self.walk_up_slope,
                self.walk_down_slope,
                self.walk_over_uneven_terrain,
                self.walk_up_stairs,
                self.walk_down_stairs,
                self.balance_recovery,
                self.walk_through_narrow_space
            ],
            TestCategory.MANIPULATION: [
                self.pick_and_place_basic,
                self.pick_and_place_precise,
                self.handover_object,
                self.open_door,
                self.press_button,
                self.toggle_switch,
                self.assemble_parts
            ],
            TestCategory.NAVIGATION: [
                self.navigate_to_goal,
                self.navigate_maze,
                self.avoid_dynamic_obstacles,
                self.explore_unknown_environment
            ],
            TestCategory.INTERACTION: [
                self.follow_human_guide,
                self.respond_to_voice_commands,
                self.maintain_personal_space,
                self.gesture_recognition
            ],
            TestCategory.BALANCE: [
                self.balance_on_one_foot,
                self.recover_from_push,
                self.balance_on_unstable_surface
            ]
        }

    def walk_10m_straight(self, robot_position: Tuple[float, float, float],
                         start_time: float, current_time: float) -> TestMetrics:
        """Test: Walk 10 meters in a straight line"""
        target_distance = 10.0
        current_distance = math.sqrt(
            (robot_position[0] - 0)**2 + (robot_position[1] - 0)**2
        )

        metrics = TestMetrics()
        metrics.completion_time = current_time - start_time
        metrics.accuracy = min(1.0, current_distance / target_distance)
        metrics.efficiency = current_distance / metrics.completion_time if metrics.completion_time > 0 else 0

        if current_distance >= target_distance:
            metrics.success = True
            # Check if path was straight (lateral deviation)
            lateral_deviation = abs(robot_position[1])
            if lateral_deviation < 0.5:  # Within 50cm of center line
                metrics.efficiency = 1.0
            else:
                metrics.efficiency = max(0.0, 1.0 - (lateral_deviation / 2.0))

        return metrics

    def walk_up_stairs(self, robot_position: Tuple[float, float, float],
                      joint_states: Dict[str, float],
                      start_time: float, current_time: float) -> TestMetrics:
        """Test: Walk up a set of stairs"""
        # Define stair parameters
        stair_height = 0.15  # 15cm per step
        num_steps = 5
        target_height = num_steps * stair_height

        metrics = TestMetrics()
        metrics.completion_time = current_time - start_time

        # Check if robot reached target height
        if robot_position[2] >= target_height * 0.9:  # 90% of target
            metrics.success = True
            metrics.accuracy = min(1.0, robot_position[2] / target_height)

        # Calculate efficiency based on joint smoothness
        # This is a simplified calculation
        joint_smoothness_score = self.calculate_joint_smoothness(joint_states)
        metrics.efficiency = (metrics.accuracy + joint_smoothness_score) / 2

        return metrics

    def pick_and_place_basic(self, robot_end_effector_pos: Tuple[float, float, float],
                           target_object_pos: Tuple[float, float, float],
                           target_place_pos: Tuple[float, float, float],
                           object_held: bool,
                           start_time: float, current_time: float) -> TestMetrics:
        """Test: Basic pick and place task"""
        metrics = TestMetrics()
        metrics.completion_time = current_time - start_time

        # Check if object is picked up
        pick_distance = math.sqrt(
            sum((a - b)**2 for a, b in zip(robot_end_effector_pos, target_object_pos))
        )

        if pick_distance < 0.1 and object_held:  # Picked up the object
            # Check if placed at target
            place_distance = math.sqrt(
                sum((a - b)**2 for a, b in zip(robot_end_effector_pos, target_place_pos))
            )

            if place_distance < 0.15:  # Placed near target
                metrics.success = True
                metrics.accuracy = max(0.0, 1.0 - (place_distance / 0.5))  # 50cm tolerance

        metrics.efficiency = 1.0 / metrics.completion_time if metrics.completion_time > 0 else 0

        return metrics

    def navigate_to_goal(self, robot_position: Tuple[float, float, float],
                        goal_position: Tuple[float, float, float],
                        path_taken: List[Tuple[float, float, float]],
                        start_time: float, current_time: float) -> TestMetrics:
        """Test: Navigate to a goal position"""
        target_distance = math.sqrt(
            sum((a - b)**2 for a, b in zip(robot_position, goal_position))
        )

        metrics = TestMetrics()
        metrics.completion_time = current_time - start_time

        # Check if reached goal
        if target_distance < 0.5:  # Within 50cm of goal
            metrics.success = True
            metrics.accuracy = 1.0

        # Calculate path efficiency (shortest path vs actual path)
        if path_taken:
            actual_path_length = sum(
                math.sqrt(sum((a - b)**2 for a, b in zip(p1, p2)))
                for p1, p2 in zip(path_taken[:-1], path_taken[1:])
            )
            optimal_distance = math.sqrt(
                sum((a - b)**2 for a, b in zip(path_taken[0], goal_position))
            )

            if optimal_distance > 0:
                path_efficiency = optimal_distance / actual_path_length
                metrics.efficiency = min(1.0, path_efficiency)
            else:
                metrics.efficiency = 1.0

        return metrics

    def balance_recovery(self, robot_orientation: Tuple[float, float, float, float],
                        robot_angular_velocity: Tuple[float, float, float],
                        start_time: float, current_time: float) -> TestMetrics:
        """Test: Balance recovery after disturbance"""
        # Convert quaternion to roll/pitch
        roll, pitch, _ = self.quaternion_to_euler(robot_orientation)

        metrics = TestMetrics()
        metrics.completion_time = current_time - start_time

        # Check if robot maintained balance (angles within reasonable bounds)
        max_angle = math.radians(15)  # 15 degrees
        if abs(roll) < max_angle and abs(pitch) < max_angle:
            metrics.success = True
            # Calculate balance score based on angle deviation
            angle_deviation = (abs(roll) + abs(pitch)) / 2
            metrics.accuracy = max(0.0, 1.0 - (angle_deviation / max_angle))

        # Calculate stability based on angular velocity
        angular_velocity_magnitude = math.sqrt(
            sum(v**2 for v in robot_angular_velocity)
        )
        stability_score = max(0.0, 1.0 - angular_velocity_magnitude)
        metrics.efficiency = (metrics.accuracy + stability_score) / 2

        return metrics

    def calculate_joint_smoothness(self, joint_states: Dict[str, float]) -> float:
        """Calculate smoothness of joint movements"""
        # This is a simplified implementation
        # In practice, you'd track joint velocities and accelerations over time
        return 0.8  # Placeholder value

    def quaternion_to_euler(self, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def run_benchmark(self, category: TestCategory, test_index: int,
                     robot_data: Dict) -> TestMetrics:
        """Run a specific benchmark test"""
        if category not in self.benchmarks:
            raise ValueError(f"Unknown test category: {category}")

        tests = self.benchmarks[category]
        if test_index >= len(tests):
            raise ValueError(f"Test index {test_index} out of range for category {category}")

        test_func = tests[test_index]
        return test_func(**robot_data)

# Example usage
if __name__ == "__main__":
    benchmarks = HumanoidBenchmarks()

    # Example: Run a locomotion test
    robot_data = {
        'robot_position': (9.5, 0.2, 0.8),  # x, y, z position
        'start_time': 0.0,
        'current_time': 15.5
    }

    metrics = benchmarks.run_benchmark(
        TestCategory.LOCOMOTION, 0, robot_data
    )

    print(f"Test Success: {metrics.success}")
    print(f"Completion Time: {metrics.completion_time:.2f}s")
    print(f"Accuracy: {metrics.accuracy:.2f}")
    print(f"Efficiency: {metrics.efficiency:.2f}")
```

## Performance Evaluation and Metrics

### Creating an Evaluation Framework

```python
# evaluation/test_evaluator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
import time
import json
from datetime import datetime
from .humanoid_benchmarks import HumanoidBenchmarks, TestCategory, TestMetrics

class TestEvaluator(Node):
    def __init__(self):
        super().__init__('test_evaluator')

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.robot_pose_sub = self.create_subscription(
            Pose, '/robot_pose', self.robot_pose_callback, 10
        )

        self.robot_twist_sub = self.create_subscription(
            Twist, '/robot_twist', self.robot_twist_callback, 10
        )

        self.test_status_sub = self.create_subscription(
            String, '/test_status', self.test_status_callback, 10
        )

        # Publishers
        self.performance_pub = self.create_publisher(
            String, '/test_performance', 10
        )

        self.metrics_pub = self.create_publisher(
            String, '/test_metrics', 10
        )

        # Timer for evaluation
        self.eval_timer = self.create_timer(0.1, self.evaluate_performance)

        # Initialize benchmarks
        self.benchmarks = HumanoidBenchmarks()

        # Data storage
        self.joint_states = {}
        self.robot_pose = None
        self.robot_twist = None
        self.current_test = None
        self.test_start_time = None
        self.path_history = []
        self.evaluation_data = {}

        # Performance tracking
        self.performance_history = []

        self.get_logger().info('Test Evaluator initialized')

    def joint_state_callback(self, msg):
        """Store joint state data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def robot_pose_callback(self, msg):
        """Store robot pose data"""
        self.robot_pose = (msg.position.x, msg.position.y, msg.position.z)

        # Add to path history for navigation tests
        if len(self.path_history) == 0 or len(self.path_history) % 10 == 0:  # Sample every 10th pose
            self.path_history.append(self.robot_pose)

    def robot_twist_callback(self, msg):
        """Store robot twist (velocity) data"""
        self.robot_twist = (msg.linear.x, msg.linear.y, msg.linear.z,
                           msg.angular.x, msg.angular.y, msg.angular.z)

    def test_status_callback(self, msg):
        """Handle test status updates"""
        status = msg.data
        self.get_logger().info(f'Test status: {status}')

        if 'started' in status:
            self.current_test = status.split()[1]  # Extract test name
            self.test_start_time = time.time()
            self.path_history = []
            self.evaluation_data = {
                'start_pose': self.robot_pose,
                'start_time': self.test_start_time
            }
        elif 'stopped' in status:
            self.finalize_test_evaluation()

    def evaluate_performance(self):
        """Evaluate robot performance during tests"""
        if not self.current_test or not self.test_start_time:
            return

        current_time = time.time()
        test_duration = current_time - self.test_start_time

        # Prepare data for benchmark evaluation
        robot_data = {
            'robot_position': self.robot_pose or (0, 0, 0),
            'joint_states': self.joint_states,
            'robot_angular_velocity': self.robot_twist[3:6] if self.robot_twist else (0, 0, 0),
            'start_time': self.test_start_time,
            'current_time': current_time,
            'path_taken': self.path_history
        }

        # Determine test category and run appropriate benchmark
        if 'locomotion' in self.current_test:
            category = TestCategory.LOCOMOTION
            test_idx = 0  # Use first locomotion test as example
        elif 'manipulation' in self.current_test:
            category = TestCategory.MANIPULATION
            test_idx = 0  # Use first manipulation test as example
        elif 'navigation' in self.current_test:
            category = TestCategory.NAVIGATION
            test_idx = 0  # Use first navigation test as example
        elif 'balance' in self.current_test:
            category = TestCategory.BALANCE
            test_idx = 0  # Use first balance test as example
        else:
            category = TestCategory.LOCOMOTION  # Default
            test_idx = 0

        try:
            metrics = self.benchmarks.run_benchmark(category, test_idx, robot_data)
            self.publish_metrics(metrics, test_duration)
        except Exception as e:
            self.get_logger().error(f'Error evaluating performance: {e}')

    def publish_metrics(self, metrics: TestMetrics, duration: float):
        """Publish test metrics"""
        metrics_dict = {
            'success': metrics.success,
            'completion_time': metrics.completion_time,
            'efficiency': metrics.efficiency,
            'accuracy': metrics.accuracy,
            'energy_consumption': metrics.energy_consumption,
            'safety_score': metrics.safety_score,
            'error_count': metrics.error_count,
            'timestamp': datetime.now().isoformat()
        }

        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics_dict)
        self.metrics_pub.publish(metrics_msg)

        # Log performance
        self.performance_history.append(metrics_dict)

    def finalize_test_evaluation(self):
        """Finalize test evaluation and generate report"""
        if not self.performance_history:
            return

        # Calculate aggregate metrics
        avg_efficiency = sum(m['efficiency'] for m in self.performance_history) / len(self.performance_history)
        avg_accuracy = sum(m['accuracy'] for m in self.performance_history) / len(self.performance_history)
        success_rate = sum(1 for m in self.performance_history if m['success']) / len(self.performance_history)

        report = {
            'test_name': self.current_test,
            'aggregate_metrics': {
                'average_efficiency': avg_efficiency,
                'average_accuracy': avg_accuracy,
                'success_rate': success_rate,
                'total_tests_run': len(self.performance_history)
            },
            'detailed_metrics': self.performance_history,
            'evaluation_timestamp': datetime.now().isoformat()
        }

        # Publish final report
        report_msg = String()
        report_msg.data = json.dumps(report, indent=2)
        self.performance_pub.publish(report_msg)

        # Log summary
        self.get_logger().info(f'Test {self.current_test} completed:')
        self.get_logger().info(f'  Success Rate: {success_rate:.2%}')
        self.get_logger().info(f'  Avg Efficiency: {avg_efficiency:.2f}')
        self.get_logger().info(f'  Avg Accuracy: {avg_accuracy:.2f}')

        # Reset for next test
        self.performance_history = []
        self.current_test = None
        self.test_start_time = None

def main(args=None):
    rclpy.init(args=args)
    evaluator = TestEvaluator()

    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        pass
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Complete Interactive Testing Environment

Create a complete testing environment system:

1. **Create a launch file** for the testing environment:

```python
# launch/interactive_testing.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    test_scenario = LaunchConfiguration('test_scenario')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_test_scenario = DeclareLaunchArgument(
        'test_scenario',
        default_value='locomotion',
        description='Test scenario to run: locomotion, manipulation, navigation, interaction'
    )

    # Launch Gazebo with appropriate world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                get_package_share_directory('my_humanoid_test'),
                'worlds',
                [test_scenario, '_test.world']
            ])
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Environment Test Manager
    env_manager = Node(
        package='my_humanoid_test',
        executable='env_test_manager',
        name='env_test_manager',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Test Evaluator
    test_evaluator = Node(
        package='my_humanoid_test',
        executable='test_evaluator',
        name='test_evaluator',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_test_scenario,
        gazebo,
        robot_state_publisher,
        env_manager,
        test_evaluator
    ])
```

2. **Run the complete testing system:**

```bash
# Build your packages
cd ros2_ws
colcon build --packages-select my_humanoid_test
source install/setup.bash

# Run a locomotion test
ros2 launch my_humanoid_test interactive_testing.launch.py test_scenario:=locomotion

# In another terminal, send test commands
ros2 topic pub /test_command std_msgs/String "data: 'start_locomotion'"

# Monitor test results
ros2 topic echo /test_metrics
ros2 topic echo /test_performance
```

## Troubleshooting Common Environment Issues

### Performance Issues
- **Slow simulation**: Reduce physics update rate or simplify collision meshes
- **High CPU usage**: Limit the number of active sensors or reduce their update rates
- **Memory leaks**: Ensure proper cleanup of spawned objects

### Physics Issues
- **Objects falling through floors**: Check collision geometries and physics parameters
- **Unstable joints**: Adjust joint limits, damping, and stiffness parameters
- **Tunneling effects**: Reduce time step or increase collision mesh resolution

### Sensor Issues
- **No sensor data**: Verify sensor plugins are loaded and topics are connected
- **Noisy data**: Adjust sensor noise parameters in the URDF/SDF
- **Wrong coordinate frames**: Check TF tree and frame names

## Summary

In this chapter, we've explored creating interactive testing environments for humanoid robots. We covered designing various types of environments (locomotion, manipulation, navigation, interaction), implementing dynamic and interactive elements, creating standardized test scenarios and benchmarks, and establishing evaluation frameworks. Interactive testing environments are essential for validating humanoid robot capabilities safely and systematically before real-world deployment.

## Next Steps

- Create your own custom testing environments
- Implement additional benchmark tests
- Integrate with your robot's control system
- Develop automated testing procedures
- Learn about advanced simulation techniques for robotics