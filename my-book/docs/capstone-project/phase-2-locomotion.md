---
sidebar_position: 3
---

# Phase 2: Basic Locomotion & Sensor Integration

## Objectives

In this phase, you will:
- Implement basic locomotion controls for your humanoid robot
- Integrate essential sensors for balance and navigation
- Develop joint controllers for stable movement
- Test locomotion in simulation environment

## Locomotion Requirements

Your humanoid robot should be able to:
- Stand up from a resting position
- Maintain balance in a standing position
- Move forward, backward, and sideways
- Turn left and right
- Demonstrate basic walking gait (for advanced implementations)

## Control Architecture

Implement a hierarchical control system:

1. **High-Level Planner**: Determines overall movement direction and goals
2. **Gait Generator**: Creates timing and coordination patterns for locomotion
3. **Balance Controller**: Maintains center of mass within support polygon
4. **Joint Controllers**: Low-level control of individual joint positions/velocities

## Implementation Steps

### Step 1: Install Required Packages

```bash
# Install joint state controller
sudo apt install ros-humble-joint-state-controller

# Install position and velocity controllers
sudo apt install ros-humble-joint-trajectory-controllers

# Install robot state publisher
sudo apt install ros-humble-robot-state-publisher
```

### Step 2: Configure Controllers

Create a ROS2 control configuration file (`config/controllers.yaml`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    humanoid_arm_controller:
      type: position_controllers/JointGroupPositionController

    humanoid_leg_controller:
      type: position_controllers/JointGroupPositionController

    humanoid_torque_controller:
      type: effort_controllers/JointGroupEffortController

humanoid_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint
      - right_shoulder_joint
      - right_elbow_joint

humanoid_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint
```

### Step 3: Launch Controller Setup

Create a launch file (`launch/controller.launch.py`):

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Load controller configurations
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("humanoid_description"), "config", "controllers.yaml"]
    )

    # Robot state publisher node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz and joint state broadcaster after the robot description is published
    delay_joint_state_broadcaster_after_robot_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    # Position controllers
    humanoid_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["humanoid_arm_controller", "--controller-manager", "/controller_manager"],
    )

    humanoid_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["humanoid_leg_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        delay_joint_state_broadcaster_after_robot_control_node,
        humanoid_arm_controller_spawner,
        humanoid_leg_controller_spawner,
    ])
```

### Step 4: Implement Balance Controller

Create a simple balance controller node (`src/balance_controller.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import numpy as np


class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        # PID parameters for balance control
        self.kp = 5.0  # Proportional gain
        self.ki = 0.1  # Integral gain  
        self.kd = 0.5  # Derivative gain
        
        # Error accumulators
        self.roll_error_sum = 0.0
        self.pitch_error_sum = 0.0
        self.prev_roll_error = 0.0
        self.prev_pitch_error = 0.0
        
        # Target angles (should be close to 0 for balance)
        self.target_roll = 0.0
        self.target_pitch = 0.0
        
        # Publishers and subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/humanoid_leg_controller/commands',
            10
        )
        
        # Control timer
        self.timer = self.create_timer(0.01, self.balance_control_loop)  # 100Hz
        
        self.get_logger().info('Balance controller initialized')

    def imu_callback(self, msg):
        # Extract roll and pitch from quaternion
        orientation = msg.orientation
        self.roll, self.pitch, self.yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

    def quaternion_to_euler(self, x, y, z, w):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def balance_control_loop(self):
        # Calculate errors
        roll_error = self.target_roll - self.roll
        pitch_error = self.target_pitch - self.pitch
        
        # Update integral terms
        self.roll_error_sum += roll_error * 0.01  # dt = 0.01s
        self.pitch_error_sum += pitch_error * 0.01
        
        # Calculate derivatives
        roll_error_deriv = (roll_error - self.prev_roll_error) / 0.01
        pitch_error_deriv = (pitch_error - self.prev_pitch_error) / 0.01
        
        # PID control
        roll_control = (self.kp * roll_error + 
                       self.ki * self.roll_error_sum + 
                       self.kd * roll_error_deriv)
        
        pitch_control = (self.kp * pitch_error + 
                        self.ki * self.pitch_error_sum + 
                        self.kd * pitch_error_deriv)
        
        # Update previous errors
        self.prev_roll_error = roll_error
        self.prev_pitch_error = pitch_error
        
        # Generate joint commands based on balance corrections
        joint_commands = Float64MultiArray()
        
        # Simple mapping: adjust ankle and hip joints to maintain balance
        commands = [
            pitch_control * 0.5,   # left_hip
            roll_control * 0.1,    # left_knee
            -roll_control * 0.5,   # left_ankle
            -pitch_control * 0.5,  # right_hip
            -roll_control * 0.1,   # right_knee
            roll_control * 0.5     # right_ankle
        ]
        
        joint_commands.data = commands
        self.joint_cmd_pub.publish(joint_commands)


def main(args=None):
    rclpy.init(args=args)
    balance_controller = BalanceController()
    
    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        pass
    
    balance_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Integrate with Gazebo

Update your launch file to include both robot spawning and controller launching:

```python
# launch/spawn_and_control.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Gazebo with the robot world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )

    # Robot spawn node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.0'
        ],
        output='screen'
    )

    # Delay spawn after Gazebo is ready
    delay_spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo,
            on_start=[spawn_entity],
        )
    )

    # Include controller launch
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'launch',
                'controller.launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        gazebo,
        delay_spawn_after_gazebo,
        controller_launch
    ])
```

## Testing and Validation

### 1. Verify Joint States
```bash
# Check if joint states are being published
ros2 topic echo /joint_states

# Check if controllers are running
ros2 control list_controllers
```

### 2. Test Individual Joint Control
```bash
# Send position commands to a specific joint
ros2 topic pub /humanoid_leg_controller/commands std_msgs/Float64MultiArray "data: [0.1, 0.0, 0.0, -0.1, 0.0, 0.0]"
```

### 3. Run Balance Controller Test
```bash
# Launch the complete system
ros2 launch humanoid_description spawn_and_control.launch.py

# In another terminal, run the balance controller
ros2 run humanoid_control balance_controller
```

## Deliverables

- Working joint controllers for robot locomotion
- Balance controller that maintains upright position
- Launch files to start the complete system
- Documentation of control parameters and tuning process
- Video demonstration of basic locomotion in simulation

## Advanced Extensions

For more advanced implementations, consider:

- Implementing a walking gait pattern using Central Pattern Generators (CPGs)
- Adding footstep planning for stable walking
- Incorporating ZMP (Zero Moment Point) control for balance
- Developing dynamic walking patterns instead of static poses

## Next Phase

After completing Phase 2, proceed to Phase 3 where you'll implement the perception system using NVIDIA Isaac ROS for computer vision.