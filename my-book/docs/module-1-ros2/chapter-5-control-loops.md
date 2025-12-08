---
title: "Basic Control Loop (locomotion/manipulation)"
description: "Implementing fundamental control loops for humanoid robot locomotion and manipulation"
learning_objectives:
  - "Understand the principles of robot control systems"
  - "Implement PID controllers for joint position control"
  - "Create control loops for locomotion and manipulation"
  - "Design state machines for complex robot behaviors"
---

# Basic Control Loop (locomotion/manipulation)

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the principles of robot control systems
- Implement PID controllers for joint position control
- Create control loops for locomotion and manipulation
- Design state machines for complex robot behaviors

## Introduction

Control systems are the backbone of humanoid robotics, enabling precise movement and interaction with the environment. A control loop continuously measures the current state of the robot, compares it to the desired state, and applies corrective actions. This chapter will guide you through implementing fundamental control loops for both locomotion (walking, balancing) and manipulation (grasping, moving objects) tasks in humanoid robots.

## Control System Fundamentals

### Open-Loop vs Closed-Loop Control

In open-loop control, commands are sent to the robot without feedback about the actual state. In closed-loop control, sensors provide feedback about the actual state, allowing the controller to correct errors.

```python
# Open-loop control (no feedback)
def open_loop_move_to_position(target_position):
    # Send command to move to target position
    # No verification of actual position
    pass

# Closed-loop control (with feedback)
def closed_loop_move_to_position(target_position, current_position):
    error = target_position - current_position
    # Apply control law based on error
    control_output = calculate_control_output(error)
    return control_output
```

### Control Loop Structure

A typical control loop follows this structure:

1. **Sense**: Read sensor data (joint positions, IMU, etc.)
2. **Compare**: Calculate error between desired and actual states
3. **Compute**: Apply control law to generate control commands
4. **Actuate**: Send commands to actuators
5. **Repeat**: Continue at a fixed frequency

## PID Control

Proportional-Integral-Derivative (PID) control is the most common control algorithm in robotics. It combines three terms:

- **Proportional (P)**: Responds to current error
- **Integral (I)**: Responds to accumulated past error
- **Derivative (D)**: Responds to rate of error change

```python
class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-float('inf'), float('inf'))):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        self.reset()

    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None

    def compute(self, setpoint, measured_value, dt=None):
        current_time = time.time()

        if dt is None:
            if self.previous_time is None:
                dt = 0.0
            else:
                dt = current_time - self.previous_time

        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Total output
        output = p_term + i_term + d_term

        # Apply output limits
        output = max(self.output_limits[0], min(output, self.output_limits[1]))

        # Store values for next iteration
        self.previous_error = error
        self.previous_time = current_time

        return output
```

### Joint Position Control

Here's an example of using PID control for joint position control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Declare parameters for PID gains
        self.declare_parameter('kp', 10.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.5)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Current joint states
        self.current_positions = {}
        self.current_velocities = {}

        # Target positions
        self.target_positions = {}

        # PID controllers for each joint
        self.pid_controllers = {}

        # Subscribers and publishers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10
        )

        self.trajectory_sub = self.create_subscription(
            JointTrajectory, 'joint_trajectory', self.trajectory_callback, 10
        )

        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        self.get_logger().info('Joint controller initialized')

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_velocities[name] = msg.velocity[i]

    def trajectory_callback(self, msg):
        if len(msg.points) > 0:
            target_point = msg.points[0]
            for i, joint_name in enumerate(msg.joint_names):
                if i < len(target_point.positions):
                    self.target_positions[joint_name] = target_point.positions[i]

                    # Initialize PID controller if not exists
                    if joint_name not in self.pid_controllers:
                        self.pid_controllers[joint_name] = PIDController(
                            self.kp, self.ki, self.kd
                        )

    def control_loop(self):
        if not self.target_positions or not self.current_positions:
            return

        command_msg = Float64MultiArray()
        command_msg.data = []

        for joint_name, target_pos in self.target_positions.items():
            if joint_name in self.current_positions:
                current_pos = self.current_positions[joint_name]

                # Get or create PID controller
                if joint_name not in self.pid_controllers:
                    self.pid_controllers[joint_name] = PIDController(
                        self.kp, self.ki, self.kd
                    )

                # Compute control output
                control_output = self.pid_controllers[joint_name].compute(
                    target_pos, current_pos, 0.01  # dt = 0.01s
                )

                command_msg.data.append(control_output)
                command_msg.layout.dim.append(
                    Float64MultiArray.layout.data_offset
                )

        # Publish joint commands
        self.joint_command_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = JointController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

## Locomotion Control

For humanoid robots, locomotion control involves coordinating multiple joints to achieve stable walking or balancing.

### Balance Control

Balance control uses IMU data to maintain the robot's center of mass:

```python
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Joint command publisher
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 'balance_joint_commands', 10
        )

        # PID controllers for balance
        self.roll_pid = PIDController(15.0, 0.1, 1.0)
        self.pitch_pid = PIDController(15.0, 0.1, 1.0)

        # Target angles (usually 0 for upright position)
        self.target_roll = 0.0
        self.target_pitch = 0.0

        self.current_roll = 0.0
        self.current_pitch = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.01, self.balance_control_loop)

    def imu_callback(self, msg):
        # Convert quaternion to roll/pitch/yaw (simplified)
        # In practice, use proper quaternion to Euler conversion
        self.current_roll = np.arcsin(2.0 * (msg.orientation.w * msg.orientation.x +
                                           msg.orientation.y * msg.orientation.z))
        self.current_pitch = np.arctan2(2.0 * (msg.orientation.w * msg.orientation.y -
                                              msg.orientation.z * msg.orientation.x),
                                       1.0 - 2.0 * (msg.orientation.y * msg.orientation.y +
                                                   msg.orientation.x * msg.orientation.x))

    def balance_control_loop(self):
        # Compute balance corrections
        roll_correction = self.roll_pid.compute(self.target_roll, self.current_roll, 0.01)
        pitch_correction = self.pitch_pid.compute(self.target_pitch, self.current_pitch, 0.01)

        # Apply corrections to appropriate joints
        # This is simplified - real implementation would use inverse kinematics
        balance_commands = Float64MultiArray()
        balance_commands.data = [roll_correction, pitch_correction, 0.0, 0.0]  # hip and ankle adjustments

        self.joint_cmd_pub.publish(balance_commands)
```

## Manipulation Control

Manipulation control focuses on precise end-effector positioning and grasping.

### Cartesian Control

For manipulation tasks, it's often easier to control in Cartesian space:

```python
class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')

        # Subscribe to end-effector pose
        self.pose_sub = self.create_subscription(
            PoseStamped, 'end_effector_pose', self.pose_callback, 10
        )

        # Command publisher
        self.cartesian_cmd_pub = self.create_publisher(
            Twist, 'cartesian_commands', 10
        )

        # PID controllers for Cartesian control
        self.x_pid = PIDController(5.0, 0.05, 0.5)
        self.y_pid = PIDController(5.0, 0.05, 0.5)
        self.z_pid = PIDController(5.0, 0.05, 0.5)

        self.target_pose = None
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def set_target_pose(self, target_pose):
        self.target_pose = target_pose

    def cartesian_control_loop(self):
        if not self.target_pose or not self.current_pose:
            return

        # Calculate position errors
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        dz = self.target_pose.position.z - self.current_pose.position.z

        # Compute control outputs
        vx = self.x_pid.compute(self.target_pose.position.x, self.current_pose.position.x, 0.01)
        vy = self.y_pid.compute(self.target_pose.position.y, self.current_pose.position.y, 0.01)
        vz = self.z_pid.compute(self.target_pose.position.z, self.current_pose.position.z, 0.01)

        # Create and publish command
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz

        self.cartesian_cmd_pub.publish(cmd)
```

## State Machine for Complex Behaviors

For complex humanoid behaviors, state machines provide structure:

```python
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    WALKING = 2
    STANDING_UP = 3
    SITTING_DOWN = 4
    MANIPULATING = 5
    EMERGENCY_STOP = 6

class HumanoidStateMachine(Node):
    def __init__(self):
        super().__init__('humanoid_state_machine')

        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE

        # Control loop timer
        self.state_timer = self.create_timer(0.02, self.state_machine_loop)  # 50 Hz

        # Initialize controllers for different states
        self.balance_controller = BalanceController()
        self.joint_controller = JointController()
        self.walk_controller = WalkController()  # Custom walking controller

        self.get_logger().info(f'Starting in state: {self.current_state}')

    def state_machine_loop(self):
        # Process state transitions based on conditions
        self.check_state_transitions()

        # Execute behavior for current state
        if self.current_state == RobotState.IDLE:
            self.execute_idle_behavior()
        elif self.current_state == RobotState.WALKING:
            self.execute_walking_behavior()
        elif self.current_state == RobotState.STANDING_UP:
            self.execute_standing_up_behavior()
        elif self.current_state == RobotState.MANIPULATING:
            self.execute_manipulation_behavior()
        elif self.current_state == RobotState.EMERGENCY_STOP:
            self.execute_emergency_stop()

    def check_state_transitions(self):
        # Example state transition logic
        if self.is_falling() and self.current_state != RobotState.EMERGENCY_STOP:
            self.previous_state = self.current_state
            self.current_state = RobotState.EMERGENCY_STOP
        elif self.current_state == RobotState.EMERGENCY_STOP and not self.is_falling():
            self.current_state = self.previous_state
        elif self.should_stand() and self.current_state == RobotState.IDLE:
            self.current_state = RobotState.STANDING_UP

    def is_falling(self):
        # Check IMU data for falling condition
        # Implementation depends on your specific robot
        return False

    def should_stand(self):
        # Check if robot should transition to standing
        return True

    def execute_idle_behavior(self):
        # Maintain balance in idle position
        self.balance_controller.balance()

    def execute_walking_behavior(self):
        # Execute walking pattern
        self.walk_controller.step()
        self.balance_controller.balance()

    def execute_standing_up_behavior(self):
        # Execute standing up motion sequence
        pass

    def execute_manipulation_behavior(self):
        # Execute manipulation task
        pass

    def execute_emergency_stop(self):
        # Stop all motion, activate safety behaviors
        self.joint_controller.emergency_stop()
```

## Practical Exercise: Implement a Simple Walking Controller

Create a basic walking controller that alternates between left and right leg support:

1. Create a walking controller node that generates walking patterns
2. Implement a simple 3-phase walking gait (left support, double support, right support)
3. Use PID controllers to track the generated trajectories

```python
class SimpleWalkingController(Node):
    def __init__(self):
        super().__init__('simple_walking_controller')

        # Walking parameters
        self.step_length = 0.1  # meters
        self.step_height = 0.05  # meters
        self.step_duration = 1.0  # seconds
        self.zmp_offset = 0.05  # Zero Moment Point offset

        # Phase tracking
        self.current_phase = 0  # 0-2 for the 3 phases
        self.phase_time = 0.0

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 'walking_trajectory', 10
        )

        # Timer for walking control
        self.walk_timer = self.create_timer(0.02, self.walk_control_loop)  # 50 Hz

    def walk_control_loop(self):
        # Update phase based on time
        self.phase_time += 0.02  # dt

        if self.phase_time >= self.step_duration:
            self.phase_time = 0.0
            self.current_phase = (self.current_phase + 1) % 3

        # Generate walking trajectory based on current phase
        trajectory = self.generate_walking_trajectory()
        self.trajectory_pub.publish(trajectory)

    def generate_walking_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle'
        ]

        point = JointTrajectoryPoint()

        # Simplified walking pattern
        if self.current_phase == 0:  # Left support
            # Keep left leg straight, move right leg forward
            point.positions = [0.0, 0.0, 0.0, 0.1, -0.2, 0.05]  # Right leg lifted and forward
        elif self.current_phase == 1:  # Double support
            # Both legs support, prepare for step
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:  # Right support
            # Keep right leg straight, move left leg forward
            point.positions = [0.1, -0.2, 0.05, 0.0, 0.0, 0.0]  # Left leg lifted and forward

        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 20000000  # 20ms

        trajectory.points.append(point)
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    walker = SimpleWalkingController()
    rclpy.spin(walker)
    walker.destroy_node()
    rclpy.shutdown()
```

## Summary

In this chapter, we've explored the fundamental concepts of control systems for humanoid robots. We covered PID control, joint position control, balance control for locomotion, Cartesian control for manipulation, and state machines for complex behaviors. These control techniques form the foundation for creating responsive and stable humanoid robots.

## Next Steps

- Implement the walking controller example in your ROS2 workspace
- Experiment with different PID parameters to see their effect
- Add sensor feedback to improve control performance
- Explore advanced control techniques like model predictive control
- Learn about inverse kinematics for more sophisticated motion control