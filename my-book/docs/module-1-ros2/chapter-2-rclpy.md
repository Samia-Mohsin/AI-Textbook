---
title: "Python rclpy Control"
description: "Deep dive into using Python for ROS2 development with the rclpy library"
learning_objectives:
  - "Master the rclpy library for Python ROS2 development"
  - "Create custom message types and use them in nodes"
  - "Implement services and clients in Python"
  - "Build action clients and servers with rclpy"
---

# Python rclpy Control

## Learning Objectives

By the end of this chapter, you will be able to:
- Master the rclpy library for Python ROS2 development
- Create custom message types and use them in nodes
- Implement services and clients in Python
- Build action clients and servers with rclpy

## Introduction

The `rclpy` library is the Python client library for ROS 2. It provides a high-level Python API that allows you to create ROS 2 nodes, publish and subscribe to topics, make service calls, and create action clients and servers. This chapter will guide you through the essential concepts and patterns for using rclpy effectively in your humanoid robotics projects.

## Core rclpy Concepts

### Node Creation and Management

The `Node` class is the fundamental building block in rclpy. All ROS 2 functionality is accessed through a node instance.

```python
import rclpy
from rclpy.node import Node

class MyRobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create parameters with default values
        self.declare_parameter('max_velocity', 1.0)
        self.max_velocity = self.get_parameter('max_velocity').value

        # Log information
        self.get_logger().info(f'Max velocity set to: {self.max_velocity}')
```

### Publishers and Subscribers

Creating publishers and subscribers is straightforward with rclpy:

```python
from std_msgs.msg import String, Float64

class RobotCommunication(Node):
    def __init__(self):
        super().__init__('robot_comm')

        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            Float64,
            'motor_speed',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received motor speed: {msg.data}')
```

### Services and Clients

Services provide synchronous request/response communication:

```python
from example_interfaces.srv import AddTwoInts

class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Practical Exercise: Advanced Robot Controller

Create a more complex robot controller that uses multiple communication patterns:

1. Create a publisher that publishes joint states
2. Create a subscriber that listens to sensor data
3. Create a service that accepts movement commands
4. Create a client that requests path planning

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from example_interfaces.srv import Trigger

class AdvancedRobotController(Node):
    def __init__(self):
        super().__init__('advanced_robot_controller')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_cmd_pub = self.create_publisher(Float64MultiArray, 'motor_commands', 10)

        # Subscribers
        self.sensor_sub = self.create_subscription(
            Float64MultiArray,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Services
        self.move_srv = self.create_service(Trigger, 'move_robot', self.move_robot_callback)

        # Timers
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 joints
        self.target_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def sensor_callback(self, msg):
        # Process sensor data
        self.get_logger().info(f'Received sensor data: {msg.data}')

    def move_robot_callback(self, request, response):
        # Move robot to target positions
        self.target_positions = [1.0, 0.5, -0.5, 0.0, 0.5, -0.5]
        response.success = True
        response.message = 'Robot movement command received'
        self.get_logger().info('Robot movement command executed')
        return response

    def control_loop(self):
        # Simple PD controller
        for i in range(len(self.joint_positions)):
            error = self.target_positions[i] - self.joint_positions[i]
            self.joint_positions[i] += 0.1 * error  # Simple proportional control

        # Publish joint states
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = self.joint_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Message Types

Creating custom message types allows for more complex data structures:

1. Create a directory `msg` in your package
2. Create a file `JointCommand.msg`:
```
string joint_name
float64 position
float64 velocity
float64 effort
```

3. Update your `package.xml` to include the message generation dependency
4. Update your `setup.py` to include the message files

## Summary

In this chapter, we've explored the powerful features of rclpy for Python-based ROS2 development. We've covered publishers, subscribers, services, and how to structure complex robot controllers. The practical exercise demonstrates how to combine these concepts into a functional robot control system.

## Next Steps

- Implement the practical exercise in your ROS2 workspace
- Create custom message types for your specific robot
- Explore action libraries for advanced robotics behaviors
- Learn about ROS2 launch files for managing multiple nodes