---
title: "ROS2 Architecture: Nodes, Topics, Services, Actions"
description: "Understanding the fundamental building blocks of ROS2 - nodes, topics, services, and actions"
learning_objectives:
  - "Understand the ROS2 architecture and its core concepts"
  - "Learn how to create and manage ROS2 nodes"
  - "Understand the difference between topics, services, and actions"
  - "Learn how to use ROS2 tools for debugging and visualization"
---

# ROS2 Architecture: Nodes, Topics, Services, Actions

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the ROS2 architecture and its core concepts
- Create and manage ROS2 nodes
- Differentiate between topics, services, and actions
- Use ROS2 tools for debugging and visualization

## Introduction

The Robot Operating System 2 (ROS2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Understanding the architecture of ROS2 is crucial for building humanoid robots that can effectively communicate between different components.

## Core Concepts

### Nodes

A node is a process that performs computation. ROS2 is designed to be modular at the level of a process, with nodes being the primary unit of computation. Nodes are typically organized into packages for easier code sharing and reuse.

```python
# Example: Basic ROS2 Node
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Messages

Topics are named buses over which nodes exchange messages. The communication is loosely coupled: publishers and subscribers are not aware of each other. Data is transferred asynchronously via a publish/subscribe paradigm.

### Services

Services provide a more tightly coupled, synchronous form of communication. Services allow nodes to send request data to other nodes and receive a response.

### Actions

Actions are a more advanced form of communication that allows for long-running tasks with feedback and the ability to cancel. They are built on top of services and topics.

## Practical Exercise

Create a simple ROS2 package with a publisher and subscriber node that communicate over a topic.

1. Create a new ROS2 package:
```bash
cd ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials
```

2. Create a publisher node in `my_robot_tutorials/my_robot_tutorials/publisher_member_function.py`

3. Create a subscriber node in `my_robot_tutorials/my_robot_tutorials/subscriber_member_function.py`

4. Build the package:
```bash
cd ros2_ws
colcon build --packages-select my_robot_tutorials
```

5. Source the workspace and run the nodes:
```bash
source install/setup.bash
ros2 run my_robot_tutorials publisher_member_function
# In another terminal
ros2 run my_robot_tutorials subscriber_member_function
```

## Summary

In this chapter, we've covered the fundamental building blocks of ROS2 architecture. Understanding nodes, topics, services, and actions is crucial for designing effective communication patterns in your humanoid robot system. In the next chapter, we'll dive deeper into Python programming with ROS2 using the rclpy library.

## Next Steps

- Review the ROS2 documentation on nodes and topics
- Experiment with different message types
- Explore the ROS2 command-line tools for introspection