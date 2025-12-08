#!/usr/bin/env python3

"""
Advanced Vision-Language-Action node
This demonstrates a more sophisticated VLA system with planning and execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import json
import threading
import time
import math


class AdvancedVLANode(Node):
    def __init__(self):
        super().__init__('advanced_vla_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to various topics
        self.command_sub = self.create_subscription(
            String,
            '/vla/command',
            self.command_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/vla/status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Internal state
        self.latest_image = None
        self.latest_laser = None
        self.robot_pose = None
        self.command_queue = []
        self.is_executing = False
        self.safety_enabled = True

        self.get_logger().info("Advanced VLA Node initialized")

    def command_callback(self, msg):
        """Handle incoming language commands"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Parse and execute command in separate thread
        if not self.is_executing:
            execution_thread = threading.Thread(
                target=self.execute_command,
                args=(command,)
            )
            execution_thread.start()
        else:
            self.get_logger().warn("Command queueing not implemented, ignoring command")

    def image_callback(self, msg):
        """Handle incoming images"""
        self.latest_image = msg

    def laser_callback(self, msg):
        """Handle laser scan data for safety"""
        self.latest_laser = msg

        # Check for safety violations
        if self.safety_enabled:
            min_distance = min(msg.ranges) if msg.ranges else float('inf')
            if min_distance < 0.5:  # 50cm threshold
                self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m, stopping robot")
                self.stop_robot()
                self.publish_status(f"SAFETY_STOP: Obstacle at {min_distance:.2f}m")

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.robot_pose = msg.pose.pose

    def execute_command(self, command):
        """Execute a command with safety checks"""
        self.is_executing = True
        self.publish_status(f"EXECUTING: {command}")

        try:
            # Parse the command to determine action
            action_plan = self.parse_command(command)

            # Execute the plan
            success = self.execute_plan(action_plan)

            if success:
                self.publish_status(f"SUCCESS: {command}")
            else:
                self.publish_status(f"FAILED: {command}")

        except Exception as e:
            self.get_logger().error(f"Command execution error: {str(e)}")
            self.publish_status(f"ERROR: {str(e)}")
        finally:
            self.is_executing = False

    def parse_command(self, command):
        """Parse a command and return an action plan"""
        command_lower = command.lower()
        plan = []

        # Simple command parsing
        if 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract destination
            import re
            matches = re.findall(r'to\s+([a-zA-Z\s]+?)(?:\s|$|,)', command_lower)
            if matches:
                destination = matches[-1].strip()
                plan.append({
                    'action': 'navigate',
                    'destination': destination,
                    'description': f'Navigate to {destination}'
                })

        elif 'find' in command_lower or 'look for' in command_lower:
            # Extract object to find
            import re
            matches = re.findall(r'(?:find|look for)\s+([a-zA-Z\s]+?)(?:\s|$|,)', command_lower)
            if matches:
                obj = matches[-1].strip()
                plan.append({
                    'action': 'find_object',
                    'object': obj,
                    'description': f'Find {obj}'
                })

        elif 'pick up' in command_lower or 'grasp' in command_lower:
            # Extract object to pick up
            import re
            matches = re.findall(r'(?:pick up|grasp|get)\s+([a-zA-Z\s]+?)(?:\s|$|,)', command_lower)
            if matches:
                obj = matches[-1].strip()
                plan.append({
                    'action': 'pick_up',
                    'object': obj,
                    'description': f'Pick up {obj}'
                })

        else:
            # Default to a simple action
            plan.append({
                'action': 'unknown',
                'command': command,
                'description': f'Unknown command: {command}'
            })

        return plan

    def execute_plan(self, plan):
        """Execute a plan of actions"""
        for action in plan:
            self.get_logger().info(f"Executing: {action['description']}")

            action_type = action['action']
            if action_type == 'navigate':
                success = self.execute_navigation(action)
            elif action_type == 'find_object':
                success = self.execute_find_object(action)
            elif action_type == 'pick_up':
                success = self.execute_pick_up(action)
            else:
                success = False
                self.get_logger().warn(f"Unknown action type: {action_type}")

            if not success:
                self.get_logger().error(f"Action failed: {action['description']}")
                return False

        return True

    def execute_navigation(self, action):
        """Execute navigation action"""
        destination = action['destination']

        # In a real system, you would have a map of named locations
        # For this example, we'll use a simple mapping
        locations = {
            'kitchen': (2.0, 1.0, 0.0),
            'living room': (0.0, 2.0, 1.57),
            'bedroom': (-1.0, -1.0, 3.14),
            'office': (3.0, -1.0, -1.57)
        }

        if destination in locations:
            x, y, theta = locations[destination]
            return self.navigate_to_pose(x, y, theta)
        else:
            self.get_logger().error(f"Unknown destination: {destination}")
            return False

    def navigate_to_pose(self, x, y, theta):
        """Navigate to a specific pose using Nav2"""
        # Wait for navigation server
        self.nav_client.wait_for_server()

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from math import sin, cos
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)

        # Send goal
        goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=30.0)

        if goal_future.result() is None:
            self.get_logger().error("Navigation goal timed out")
            return False

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result().result
        return result.error_code == 0

    def execute_find_object(self, action):
        """Execute object finding action"""
        obj = action['object']
        self.get_logger().info(f"Looking for object: {obj}")

        # Process the latest image to find the object
        if self.latest_image is None:
            self.get_logger().error("No image available for object detection")
            return False

        # For this example, we'll just return success
        # In a real system, you would run object detection
        self.get_logger().info(f"Object '{obj}' detection would happen here")
        return True

    def execute_pick_up(self, action):
        """Execute pick up action"""
        obj = action['object']
        self.get_logger().info(f"Attempting to pick up: {obj}")

        # For this example, we'll just return success
        # In a real system, you would control the robot's manipulator
        self.get_logger().info(f"Pick up action for '{obj}' would happen here")
        return True

    def stop_robot(self):
        """Stop the robot immediately"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def publish_status(self, status):
        """Publish status message"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    vla_node = AdvancedVLANode()

    try:
        rclpy.spin(vla_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()