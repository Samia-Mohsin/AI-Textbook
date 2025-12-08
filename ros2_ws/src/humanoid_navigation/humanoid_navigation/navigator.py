#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import math

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriber for navigation commands
        self.nav_sub = self.create_subscription(
            String, 'navigation_command', self.nav_command_callback, 10
        )

        # Publisher for navigation status
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)

        self.get_logger().info("Humanoid Navigator initialized")

    def nav_command_callback(self, msg):
        """Handle navigation commands"""
        command = msg.data
        self.get_logger().info(f"Received navigation command: {command}")

        # Parse command (simple parsing for demonstration)
        if command.startswith("goto:"):
            # Format: "goto:x,y,theta" or "goto:location_name"
            coords = command[5:].split(',')
            if len(coords) == 3:
                try:
                    x = float(coords[0])
                    y = float(coords[1])
                    theta = float(coords[2])
                    self.navigate_to_pose(x, y, theta)
                except ValueError:
                    self.get_logger().error(f"Invalid coordinates: {command}")
            else:
                # Handle named locations
                self.navigate_to_location(command[5:])

    def navigate_to_pose(self, x, y, theta):
        """Navigate to a specific pose"""
        self.get_logger().info(f"Navigating to pose: ({x}, {y}, {theta})")

        # Wait for navigation server
        self.nav_client.wait_for_server()

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
        from math import sin, cos
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)

        # Send goal
        goal_handle_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Publish status
        status_msg = String()
        status_msg.data = f"NAVIGATING: ({x}, {y})"
        self.status_pub.publish(status_msg)

        # Wait for result
        goal_handle_future.add_done_callback(self.goal_response_callback)

    def navigate_to_location(self, location_name):
        """Navigate to a named location"""
        # Define known locations
        locations = {
            'kitchen': (2.0, 1.0, 0.0),
            'living_room': (0.0, 2.0, 1.57),
            'bedroom': (-1.0, -1.0, 3.14),
            'office': (3.0, -1.0, -1.57)
        }

        if location_name in locations:
            x, y, theta = locations[location_name]
            self.navigate_to_pose(x, y, theta)
        else:
            self.get_logger().error(f"Unknown location: {location_name}")

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        self.get_logger().info(
            f'Navigation feedback: {feedback_msg.distance_remaining:.2f}m remaining'
        )

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Navigation succeeded')
            status_msg = String()
            status_msg.data = "NAVIGATION_SUCCESS"
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().error(f'Navigation failed with error code: {result.error_code}')
            status_msg = String()
            status_msg.data = f"NAVIGATION_FAILED: {result.error_code}"
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)

    node = HumanoidNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down humanoid navigator")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()