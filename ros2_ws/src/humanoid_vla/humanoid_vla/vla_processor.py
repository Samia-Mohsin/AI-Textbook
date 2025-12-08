#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import json
import threading

class VLAProcessor(Node):
    def __init__(self):
        super().__init__('vla_processor')

        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'vla_command', self.command_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.action_pub = self.create_publisher(String, 'action_command', 10)
        self.status_pub = self.create_publisher(String, 'vla_status', 10)

        # Internal state
        self.latest_image = None
        self.command_queue = []
        self.processing_lock = threading.Lock()

        self.get_logger().info("VLA Processor initialized")

    def command_callback(self, msg):
        """Handle incoming VLA commands"""
        command = msg.data
        self.get_logger().info(f"Received VLA command: {command}")

        # Add command to processing queue
        with self.processing_lock:
            self.command_queue.append(command)

        # Process commands in a separate thread to avoid blocking
        processing_thread = threading.Thread(target=self.process_command_queue)
        processing_thread.start()

    def image_callback(self, msg):
        """Handle incoming images for vision processing"""
        self.latest_image = msg

    def process_command_queue(self):
        """Process commands in the queue"""
        with self.processing_lock:
            if not self.command_queue:
                return
            command = self.command_queue.pop(0)

        # Publish status
        status_msg = String()
        status_msg.data = f"PROCESSING: {command}"
        self.status_pub.publish(status_msg)

        # Process the command
        try:
            action_commands = self.process_vla_command(command)

            # Publish resulting action commands
            for action_cmd in action_commands:
                action_msg = String()
                action_msg.data = action_cmd
                self.action_pub.publish(action_msg)

            self.get_logger().info(f"Processed command: {command} -> {action_commands}")

            # Publish success status
            status_msg = String()
            status_msg.data = f"PROCESSED: {command}"
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing command {command}: {str(e)}")
            status_msg = String()
            status_msg.data = f"ERROR: {str(e)}"
            self.status_pub.publish(status_msg)

    def process_vla_command(self, command):
        """Process a VLA command and return action commands"""
        # This is where the complex VLA processing happens
        # In a real implementation, this would involve:
        # 1. Natural language understanding
        # 2. Vision processing to understand the environment
        # 3. Action planning based on the command and environment
        # 4. Generation of executable robot actions

        # For this example, we'll use simple pattern matching
        command_lower = command.lower()
        actions = []

        if 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract destination
            import re
            matches = re.findall(r'to\s+([a-zA-Z\s]+?)(?:\s|$|,)', command_lower)
            if matches:
                destination = matches[-1].strip()
                actions.append(f"navigation:goto:{destination}")

        if 'pick up' in command_lower or 'grasp' in command_lower:
            # Extract object
            import re
            matches = re.findall(r'(?:pick up|grasp|get|take)\s+([a-zA-Z\s]+?)(?:\s|$|,)', command_lower)
            if matches:
                obj = matches[-1].strip()
                actions.append(f"manipulation:pick_up:{obj}")

        if 'move' in command_lower and ('left' in command_lower or 'right' in command_lower):
            if 'left' in command_lower:
                actions.append("motion:turn_left:90")
            elif 'right' in command_lower:
                actions.append("motion:turn_right:90")

        # If no specific actions were identified, default to a simple action
        if not actions:
            actions.append(f"unknown_command:{command}")

        return actions

def main(args=None):
    rclpy.init(args=args)

    node = VLAProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down VLA processor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()