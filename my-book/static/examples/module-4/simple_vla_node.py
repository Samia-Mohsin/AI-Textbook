#!/usr/bin/env python3

"""
Simple Vision-Language-Action node example
This demonstrates the basic concept of connecting language understanding
with visual perception and action execution
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import json


class SimpleVLANode(Node):
    def __init__(self):
        super().__init__('simple_vla_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to command and image topics
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

        # Publisher for robot motion commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher for status updates
        self.status_pub = self.create_publisher(
            String,
            '/vla/status',
            10
        )

        # Internal state
        self.latest_image = None
        self.command_queue = []

        self.get_logger().info("Simple VLA Node initialized")

    def command_callback(self, msg):
        """Handle incoming language commands"""
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")

        # Simple command parsing
        if 'forward' in command or 'go' in command:
            self.move_forward()
        elif 'backward' in command or 'back' in command:
            self.move_backward()
        elif 'left' in command:
            self.turn_left()
        elif 'right' in command:
            self.turn_right()
        elif 'stop' in command:
            self.stop_robot()
        else:
            self.get_logger().info(f"Unknown command: {command}")

    def image_callback(self, msg):
        """Handle incoming images"""
        self.latest_image = msg
        # Process image if needed
        self.process_image()

    def process_image(self):
        """Process the latest image for perception"""
        if self.latest_image is None:
            return

        try:
            # Convert to OpenCV for processing
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")

            # Simple example: detect red objects (for "find the red cup" type commands)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define range for red color
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            lower_red = np.array([170, 50, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)

            mask = mask1 + mask2

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # If red object detected, update status
            if len(contours) > 0:
                status_msg = String()
                status_msg.data = "RED_OBJECT_DETECTED"
                self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def move_forward(self):
        """Move the robot forward"""
        twist = Twist()
        twist.linear.x = 0.2  # Forward at 0.2 m/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Moving forward")

    def move_backward(self):
        """Move the robot backward"""
        twist = Twist()
        twist.linear.x = -0.2  # Backward at 0.2 m/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Moving backward")

    def turn_left(self):
        """Turn the robot left"""
        twist = Twist()
        twist.angular.z = 0.5  # Turn left at 0.5 rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Turning left")

    def turn_right(self):
        """Turn the robot right"""
        twist = Twist()
        twist.angular.z = -0.5  # Turn right at 0.5 rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Turning right")

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Stopping robot")


def main(args=None):
    rclpy.init(args=args)

    vla_node = SimpleVLANode()

    try:
        rclpy.spin(vla_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()