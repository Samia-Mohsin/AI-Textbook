#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from nav2_msgs.action import NavigateToPose
from humanoid_interfaces.action import ManipulateObject
from humanoid_interfaces.msg import CommandFeedback

import threading
import time
import json

class AutonomousHumanoidNode(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_node')

        # Initialize all subsystems
        self.initialize_subsystems()

        # Setup ROS2 interfaces
        self.setup_ros_interfaces()

        # State management
        self.current_state = 'idle'
        self.is_executing = False
        self.execution_thread = None

    def initialize_subsystems(self):
        """Initialize all subsystems required for autonomous operation"""
        # Initialize cognitive planner
        self.cognitive_planner = CognitivePlanner(self)

        # Initialize perception system
        self.perception_system = PerceptionSystem(self)

        # Initialize safety system
        self.safety_system = SafetySystem(self)

        # Initialize action execution
        self.action_executor = ActionExecutor(self)

    def setup_ros_interfaces(self):
        """Setup all ROS2 publishers, subscribers, and action clients"""
        # Publishers
        self.status_pub = self.create_publisher(String, '/humanoid/status', 10)
        self.feedback_pub = self.create_publisher(CommandFeedback, '/humanoid/feedback', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/vla/command', self.command_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, ManipulateObject, 'manipulate_object')

    def command_callback(self, msg: String):
        """Handle incoming VLA commands"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Validate command safety first
        if not self.safety_system.validate_command(command):
            self.get_logger().error(f"Unsafe command blocked: {command}")
            return

        # Process command in separate thread to avoid blocking
        self.execution_thread = threading.Thread(
            target=self.execute_command_threaded,
            args=(command,)
        )
        self.execution_thread.start()

    def execute_command_threaded(self, command: str):
        """Execute command in a separate thread"""
        if self.is_executing:
            self.get_logger().warn("Command execution already in progress, ignoring new command")
            return

        self.is_executing = True
        self.current_state = 'planning'

        try:
            # Update status
            status_msg = String()
            status_msg.data = f"PLANNING: {command}"
            self.status_pub.publish(status_msg)

            # Plan the command
            plan = self.cognitive_planner.generate_plan(command)

            # Execute the plan
            self.current_state = 'executing'
            status_msg.data = f"EXECUTING: {command}"
            self.status_pub.publish(status_msg)

            success = self.execute_plan(plan)

            # Report results
            if success:
                self.get_logger().info(f"Command completed successfully: {command}")
                status_msg.data = f"SUCCESS: {command}"
            else:
                self.get_logger().error(f"Command execution failed: {command}")
                status_msg.data = f"FAILED: {command}"

            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Command execution error: {str(e)}")
            status_msg = String()
            status_msg.data = f"ERROR: {str(e)}"
            self.status_pub.publish(status_msg)
        finally:
            self.is_executing = False
            self.current_state = 'idle'

    def execute_plan(self, plan):
        """Execute a planned sequence of actions"""
        for i, action in enumerate(plan):
            # Check safety before each action
            if not self.safety_system.is_safe_to_proceed():
                self.get_logger().error("Safety check failed, stopping execution")
                return False

            # Execute action
            success = self.action_executor.execute_action(action)

            if not success:
                self.get_logger().error(f"Action failed: {action}")
                return False

            # Small delay between actions for safety
            time.sleep(0.1)

        return True

    def image_callback(self, msg: Image):
        """Process incoming images for perception"""
        self.perception_system.update_image(msg)

    def laser_callback(self, msg: LaserScan):
        """Process laser scan data for navigation and safety"""
        self.perception_system.update_laser_scan(msg)
        self.safety_system.update_laser_data(msg)

    def odom_callback(self, msg: Odometry):
        """Update robot's position and orientation"""
        self.perception_system.update_odometry(msg)
        self.safety_system.update_robot_pose(msg.pose.pose)

class CognitivePlanner:
    """Cognitive planner that uses LLM to decompose commands into actions"""

    def __init__(self, node):
        self.node = node
        # Initialize OpenAI client (or use local LLM)
        # self.client = OpenAI(api_key="your-api-key")

    def generate_plan(self, command: str):
        """Generate an action plan for the given command"""
        # For this implementation, we'll use a rule-based approach
        # but the structure allows for LLM integration

        plan = []

        # Simple command parsing for demonstration
        command_lower = command.lower()

        if 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract destination
            destination = self.extract_destination(command)
            plan.append({
                'type': 'navigation',
                'destination': destination,
                'description': f'Navigate to {destination}'
            })

        if 'pick up' in command_lower or 'grasp' in command_lower:
            # Extract object
            obj = self.extract_object(command)
            plan.append({
                'type': 'manipulation',
                'action': 'pick_up',
                'object': obj,
                'description': f'Pick up {obj}'
            })

        if 'bring to' in command_lower or 'deliver to' in command_lower:
            # Extract destination
            destination = self.extract_destination(command)
            plan.append({
                'type': 'navigation',
                'destination': destination,
                'description': f'Navigate to {destination} with object'
            })

        if 'place' in command_lower or 'put' in command_lower:
            plan.append({
                'type': 'manipulation',
                'action': 'place',
                'description': 'Place object at destination'
            })

        return plan

    def extract_destination(self, command: str) -> str:
        """Extract destination from command"""
        import re
        matches = re.findall(r'to\s+([a-zA-Z\s]+?)(?:\s|$|,)', command.lower())
        if matches:
            return matches[-1].strip()
        return 'unknown_location'

    def extract_object(self, command: str) -> str:
        """Extract object from command"""
        import re
        matches = re.findall(r'(?:pick up|grasp|get|take)\s+([a-zA-Z\s]+?)(?:\s|$|,)', command.lower())
        if matches:
            return matches[-1].strip()
        return 'unknown_object'

class PerceptionSystem:
    """Perception system for object detection and environment understanding"""

    def __init__(self, node):
        self.node = node
        try:
            from cv_bridge import CvBridge
            self.cv_bridge = CvBridge()
        except ImportError:
            self.node.get_logger().warn("cv_bridge not available")
            self.cv_bridge = None

        # Initialize object detector
        try:
            from ultralytics import YOLO
            self.object_detector = YOLO('yolov8n.pt')  # or your preferred model
        except ImportError:
            self.node.get_logger().warn("YOLO not available")
            self.object_detector = None

        self.latest_image = None
        self.latest_laser = None
        self.robot_pose = None
        self.detected_objects = []

    def update_image(self, image_msg):
        """Update with new image data"""
        self.latest_image = image_msg

    def update_laser_scan(self, laser_msg):
        """Update with new laser scan data"""
        self.latest_laser = laser_msg

    def update_odometry(self, odom_msg):
        """Update with new odometry data"""
        self.robot_pose = odom_msg.pose.pose

    def detect_objects(self):
        """Detect objects in the current image"""
        if self.latest_image is None or self.object_detector is None or self.cv_bridge is None:
            return []

        # Convert ROS image to OpenCV format
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.node.get_logger().error(f"Error converting image: {e}")
            return []

        # Run object detection
        results = self.object_detector(cv_image)

        objects = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())

                    # Get class name (COCO dataset names)
                    class_names = self.object_detector.names
                    class_name = class_names[cls] if cls < len(class_names) else 'unknown'

                    objects.append({
                        'class': class_name,
                        'confidence': float(conf),
                        'bbox': [float(x1), float(y1), float(x2), float(y2)],
                        'center': [(x1 + x2) / 2, (y1 + y2) / 2]
                    })

        self.detected_objects = objects
        return objects

    def find_object_by_description(self, description: str):
        """Find an object that matches the description"""
        for obj in self.detected_objects:
            if description.lower() in obj['class'].lower():
                return obj
        return None

class SafetySystem:
    """Safety system for monitoring and enforcing safety constraints"""

    def __init__(self, node):
        self.node = node
        self.emergency_stop = False
        self.safety_violations = []
        self.human_proximity_threshold = 0.8  # meters
        self.collision_threshold = 0.5  # meters
        self.robot_pose = None
        self.latest_laser = None

        # Publisher for emergency stops
        self.emergency_stop_pub = node.create_publisher(Bool, '/emergency_stop', 10)

    def validate_command(self, command: str) -> bool:
        """Validate if a command is safe to execute"""
        command_lower = command.lower()

        # Check for unsafe commands
        unsafe_keywords = ['jump', 'run', 'climb', 'touch face', 'approach quickly']
        for keyword in unsafe_keywords:
            if keyword in command_lower:
                self.safety_violations.append(f"Unsafe command contains: {keyword}")
                return False

        return True

    def is_safe_to_proceed(self) -> bool:
        """Check if it's currently safe to proceed with execution"""
        if self.emergency_stop:
            return False

        # Check proximity to obstacles
        if self.latest_laser:
            min_range = min(self.latest_laser.ranges) if self.latest_laser.ranges else float('inf')
            if min_range < self.collision_threshold:
                self.safety_violations.append(f"Obstacle too close: {min_range:.2f}m")
                # Trigger emergency stop
                self.trigger_emergency_stop()
                return False

        return True

    def update_laser_data(self, laser_msg):
        """Update with new laser scan data"""
        self.latest_laser = laser_msg

    def update_robot_pose(self, pose_msg):
        """Update with new robot pose"""
        self.robot_pose = pose_msg

    def trigger_emergency_stop(self):
        """Trigger emergency stop procedure"""
        self.emergency_stop = True
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
        self.node.get_logger().error("EMERGENCY STOP ACTIVATED")

class ActionExecutor:
    """Execute planned actions using ROS2 action servers"""

    def __init__(self, node):
        self.node = node
        self.nav_client = node.nav_client
        self.manipulation_client = node.manipulation_client

    def execute_action(self, action) -> bool:
        """Execute a single action"""
        action_type = action.get('type', 'unknown')

        if action_type == 'navigation':
            return self.execute_navigation_action(action)
        elif action_type == 'manipulation':
            return self.execute_manipulation_action(action)
        else:
            self.node.get_logger().error(f"Unknown action type: {action_type}")
            return False

    def execute_navigation_action(self, action) -> bool:
        """Execute a navigation action"""
        destination = action.get('destination', 'unknown')

        # In a real system, you would convert destination to coordinates
        # For this example, we'll use a placeholder
        goal_pose = self.get_pose_for_location(destination)

        if goal_pose is None:
            self.node.get_logger().error(f"Unknown destination: {destination}")
            return False

        # Wait for navigation server
        self.nav_client.wait_for_server()

        # Create and send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result (with timeout)
        import rclpy
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)

        if future.result() is None:
            self.node.get_logger().error("Navigation goal timed out")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Navigation goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)

        result = result_future.result().result
        return result.error_code == 0  # Success

    def execute_manipulation_action(self, action) -> bool:
        """Execute a manipulation action"""
        manipulation_action = action.get('action', 'unknown')

        # Wait for manipulation server
        self.manipulation_client.wait_for_server()

        # Create and send manipulation goal
        goal_msg = ManipulateObject.Goal()
        goal_msg.action = manipulation_action
        goal_msg.object_name = action.get('object', 'unknown')

        future = self.manipulation_client.send_goal_async(goal_msg)

        # Wait for result (with timeout)
        import rclpy
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)

        if future.result() is None:
            self.node.get_logger().error("Manipulation goal timed out")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Manipulation goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)

        result = result_future.result().result
        return result.success

    def get_pose_for_location(self, location_name: str):
        """Get pose for a named location (would use location map in real system)"""
        # This would typically load from a location map
        # For demonstration, return a placeholder
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = 1.0  # Placeholder coordinates
        pose.position.y = 1.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0  # No rotation

        return pose

def main(args=None):
    rclpy.init(args=args)

    node = AutonomousHumanoidNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down autonomous humanoid node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()