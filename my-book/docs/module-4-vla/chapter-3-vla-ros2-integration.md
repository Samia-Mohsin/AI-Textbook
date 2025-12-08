---
sidebar_position: 3
title: "VLA Integration with ROS2 Control Systems"
description: "Implementing Vision-Language-Action systems with ROS2 for humanoid robot control"
learning_objectives:
  - "Integrate VLA models with ROS2 control architecture"
  - "Implement action execution pipelines using ROS2 actions and services"
  - "Design feedback mechanisms for VLA-ROS2 systems"
  - "Handle multi-modal sensor data in ROS2 ecosystem"
---

# VLA Integration with ROS2 Control Systems

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate VLA models with ROS2 control architecture
- Implement action execution pipelines using ROS2 actions and services
- Design feedback mechanisms for VLA-ROS2 systems
- Handle multi-modal sensor data in ROS2 ecosystem

## Introduction

Integrating Vision-Language-Action (VLA) systems with ROS2 creates a powerful framework for intelligent humanoid robot control. This integration enables robots to process natural language commands, perceive their environment visually, and execute complex actions through ROS2's distributed architecture. This chapter explores the implementation of VLA-ROS2 integration, covering communication protocols, action execution frameworks, and feedback mechanisms that ensure reliable robot operation.

The integration requires careful design of interfaces between the VLA model and ROS2 components, handling multi-modal data streams, and ensuring real-time performance for interactive applications. We'll explore how to structure this integration to maintain modularity, scalability, and robustness.

## VLA-ROS2 Architecture

### System Overview

The VLA-ROS2 integration architecture consists of several key components working together:

```
Human User
    ↓ (Natural Language)
VLA Node ← Audio/Text Input
    ↓ (Parsed Action)
ROS2 Action Server ← VLA-Generated Action Plan
    ↓ (Robot Commands)
Hardware Interface Nodes ← Joint Commands, Base Commands, etc.
    ↓ (Sensor Data)
Perception Nodes ← Camera, IMU, LIDAR, etc.
    ↓ (Processed Perception)
VLA Node ← Grounded Perceptions
```

### Key ROS2 Components

The VLA-ROS2 system involves several specialized ROS2 components:

1. **VLA Interface Node**: Coordinates between VLA model and ROS2 ecosystem
2. **Action Execution Server**: Handles complex action sequences
3. **Perception Integration Node**: Connects VLA with sensor processing
4. **Control Bridge Node**: Translates VLA actions to robot controls
5. **Feedback Aggregation Node**: Collects and processes execution feedback

## Implementing the VLA Interface Node

The VLA Interface Node serves as the primary bridge between the VLA model and ROS2:

```python
# vla_interface_node.py - Main VLA-ROS2 interface
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

# Custom action messages (these would be defined in .action files)
from humanoid_vla_interfaces.action import ExecuteVLAAction
from humanoid_vla_interfaces.msg import VLAPerception, VLACommand, VLAActionResult
from humanoid_control_interfaces.action import Navigation, Manipulation

import torch
import numpy as np
from cv_bridge import CvBridge
import json
from typing import Dict, List, Optional, Any
import asyncio
import threading
from dataclasses import dataclass


@dataclass
class VLAExecutionState:
    """State tracking for VLA execution"""
    current_action: str
    progress: float
    status: str
    feedback: Dict[str, Any]
    execution_time: float


class VLAInterfaceNode(Node):
    """
    Main interface node for VLA-ROS2 integration
    """
    def __init__(self):
        super().__init__('vla_interface_node')

        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Publishers for various data streams
        self.perception_publisher = self.create_publisher(
            VLAPerception, '/vla/perception', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/vla/status', 10
        )
        self.debug_publisher = self.create_publisher(
            String, '/vla/debug', 10
        )

        # Subscribers for sensor data
        self.image_subscription = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )
        self.joint_state_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.command_subscription = self.create_subscription(
            VLACommand, '/vla/command', self.command_callback, 10
        )

        # Action clients for different robot capabilities
        self.navigation_client = ActionClient(
            self, Navigation, 'navigate_to_pose'
        )
        self.manipulation_client = ActionClient(
            self, Manipulation, 'manipulation_sequence'
        )

        # Service clients for perception and control
        self.perception_service_client = self.create_client(
            String, '/perception/process'
        )

        # Internal state management
        self.current_perception = None
        self.current_joints = None
        self.vla_execution_state = VLAExecutionState(
            current_action="idle",
            progress=0.0,
            status="ready",
            feedback={},
            execution_time=0.0
        )

        # Configuration parameters
        self.declare_parameter('model_path', '/models/vla_model.pt')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('max_execution_time', 30.0)

        # Load VLA model
        self.vla_model = self._load_vla_model()
        
        # Execution timer
        self.execution_timer = self.create_timer(0.1, self.execution_callback)

        self.get_logger().info('VLA Interface Node initialized')

    def _load_vla_model(self):
        """
        Load the VLA model for inference
        """
        try:
            model_path = self.get_parameter('model_path').value
            # Load model (implementation depends on your VLA model architecture)
            if model_path and model_path.endswith('.pt'):
                model = torch.load(model_path)
                self.get_logger().info(f'Loaded VLA model from {model_path}')
            else:
                # Initialize a default model
                model = self._initialize_default_model()
                self.get_logger().info('Initialized default VLA model')
            
            return model
        except Exception as e:
            self.get_logger().error(f'Error loading VLA model: {e}')
            return self._initialize_default_model()

    def _initialize_default_model(self):
        """
        Initialize a default/simplified VLA model
        """
        # Implement a basic model or return a mock for testing
        class MockVLAModel:
            def __call__(self, *args, **kwargs):
                return {"action": "none", "confidence": 0.0}
        
        return MockVLAModel()

    def image_callback(self, msg: Image):
        """
        Handle incoming camera images
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process image for VLA perception
            perception_data = self._process_image_for_vla(cv_image, msg.header)
            
            # Store for VLA processing
            self.current_perception = perception_data
            
            # Publish processed perception data
            vla_perception_msg = self._create_vla_perception_msg(perception_data, msg.header)
            self.perception_publisher.publish(vla_perception_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def joint_state_callback(self, msg: JointState):
        """
        Handle incoming joint state data
        """
        try:
            # Store current joint states
            self.current_joints = {
                name: pos for name, pos in zip(msg.name, msg.position)
            }
        except Exception as e:
            self.get_logger().error(f'Error processing joint states: {e}')

    def command_callback(self, msg: VLACommand):
        """
        Handle incoming VLA commands
        """
        try:
            # Process VLA command
            self._execute_vla_command(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing VLA command: {e}')

    def _process_image_for_vla(self, image, header):
        """
        Process image data for VLA perception
        """
        # Convert image to format expected by VLA model
        # This is a simplified example - actual implementation depends on your VLA model
        processed_image = self._preprocess_image(image)
        
        # Extract relevant features
        features = {
            'rgb': processed_image,
            'timestamp': header.stamp.sec + header.stamp.nanosec * 1e-9,
            'frame_id': header.frame_id
        }
        
        return features

    def _preprocess_image(self, image):
        """
        Preprocess image for VLA model input
        """
        # Resize and normalize image
        import cv2
        # Example preprocessing - adjust based on your model requirements
        resized = cv2.resize(image, (224, 224))
        normalized = resized.astype(np.float32) / 255.0
        
        return normalized

    def _create_vla_perception_msg(self, perception_data, header):
        """
        Create a VLAPerception message from perception data
        """
        msg = VLAPerception()
        msg.header = header
        
        # Convert perception data to message format
        if 'rgb' in perception_data:
            # Convert back to Image message
            image_msg = self.cv_bridge.cv2_to_imgmsg(
                (perception_data['rgb'] * 255).astype(np.uint8), 
                encoding='rgb8'
            )
            msg.rgb_image = image_msg
        
        msg.timestamp = perception_data.get('timestamp', 0.0)
        msg.frame_id = perception_data.get('frame_id', 'camera')
        
        return msg

    def _execute_vla_command(self, vla_command: VLACommand):
        """
        Execute a VLA command by passing it to the model and handling the result
        """
        # Update execution state
        self.vla_execution_state.current_action = "processing_command"
        self.vla_execution_state.status = "executing"
        
        try:
            # Prepare data for VLA model
            input_data = self._prepare_vla_input(vla_command)
            
            # Run VLA model inference
            model_output = self._run_vla_inference(input_data)
            
            # Process model output and generate robot commands
            action_plan = self._interpret_vla_output(model_output)
            
            if action_plan:
                # Execute the action plan
                self._execute_action_plan(action_plan)
            else:
                self.get_logger().info('No valid action plan generated from VLA output')

        except Exception as e:
            self.get_logger().error(f'Error executing VLA command: {e}')
            self.vla_execution_state.status = f"error: {str(e)}"

        finally:
            # Update execution state
            self.vla_execution_state.status = "completed"

    def _prepare_vla_input(self, vla_command: VLACommand):
        """
        Prepare input data for VLA model
        """
        input_data = {
            'language_instruction': vla_command.text,
            'visual_observation': self.current_perception,
            'joint_states': self.current_joints,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        return input_data

    def _run_vla_inference(self, input_data):
        """
        Run VLA model inference on input data
        """
        # This is where you'd actually call your VLA model
        # For now, using the mock model
        result = self.vla_model(input_data)
        return result

    def _interpret_vla_output(self, vla_output):
        """
        Interpret VLA model output and convert to executable action plan
        """
        # Extract action and confidence from VLA output
        action_type = vla_output.get('action', 'none')
        confidence = vla_output.get('confidence', 0.0)
        
        # Check confidence threshold
        confidence_threshold = self.get_parameter('confidence_threshold').value or 0.7
        if confidence < confidence_threshold:
            self.get_logger().info(f'VLA output confidence too low: {confidence}')
            return None

        # Convert to action plan
        action_plan = {
            'action_type': action_type,
            'parameters': vla_output.get('parameters', {}),
            'confidence': confidence,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        return action_plan

    def _execute_action_plan(self, action_plan):
        """
        Execute an action plan using appropriate ROS2 interfaces
        """
        action_type = action_plan['action_type']
        
        if action_type == 'navigation':
            self._execute_navigation_action(action_plan)
        elif action_type == 'manipulation':
            self._execute_manipulation_action(action_plan)
        elif action_type == 'posture':
            self._execute_posture_action(action_plan)
        elif action_type == 'communication':
            self._execute_communication_action(action_plan)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            self.vla_execution_state.status = "unknown_action_type"

    def _execute_navigation_action(self, action_plan):
        """
        Execute navigation action
        """
        self.get_logger().info('Executing navigation action')
        
        # Wait for navigation action server
        if not self.navigation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Create navigation goal
        goal_msg = Navigation.Goal()
        # Set navigation parameters based on action plan
        # This is a simplified example - implement based on your Navigation interface
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = action_plan['parameters'].get('x', 0.0)
        goal_msg.target_pose.pose.position.y = action_plan['parameters'].get('y', 0.0)
        goal_msg.target_pose.pose.orientation.w = 1.0

        # Send navigation goal
        future = self.navigation_client.send_goal_async(goal_msg)
        future.add_done_callback(self._navigation_done_callback)

    def _navigation_done_callback(self, future):
        """
        Handle completion of navigation action
        """
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Navigation goal accepted')
        else:
            self.get_logger().warn('Navigation goal rejected')

    def _execute_manipulation_action(self, action_plan):
        """
        Execute manipulation action
        """
        self.get_logger().info('Executing manipulation action')
        
        # Wait for manipulation action server
        if not self.manipulation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Manipulation action server not available')
            return

        # Create manipulation goal
        goal_msg = Manipulation.Goal()
        # Set manipulation parameters based on action plan
        goal_msg.task = action_plan['parameters'].get('task', 'move_to_pose')

        # Send manipulation goal
        future = self.manipulation_client.send_goal_async(goal_msg)
        future.add_done_callback(self._manipulation_done_callback)

    def _manipulation_done_callback(self, future):
        """
        Handle completion of manipulation action
        """
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Manipulation goal accepted')
        else:
            self.get_logger().warn('Manipulation goal rejected')

    def _execute_posture_action(self, action_plan):
        """
        Execute posture action
        """
        self.get_logger().info('Executing posture action')
        
        # Publish joint commands directly
        joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set joint positions from action plan
        joint_positions = action_plan['parameters'].get('joint_positions', [])
        joint_msg.name = [f'joint_{i}' for i in range(len(joint_positions))]
        joint_msg.position = joint_positions
        
        joint_pub.publish(joint_msg)

    def _execute_communication_action(self, action_plan):
        """
        Execute communication action
        """
        self.get_logger().info('Executing communication action')
        
        # Publish text to speech command
        speech_pub = self.create_publisher(String, '/text_to_speech', 10)
        speech_msg = String()
        speech_msg.data = action_plan['parameters'].get('text', 'Hello, I am a robot')
        speech_pub.publish(speech_msg)

    def execution_callback(self):
        """
        Regular callback to update execution state and publish status
        """
        # Update execution timer
        self.vla_execution_state.execution_time += 0.1
        
        # Check for timeout
        max_time = self.get_parameter('max_execution_time').value or 30.0
        if self.vla_execution_state.execution_time > max_time:
            self.get_logger().warn('VLA action execution timed out')
            self.vla_execution_state.status = "timeout"
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'current_action': self.vla_execution_state.current_action,
            'progress': self.vla_execution_state.progress,
            'status': self.vla_execution_state.status,
            'execution_time': self.vla_execution_state.execution_time
        })
        self.status_publisher.publish(status_msg)


def main(args=None):
    """
    Main function to run the VLA Interface Node
    """
    rclpy.init(args=args)
    
    node = VLAInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating Custom VLA-ROS2 Interfaces

To properly integrate VLA with ROS2, we need to define custom interfaces:

```python
# action_interface_examples.py - Example action and message definitions
# These would typically be defined in .action files in a ROS2 package

# For action files (.action), the structure would be:
"""
# VLACommand.action
string text
float32[] visual_features
builtin_interfaces/Time timestamp
---
VLAActionResult result
---
VLAActionFeedback feedback
"""

# For message files (.msg), examples include:

# VLAPerception.msg
"""
std_msgs/Header header
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
geometry_msgs/Pose camera_pose
string[] detected_objects
float32[] object_features
builtin_interfaces/Time timestamp
"""

# VLACommand.msg
"""
string text
float32 confidence
builtin_interfaces/Time timestamp
string speaker_id
"""

# VLAActionResult.msg
"""
bool success
string action_type
float32 confidence
string error_message
builtin_interfaces/Time completion_time
"""

# In Python, these would be used as:
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time


class VLACommandInterface:
    """
    Interface definitions for VLA-ROS2 communication
    """
    @staticmethod
    def create_command(text: str, confidence: float = 1.0, speaker_id: str = "") -> VLACommand:
        """Create a VLA command message"""
        msg = VLACommand()
        msg.text = text
        msg.confidence = confidence
        msg.timestamp = Time(sec=0, nanosec=0)  # Will be filled by ROS2
        msg.speaker_id = speaker_id
        return msg

    @staticmethod
    def create_perception(rgb_image: Image, depth_image: Image = None) -> VLAPerception:
        """Create a VLA perception message"""
        msg = VLAPerception()
        msg.header.stamp = Time(sec=0, nanosec=0)  # Will be filled by ROS2
        msg.header.frame_id = "camera"
        msg.rgb_image = rgb_image
        if depth_image:
            msg.depth_image = depth_image
        return msg

    @staticmethod
    def create_result(success: bool, action_type: str, confidence: float = 1.0) -> VLAActionResult:
        """Create a VLA action result message"""
        msg = VLAActionResult()
        msg.success = success
        msg.action_type = action_type
        msg.confidence = confidence
        msg.completion_time = Time(sec=0, nanosec=0)  # Will be filled by ROS2
        msg.error_message = ""
        return msg
```

## Advanced VLA-ROS2 Integration Patterns

### Asynchronous Action Execution

For complex VLA tasks that involve multiple sequential actions, we can implement asynchronous execution:

```python
# async_vla_executor.py - Asynchronous VLA action execution
import asyncio
from typing import List, Dict, Any
from enum import Enum


class ActionState(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class AsyncVLAExecutor:
    """
    Asynchronous executor for VLA action plans
    """
    def __init__(self, vla_interface_node):
        self.vla_interface = vla_interface_node
        self.action_queue = asyncio.Queue()
        self.active_actions = {}
        self.action_callbacks = {}
        
        # Start the execution loop
        self.execution_task = asyncio.create_task(self._execution_loop())

    async def submit_action_plan(self, action_plan: Dict[str, Any], callback=None):
        """
        Submit an action plan for asynchronous execution
        """
        action_id = f"action_{len(self.active_actions)}_{asyncio.get_event_loop().time()}"
        
        # Create action entry
        action_entry = {
            'id': action_id,
            'plan': action_plan,
            'state': ActionState.PENDING,
            'start_time': asyncio.get_event_loop().time(),
            'callback': callback
        }
        
        await self.action_queue.put(action_entry)
        self.active_actions[action_id] = action_entry
        
        return action_id

    async def _execution_loop(self):
        """
        Main execution loop for handling action plans
        """
        while True:
            try:
                # Get next action from queue
                action_entry = await self.action_queue.get()
                
                # Update state
                action_entry['state'] = ActionState.EXECUTING
                
                # Execute the action
                success = await self._execute_single_action(action_entry)
                
                # Update state based on result
                if success:
                    action_entry['state'] = ActionState.COMPLETED
                else:
                    action_entry['state'] = ActionState.FAILED
                
                # Call completion callback if provided
                if action_entry['callback']:
                    await action_entry['callback'](action_entry)
                
                # Mark task as done
                self.action_queue.task_done()
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.vla_interface.get_logger().error(f'Error in execution loop: {e}')

    async def _execute_single_action(self, action_entry: Dict[str, Any]) -> bool:
        """
        Execute a single action from the plan
        """
        try:
            action_plan = action_entry['plan']
            action_type = action_plan.get('action_type')
            
            if action_type == 'composite':
                # Handle composite actions (multiple sub-actions)
                return await self._execute_composite_action(action_plan)
            else:
                # Handle single actions
                return await self._execute_primitive_action(action_plan)
                
        except Exception as e:
            self.vla_interface.get_logger().error(f'Error executing action: {e}')
            return False

    async def _execute_composite_action(self, action_plan: Dict[str, Any]) -> bool:
        """
        Execute a composite action with multiple sub-actions
        """
        sub_actions = action_plan.get('sub_actions', [])
        
        for sub_action in sub_actions:
            success = await self._execute_primitive_action(sub_action)
            if not success:
                self.vla_interface.get_logger().error(
                    f'Sub-action failed: {sub_action.get("action_type")}'
                )
                return False
            # Small delay between sub-actions
            await asyncio.sleep(0.1)
        
        return True

    async def _execute_primitive_action(self, action_plan: Dict[str, Any]) -> bool:
        """
        Execute a primitive (single) action
        """
        action_type = action_plan.get('action_type')
        
        # Execute based on action type
        if action_type == 'navigation':
            return await self._async_execute_navigation(action_plan)
        elif action_type == 'manipulation':
            return await self._async_execute_manipulation(action_plan)
        elif action_type == 'perception':
            return await self._async_execute_perception(action_plan)
        else:
            self.vla_interface.get_logger().warn(f'Unknown action type: {action_type}')
            return False

    async def _async_execute_navigation(self, action_plan: Dict[str, Any]) -> bool:
        """
        Asynchronously execute navigation action
        """
        # In a real implementation, this would interact with navigation servers
        # For now, using a mock implementation
        await asyncio.sleep(2.0)  # Simulate navigation time
        return True

    async def _async_execute_manipulation(self, action_plan: Dict[str, Any]) -> bool:
        """
        Asynchronously execute manipulation action
        """
        # In a real implementation, this would interact with manipulation servers
        # For now, using a mock implementation
        await asyncio.sleep(1.5)  # Simulate manipulation time
        return True

    async def _async_execute_perception(self, action_plan: Dict[str, Any]) -> bool:
        """
        Asynchronously execute perception action
        """
        # In a real implementation, this would trigger perception processing
        # For now, using a mock implementation
        await asyncio.sleep(0.5)  # Simulate perception time
        return True

    def get_action_status(self, action_id: str) -> ActionState:
        """
        Get the current status of an action
        """
        if action_id in self.active_actions:
            return self.active_actions[action_id]['state']
        else:
            return ActionState.CANCELLED  # or raise exception

    def cancel_action(self, action_id: str) -> bool:
        """
        Cancel a pending or executing action
        """
        if action_id in self.active_actions:
            action_entry = self.active_actions[action_id]
            if action_entry['state'] in [ActionState.PENDING, ActionState.EXECUTING]:
                action_entry['state'] = ActionState.CANCELLED
                return True
        return False

    async def shutdown(self):
        """
        Shutdown the executor
        """
        self.execution_task.cancel()
        try:
            await self.execution_task
        except asyncio.CancelledError:
            pass
```

## Testing and Validation

### Unit Testing VLA-ROS2 Integration

Create comprehensive tests for the VLA-ROS2 integration:

```python
# test_vla_integration.py - Tests for VLA-ROS2 integration
import unittest
from unittest.mock import Mock, AsyncMock, patch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio

from vla_interface_node import VLAInterfaceNode
from async_vla_executor import AsyncVLAExecutor, ActionState


class TestVLAIntegration(unittest.TestCase):
    """
    Tests for VLA-ROS2 integration
    """
    def setUp(self):
        """
        Set up test environment
        """
        rclpy.init()
        self.node = VLAInterfaceNode()
        self.executor = AsyncVLAExecutor(self.node)

    def tearDown(self):
        """
        Clean up after tests
        """
        rclpy.shutdown()

    def test_vla_model_loading(self):
        """
        Test that VLA model loads correctly
        """
        self.assertIsNotNone(self.node.vla_model)
        self.assertIsInstance(self.node.vla_model, object)  # Mock model or actual model

    def test_image_callback(self):
        """
        Test image callback functionality
        """
        # Create mock image message
        mock_image_msg = Mock()
        mock_image_msg.header = Mock()
        mock_image_msg.header.stamp = Mock()
        mock_image_msg.header.stamp.sec = 0
        mock_image_msg.header.stamp.nanosec = 0
        mock_image_msg.header.frame_id = "camera"
        
        # Call the callback
        try:
            self.node.image_callback(mock_image_msg)
            # Verify perception was updated
            self.assertIsNotNone(self.node.current_perception)
        except Exception as e:
            self.fail(f"Image callback failed: {e}")

    def test_command_callback(self):
        """
        Test command callback functionality
        """
        # Create mock command
        mock_command = Mock()
        mock_command.text = "Move forward"
        mock_command.confidence = 0.9
        
        # Call the callback
        try:
            self.node.command_callback(mock_command)
            # Verify execution state was updated
            self.assertEqual(self.node.vla_execution_state.status, "executing")
        except Exception as e:
            self.fail(f"Command callback failed: {e}")

    def test_action_execution(self):
        """
        Test action execution through the async executor
        """
        action_plan = {
            'action_type': 'navigation',
            'parameters': {'x': 1.0, 'y': 1.0},
            'confidence': 0.9
        }
        
        # Submit action to executor
        async def run_test():
            action_id = await self.executor.submit_action_plan(action_plan)
            self.assertIsNotNone(action_id)
            
            # Wait a bit for execution
            await asyncio.sleep(0.5)
            
            # Check action status
            status = self.executor.get_action_status(action_id)
            # Action might be completed or still executing
            self.assertIn(status, [ActionState.EXECUTING, ActionState.COMPLETED])
        
        # Run the async test
        asyncio.run(run_test())


def test_suite():
    """
    Create test suite for VLA integration
    """
    suite = unittest.TestSuite()
    suite.addTest(TestVLAIntegration('test_vla_model_loading'))
    suite.addTest(TestVLAIntegration('test_image_callback'))
    suite.addTest(TestVLAIntegration('test_command_callback'))
    suite.addTest(TestVLAIntegration('test_action_execution'))
    return suite


if __name__ == '__main__':
    runner = unittest.TextTestRunner(verbosity=2)
    runner.run(test_suite())
```

## Performance Considerations

### Real-Time Performance Optimization

For real-time VLA-ROS2 integration, performance optimization is crucial:

```python
# performance_optimization.py - Performance optimizations for VLA-ROS2
import time
import threading
from collections import deque
import numpy as np
import queue


class VLAPerformanceOptimizer:
    """
    Performance optimizer for VLA-ROS2 integration
    """
    def __init__(self, node):
        self.node = node
        
        # Performance metrics tracking
        self.inference_times = deque(maxlen=100)
        self.pipeline_times = deque(maxlen=100)
        self.frame_rates = deque(maxlen=100)
        
        # Threading for non-blocking operations
        self.processing_queue = queue.Queue(maxsize=10)
        self.result_queue = queue.Queue(maxsize=10)
        
        # Model optimization
        self.optimized_model = None
        self.calibration_params = {}

    def optimize_inference(self):
        """
        Optimize VLA model inference performance
        """
        # In a real implementation, this might involve:
        # - Model quantization
        # - TensorRT optimization
        # - ONNX conversion
        # - GPU optimization
        pass

    def adaptive_sampling(self, sensor_data_rate: float) -> float:
        """
        Adaptively adjust sensor sampling rate based on system load
        """
        avg_inference_time = np.mean(self.inference_times) if self.inference_times else 0.1
        
        # Adjust sampling rate based on inference time
        target_inference_rate = 1.0 / avg_inference_time if avg_inference_time > 0 else 10.0
        
        # Limit to reasonable bounds
        adjusted_rate = max(1.0, min(target_inference_rate, sensor_data_rate))
        
        return adjusted_rate

    def pipeline_monitoring(self):
        """
        Monitor pipeline performance and adjust parameters
        """
        current_time = time.time()
        
        # Log performance metrics periodically
        if current_time % 5.0 < 0.1:  # Log every 5 seconds
            avg_inference = np.mean(self.inference_times) if self.inference_times else 0
            avg_pipeline = np.mean(self.pipeline_times) if self.pipeline_times else 0
            avg_fps = np.mean(self.frame_rates) if self.frame_rates else 0
            
            self.node.get_logger().info(
                f'Performance - Inference: {avg_inference:.3f}s, '
                f'Pipeline: {avg_pipeline:.3f}s, FPS: {avg_fps:.1f}'
            )

    def prioritize_commands(self, command_queue: deque) -> deque:
        """
        Prioritize commands based on type and urgency
        """
        # Separate commands by priority
        high_priority = []
        medium_priority = []
        low_priority = []
        
        for cmd in command_queue:
            priority = cmd.get('priority', 5)  # Default to medium
            if priority >= 8:
                high_priority.append(cmd)
            elif priority >= 5:
                medium_priority.append(cmd)
            else:
                low_priority.append(cmd)
        
        # Reorder queue: high, medium, low
        reordered = high_priority + medium_priority + low_priority
        return deque(reordered)


class ThreadedVLAProcessor:
    """
    Threaded processor for non-blocking VLA operations
    """
    def __init__(self, vla_model):
        self.vla_model = vla_model
        self.input_queue = queue.Queue(maxsize=5)
        self.output_queue = queue.Queue(maxsize=5)
        
        # Processing thread
        self.processing_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.processing_thread.start()
        
        # Stop event
        self.stop_event = threading.Event()

    def submit_input(self, input_data):
        """
        Submit input data for processing (non-blocking)
        """
        try:
            self.input_queue.put_nowait(input_data)
            return True
        except queue.Full:
            return False  # Queue full, input rejected

    def get_output(self, timeout=None):
        """
        Get processed output (blocking with optional timeout)
        """
        try:
            return self.output_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _process_loop(self):
        """
        Main processing loop running in separate thread
        """
        while not self.stop_event.is_set():
            try:
                # Get input with timeout
                input_data = self.input_queue.get(timeout=0.1)
                
                # Process with VLA model
                start_time = time.time()
                output = self._run_vla_model(input_data)
                processing_time = time.time() - start_time
                
                # Put result in output queue
                try:
                    self.output_queue.put_nowait(output)
                except queue.Full:
                    # Output queue full, drop result
                    pass
                    
            except queue.Empty:
                # No input available, continue loop
                continue

    def _run_vla_model(self, input_data):
        """
        Run VLA model on input data
        """
        # This would call your actual VLA model
        result = self.vla_model(input_data)
        return result

    def shutdown(self):
        """
        Shutdown the processor
        """
        self.stop_event.set()
        self.processing_thread.join(timeout=1.0)
```

## Summary

This chapter has covered the essential aspects of integrating Vision-Language-Action (VLA) systems with ROS2 for humanoid robot control. We've explored:

1. The architecture of VLA-ROS2 integration systems
2. Implementation of the core VLA interface node
3. Definition of custom ROS2 interfaces for VLA communication
4. Asynchronous execution patterns for complex action plans
5. Performance optimization techniques for real-time operation
6. Testing and validation approaches

The integration of VLA models with ROS2 enables sophisticated human-robot interaction where natural language commands are understood in the context of visual perception and executed as appropriate robot actions. This combination opens up possibilities for more intuitive and flexible robot control in human environments.

In the next chapter, we'll explore advanced VLA applications and deployment considerations for real-world humanoid robotics.