---
sidebar_position: 4
title: "Integrating VLA Systems with ROS2 Actions & Nav2"
description: "Connecting Vision-Language-Action systems with ROS2 action servers and navigation stack"
---

# Integrating VLA Systems with ROS2 Actions & Nav2

## Overview of ROS2 Actions

ROS2 Actions provide a communication pattern for long-running tasks that require feedback and the ability to cancel. Unlike services, which provide request-response patterns, actions are ideal for tasks like navigation, manipulation, and other processes that take time to complete and may need intermediate feedback.

For humanoid robots implementing Vision-Language-Action (VLA) systems, ROS2 Actions serve as the bridge between high-level cognitive plans and low-level execution. This chapter explores how to effectively integrate VLA systems with ROS2 Actions and the Navigation2 (Nav2) stack.

## Understanding ROS2 Actions

### Action Structure

A ROS2 action consists of three message types:

1. **Goal**: Defines the desired outcome
2. **Feedback**: Provides ongoing status during execution
3. **Result**: Reports the final outcome when complete

```python
# Example action definition (action/FollowPath.action)
# Goal definition
geometry_msgs/PoseStamped[] path
float32 tolerance
---
# Result definition
int32 error_code
string error_message
---
# Feedback definition
int32 current_waypoint
float32 distance_remaining
geometry_msgs/PoseStamped current_pose
```

### Action Client and Server

- **Action Client**: Sends goals to an action server and receives feedback and results
- **Action Server**: Receives goals, executes them, and sends feedback and results back

## VLA Integration with ROS2 Actions

### Action-Based Task Execution

The VLA system decomposes high-level commands into action sequences that can be executed by the robot:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from humanoid_vla_interfaces.action import ExecuteVLACommand

class VLAActionBridge(Node):
    def __init__(self):
        super().__init__('vla_action_bridge')

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.vla_client = ActionClient(self, ExecuteVLACommand, 'execute_vla_command')

        # Service to accept high-level commands
        self.command_service = self.create_service(
            ExecuteVLACommand,
            'vla_command',
            self.handle_command
        )

    def handle_command(self, request, response):
        """Handle high-level VLA command and execute appropriate actions"""
        try:
            # Parse the command and determine required actions
            action_sequence = self.plan_command(request.command)

            # Execute the action sequence
            for action in action_sequence:
                result = self.execute_action(action)
                if not result.success:
                    response.success = False
                    response.message = f"Action failed: {result.message}"
                    return response

            response.success = True
            response.message = "Command completed successfully"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Command execution failed: {str(e)}"
            return response

    def plan_command(self, command: str) -> list:
        """Plan the sequence of actions needed to execute a command"""
        # This would typically call the cognitive planner from Chapter 3
        # For now, we'll implement a simple example

        # Example: "Go to the kitchen and pick up the red cup"
        actions = [
            {
                'type': 'navigation',
                'goal': self.get_location_pose('kitchen'),
                'description': 'Navigate to kitchen'
            },
            {
                'type': 'object_detection',
                'parameters': {'object_name': 'red cup'},
                'description': 'Detect red cup'
            },
            {
                'type': 'manipulation',
                'parameters': {'object_id': 'red_cup_1'},
                'description': 'Grasp red cup'
            }
        ]

        return actions

    def execute_action(self, action: dict):
        """Execute a single action based on its type"""
        if action['type'] == 'navigation':
            return self.execute_navigation_action(action)
        elif action['type'] == 'object_detection':
            return self.execute_detection_action(action)
        elif action['type'] == 'manipulation':
            return self.execute_manipulation_action(action)
        else:
            raise ValueError(f"Unknown action type: {action['type']}")

    def execute_navigation_action(self, action: dict):
        """Execute a navigation action using Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = action['goal']

        # Wait for action server
        self.nav_client.wait_for_server()

        # Send goal and wait for result
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return {'success': False, 'message': 'Navigation goal rejected'}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return {'success': True, 'result': result}
```

## Integration with Navigation2 (Nav2)

### Nav2 Architecture Overview

Navigation2 provides a complete navigation stack for mobile robots with:

- **Global Planner**: Creates optimal paths from start to goal
- **Local Planner**: Executes path following while avoiding obstacles
- **Controller**: Translates planned paths into velocity commands
- **Behavior Trees**: Configurable execution of navigation tasks

### VLA-Enhanced Navigation

For humanoid robots, navigation becomes more complex due to bipedal locomotion requirements. The VLA system enhances navigation by:

1. **Semantic Navigation**: Understanding location names rather than just coordinates
2. **Human-Aware Navigation**: Adjusting paths based on human presence
3. **Context-Aware Planning**: Choosing navigation strategies based on environment

```python
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json

class SemanticNavigationPlanner:
    def __init__(self, node):
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.location_map = self.load_location_map()

    def load_location_map(self):
        """Load semantic location map"""
        # This would typically load from a configuration file
        return {
            'kitchen': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            'dining_room': {'x': 3.0, 'y': 4.0, 'theta': 1.57},
            'living_room': {'x': 5.0, 'y': 1.0, 'theta': 3.14}
        }

    def navigate_to_location(self, location_name: str):
        """Navigate to a semantic location"""
        if location_name not in self.location_map:
            raise ValueError(f"Unknown location: {location_name}")

        location = self.location_map[location_name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = location['x']
        goal_pose.pose.position.y = location['y']

        # Set orientation (simplified)
        from math import cos, sin
        goal_pose.pose.orientation.z = sin(location['theta'] / 2.0)
        goal_pose.pose.orientation.w = cos(location['theta'] / 2.0)

        return self.execute_navigation(goal_pose)

    def execute_navigation(self, goal_pose: PoseStamped):
        """Execute navigation with feedback handling"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.nav_client.wait_for_server()

        # Send goal
        goal_handle_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Wait for result
        rclpy.spin_until_future_complete(self.node, goal_handle_future)
        goal_handle = goal_handle_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error('Navigation goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        return result.error_code == 0  # Success

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        self.node.get_logger().info(
            f'Navigating: {feedback_msg.distance_remaining:.2f}m remaining'
        )
```

## Advanced VLA Integration Patterns

### Multi-Modal Feedback Integration

VLA systems benefit from integrating feedback from multiple sources:

```python
class MultiModalFeedbackHandler:
    def __init__(self, node):
        self.node = node
        self.feedback_buffer = {}

        # Subscribe to various feedback sources
        self.nav_feedback_sub = node.create_subscription(
            NavigateToPose.Feedback,
            'navigate_to_pose/_action/feedback',
            self.nav_feedback_callback,
            10
        )

        self.vision_feedback_sub = node.create_subscription(
            ObjectDetectionFeedback,
            'object_detection/_action/feedback',
            self.vision_feedback_callback,
            10
        )

    def nav_feedback_callback(self, msg):
        """Handle navigation feedback"""
        self.feedback_buffer['navigation'] = {
            'distance_remaining': msg.distance_remaining,
            'current_pose': msg.current_pose,
            'current_waypoint': msg.current_waypoint
        }

        # Update cognitive plan based on navigation progress
        self.update_plan_context('navigation', msg)

    def vision_feedback_callback(self, msg):
        """Handle vision feedback"""
        self.feedback_buffer['vision'] = {
            'objects_detected': msg.objects_detected,
            'confidence_scores': msg.confidence_scores,
            'detection_area': msg.detection_area
        }

        # Update cognitive plan based on vision progress
        self.update_plan_context('vision', msg)

    def update_plan_context(self, source: str, feedback_msg):
        """Update the cognitive plan based on feedback"""
        # This would update the plan in the cognitive planner
        # based on real-time feedback from execution
        pass
```

### Adaptive Planning with Action Feedback

The VLA system can adapt its plans based on action execution feedback:

```python
class AdaptiveVLAPlanner:
    def __init__(self, node):
        self.node = node
        self.original_plan = None
        self.current_step = 0

    def execute_adaptive_plan(self, plan, timeout=30.0):
        """Execute a plan with adaptive capabilities"""
        self.original_plan = plan
        self.current_step = 0

        while self.current_step < len(plan):
            action = plan[self.current_step]

            try:
                # Execute the current action
                result = self.execute_action_with_monitoring(action, timeout)

                if result.success:
                    self.node.get_logger().info(
                        f"Action completed: {action['description']}"
                    )
                    self.current_step += 1
                else:
                    # Handle failure with adaptive strategies
                    adaptive_result = self.handle_action_failure(
                        action, result, plan
                    )
                    if adaptive_result.success:
                        self.current_step += 1
                    else:
                        return adaptive_result

            except Exception as e:
                self.node.get_logger().error(
                    f"Action execution error: {str(e)}"
                )
                # Try adaptive strategies
                adaptive_result = self.handle_action_failure(
                    action, {'success': False, 'error': str(e)}, plan
                )
                if not adaptive_result.success:
                    return adaptive_result

        return {'success': True, 'message': 'Plan completed successfully'}

    def handle_action_failure(self, failed_action, result, original_plan):
        """Handle action failure with adaptive strategies"""
        # Strategy 1: Retry with modified parameters
        if self.should_retry(failed_action, result):
            modified_action = self.modify_action_for_retry(failed_action, result)
            return self.execute_action_with_monitoring(modified_action)

        # Strategy 2: Skip and continue (if optional)
        if self.is_action_optional(failed_action):
            self.node.get_logger().warn(
                f"Skipping optional action: {failed_action['description']}"
            )
            self.current_step += 1
            return {'success': True}

        # Strategy 3: Use alternative approach
        alternative_actions = self.find_alternative_actions(failed_action)
        if alternative_actions:
            # Insert alternative actions into plan
            for alt_action in alternative_actions:
                original_plan.insert(self.current_step + 1, alt_action)
            return self.execute_action_with_monitoring(
                alternative_actions[0]
            )

        # Strategy 4: Request human intervention
        return self.request_human_intervention(failed_action, result)

    def should_retry(self, action, result):
        """Determine if an action should be retried"""
        # Check if failure was due to temporary conditions
        error_msg = result.get('error', '')
        return 'timeout' in error_msg.lower() or 'temporary' in error_msg.lower()

    def modify_action_for_retry(self, action, result):
        """Modify action parameters for retry"""
        modified_action = action.copy()

        # Example: Increase navigation tolerance for retry
        if action['type'] == 'navigation':
            if 'parameters' not in modified_action:
                modified_action['parameters'] = {}
            modified_action['parameters']['tolerance'] = 0.5  # Increase tolerance

        return modified_action
```

## Performance Considerations

### Action Execution Monitoring

For reliable VLA system operation, continuous monitoring is essential:

```python
class VLAExecutionMonitor:
    def __init__(self, node):
        self.node = node
        self.execution_stats = {}

    def start_monitoring_action(self, action_id, action_type):
        """Start monitoring an action execution"""
        self.execution_stats[action_id] = {
            'start_time': self.node.get_clock().now(),
            'type': action_type,
            'status': 'running',
            'feedback_count': 0
        }

    def record_feedback(self, action_id):
        """Record feedback received for an action"""
        if action_id in self.execution_stats:
            self.execution_stats[action_id]['feedback_count'] += 1

    def complete_action(self, action_id, success):
        """Complete monitoring for an action"""
        if action_id in self.execution_stats:
            end_time = self.node.get_clock().now()
            duration = (end_time - self.execution_stats[action_id]['start_time']).nanoseconds / 1e9

            self.execution_stats[action_id].update({
                'status': 'completed' if success else 'failed',
                'duration': duration,
                'success': success
            })

    def get_performance_metrics(self):
        """Get performance metrics for VLA execution"""
        total_actions = len(self.execution_stats)
        successful_actions = sum(1 for stats in self.execution_stats.values()
                                if stats.get('success', False))
        avg_duration = sum(stats.get('duration', 0) for stats in self.execution_stats.values()) / total_actions if total_actions > 0 else 0

        return {
            'total_actions': total_actions,
            'successful_actions': successful_actions,
            'success_rate': successful_actions / total_actions if total_actions > 0 else 0,
            'avg_duration': avg_duration
        }
```

## Summary

Integrating VLA systems with ROS2 Actions and Nav2 creates a robust framework for executing high-level commands on humanoid robots. The action-based approach provides the necessary feedback mechanisms and cancellation capabilities for long-running tasks, while Nav2 provides the navigation infrastructure needed for mobile manipulation.

The key to successful integration lies in:
1. Proper decomposition of high-level commands into action sequences
2. Effective feedback handling and plan adaptation
3. Safety considerations during execution
4. Performance monitoring and optimization

In the next chapter, we'll explore how vision and language systems work together to ground understanding in the physical environment.