---
title: Chapter 6 - Safety & Fallback Behaviors for Humanoid Robots
sidebar_position: 6
---

# Safety & Fallback Behaviors for Humanoid Robots

## Introduction

Safety and fallback behaviors are critical components of humanoid robot systems, particularly when operating in human environments. Unlike industrial robots operating in controlled spaces, humanoid robots must navigate complex, dynamic environments with humans present, requiring robust safety mechanisms and graceful degradation strategies. This chapter explores comprehensive safety frameworks, risk assessment methodologies, and fallback behaviors that ensure humanoid robots can operate reliably while maintaining human safety.

## Safety Framework for Humanoid Robots

### Safety-by-Design Principles

Humanoid robot safety should be implemented at multiple levels:

1. **Hardware Safety**: Intrinsic safety mechanisms in actuators and sensors
2. **Control Safety**: Real-time safety monitoring and intervention
3. **Behavioral Safety**: Safe decision-making and action execution
4. **System Safety**: Fail-safe mechanisms and error recovery

### Safety Standards and Regulations

Humanoid robots must comply with relevant safety standards:

- **ISO 13482**: Safety requirements for personal care robots
- **ISO 12100**: Safety of machinery principles
- **ISO 10218**: Safety requirements for industrial robots
- **IEC 62566**: Functional safety for robot control systems

```python
# safety_framework.py
import enum
from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass
import time
import logging
from threading import Lock

class SafetyLevel(enum.IntEnum):
    """Safety level hierarchy"""
    SAFE = 0          # Robot is in safe state
    CAUTION = 1       # Potential risk detected
    WARNING = 2       # Risk requires attention
    DANGER = 3        # Immediate danger - stop all motion
    EMERGENCY = 4     # Emergency stop required

class SafetyCondition(enum.Enum):
    """Specific safety conditions"""
    HUMAN_PROXIMITY = "human_proximity"
    COLLISION_IMMINENT = "collision_imminent"
    JOINT_LIMIT_EXCEEDED = "joint_limit_exceeded"
    VELOCITY_LIMIT_EXCEEDED = "velocity_limit_exceeded"
    TORQUE_LIMIT_EXCEEDED = "torque_limit_exceeded"
    FALL_DETECTED = "fall_detected"
    STABILITY_LOSS = "stability_loss"
    COMMUNICATION_FAILURE = "communication_failure"
    POWER_FAILURE = "power_failure"

@dataclass
class SafetyEvent:
    """Represents a safety-related event"""
    condition: SafetyCondition
    level: SafetyLevel
    timestamp: float
    source: str
    description: str
    parameters: Dict[str, Any]

class SafetyMonitor:
    """Monitors safety conditions and triggers appropriate responses"""

    def __init__(self):
        self.safety_level = SafetyLevel.SAFE
        self.event_log = []
        self.safety_callbacks = {}  # condition -> callback function
        self.lock = Lock()
        self.logger = logging.getLogger('safety_monitor')

        # Thresholds for different safety conditions
        self.thresholds = {
            'proximity_distance': 0.5,  # meters
            'collision_time_threshold': 1.0,  # seconds
            'joint_position_threshold': 0.1,  # radians
            'velocity_threshold': 2.0,  # rad/s
            'torque_threshold': 50.0,  # Nm
            'stability_margin': 0.1    # m (stability margin)
        }

    def register_callback(self, condition: SafetyCondition, callback: Callable):
        """Register a callback function for a specific safety condition"""
        self.safety_callbacks[condition] = callback

    def evaluate_proximity_safety(self, human_positions: List[tuple]) -> Optional[SafetyEvent]:
        """Evaluate safety based on human proximity"""
        for human_pos in human_positions:
            distance = self._calculate_distance_to_human(human_pos)
            if distance < self.thresholds['proximity_distance']:
                level = SafetyLevel.WARNING if distance > 0.2 else SafetyLevel.DANGER
                return SafetyEvent(
                    condition=SafetyCondition.HUMAN_PROXIMITY,
                    level=level,
                    timestamp=time.time(),
                    source="proximity_sensor",
                    description=f"Human detected at {distance:.2f}m",
                    parameters={"distance": distance, "position": human_pos}
                )
        return None

    def evaluate_collision_risk(self, trajectory: List[tuple]) -> Optional[SafetyEvent]:
        """Evaluate collision risk along a trajectory"""
        for i, (pos, time) in enumerate(trajectory):
            collision_time = self._estimate_collision_time(pos)
            if collision_time < self.thresholds['collision_time_threshold']:
                return SafetyEvent(
                    condition=SafetyCondition.COLLISION_IMMINENT,
                    level=SafetyLevel.DANGER,
                    timestamp=time.time(),
                    source="collision_predictor",
                    description=f"Collision imminent in {collision_time:.2f}s",
                    parameters={"collision_time": collision_time, "position": pos}
                )
        return None

    def evaluate_joint_safety(self, joint_states: Dict[str, float]) -> List[SafetyEvent]:
        """Evaluate joint safety conditions"""
        events = []

        for joint_name, position in joint_states.items():
            # Check joint limits
            if abs(position) > self.thresholds['joint_position_threshold']:
                events.append(SafetyEvent(
                    condition=SafetyCondition.JOINT_LIMIT_EXCEEDED,
                    level=SafetyLevel.WARNING,
                    timestamp=time.time(),
                    source=f"joint_{joint_name}",
                    description=f"Joint {joint_name} exceeds limit: {position:.2f}",
                    parameters={"joint": joint_name, "position": position}
                ))

        return events

    def evaluate_stability(self, com_position: tuple, support_polygon: List[tuple]) -> Optional[SafetyEvent]:
        """Evaluate robot stability"""
        stability_margin = self._calculate_stability_margin(com_position, support_polygon)
        if stability_margin < self.thresholds['stability_margin']:
            return SafetyEvent(
                condition=SafetyCondition.STABILITY_LOSS,
                level=SafetyLevel.DANGER,
                timestamp=time.time(),
                source="stability_monitor",
                description=f"Stability margin too low: {stability_margin:.3f}m",
                parameters={
                    "com_position": com_position,
                    "stability_margin": stability_margin,
                    "support_polygon": support_polygon
                }
            )
        return None

    def _calculate_distance_to_human(self, human_pos: tuple) -> float:
        """Calculate distance from robot to human (simplified)"""
        # This would interface with actual distance sensors
        robot_pos = (0.0, 0.0, 0.0)  # Robot position
        dx = robot_pos[0] - human_pos[0]
        dy = robot_pos[1] - human_pos[1]
        dz = robot_pos[2] - human_pos[2]
        return (dx**2 + dy**2 + dz**2)**0.5

    def _estimate_collision_time(self, position: tuple) -> float:
        """Estimate time to collision (simplified)"""
        # This would use more sophisticated collision prediction
        return 2.0  # Placeholder

    def _calculate_stability_margin(self, com_position: tuple, support_polygon: List[tuple]) -> float:
        """Calculate stability margin relative to support polygon"""
        # Calculate distance from center of mass to edge of support polygon
        # This is a simplified implementation
        return 0.15  # Placeholder value

    def process_safety_events(self, events: List[SafetyEvent]) -> SafetyLevel:
        """Process safety events and update safety level"""
        with self.lock:
            # Update safety level based on highest priority event
            for event in events:
                self.event_log.append(event)

                # Trigger callback if registered
                if event.condition in self.safety_callbacks:
                    self.safety_callbacks[event.condition](event)

                # Update safety level if new event is more critical
                if event.level > self.safety_level:
                    self.safety_level = event.level

            return self.safety_level

    def get_current_safety_level(self) -> SafetyLevel:
        """Get current safety level"""
        return self.safety_level

    def request_safety_action(self, level: SafetyLevel) -> bool:
        """Request a safety action at specified level"""
        if level <= self.safety_level:
            return True  # Already at appropriate safety level

        # Execute safety action based on requested level
        if level == SafetyLevel.DANGER:
            self._execute_safe_stop()
        elif level == SafetyLevel.EMERGENCY:
            self._execute_emergency_stop()

        self.safety_level = level
        return True

    def _execute_safe_stop(self):
        """Execute safe stop procedure"""
        self.logger.info("Executing safe stop")
        # Command robot to stop all motion gradually
        # Move to safe pose if possible
        pass

    def _execute_emergency_stop(self):
        """Execute emergency stop procedure"""
        self.logger.critical("Executing emergency stop")
        # Immediate stop, possibly cut power to actuators
        # Preserve critical systems for safety
        pass
```

## Fallback Behavior Architecture

### Hierarchical Fallback System

```python
# fallback_system.py
from enum import Enum
from typing import Dict, List, Any, Optional
import time

class FallbackStrategy(Enum):
    """Types of fallback strategies"""
    CONTINUE_WITH_MODIFIED_PLAN = "continue_modified"
    REVERT_TO_SAFE_POSE = "revert_safe_pose"
    REQUEST_HUMAN_ASSISTANCE = "request_assistance"
    RETURN_TO_HOME = "return_home"
    SAFE_SHUTDOWN = "safe_shutdown"
    SWITCH_TO_SIMPLIFIED_BEHAVIOR = "simplified_behavior"

class FallbackBehavior:
    """Manages fallback behaviors for different failure scenarios"""

    def __init__(self):
        self.fallback_rules = {
            'navigation_failure': FallbackStrategy.RETURN_TO_HOME,
            'object_detection_failure': FallbackStrategy.SWITCH_TO_SIMPLIFIED_BEHAVIOR,
            'grasp_failure': FallbackStrategy.REQUEST_HUMAN_ASSISTANCE,
            'communication_loss': FallbackStrategy.RETURN_TO_HOME,
            'power_low': FallbackStrategy.SAFE_SHUTDOWN,
            'stability_loss': FallbackStrategy.REVERT_TO_SAFE_POSE,
            'sensor_failure': FallbackStrategy.SWITCH_TO_SIMPLIFIED_BEHAVIOR
        }

        self.fallback_execution_history = []
        self.active_fallback = None

    def determine_fallback(self, failure_type: str) -> Optional[FallbackStrategy]:
        """Determine appropriate fallback for failure type"""
        return self.fallback_rules.get(failure_type)

    def execute_fallback(self, strategy: FallbackStrategy, context: Dict[str, Any]) -> bool:
        """Execute a fallback strategy"""
        start_time = time.time()
        success = False

        try:
            self.active_fallback = strategy

            if strategy == FallbackStrategy.CONTINUE_WITH_MODIFIED_PLAN:
                success = self._continue_with_modified_plan(context)
            elif strategy == FallbackStrategy.REVERT_TO_SAFE_POSE:
                success = self._revert_to_safe_pose(context)
            elif strategy == FallbackStrategy.REQUEST_HUMAN_ASSISTANCE:
                success = self._request_human_assistance(context)
            elif strategy == FallbackStrategy.RETURN_TO_HOME:
                success = self._return_to_home(context)
            elif strategy == FallbackStrategy.SAFE_SHUTDOWN:
                success = self._safe_shutdown(context)
            elif strategy == FallbackStrategy.SWITCH_TO_SIMPLIFIED_BEHAVIOR:
                success = self._switch_to_simplified_behavior(context)

            execution_time = time.time() - start_time

            self.fallback_execution_history.append({
                'strategy': strategy,
                'context': context,
                'success': success,
                'execution_time': execution_time,
                'timestamp': start_time
            })

        except Exception as e:
            self.active_fallback = None
            return False

        self.active_fallback = None
        return success

    def _continue_with_modified_plan(self, context: Dict[str, Any]) -> bool:
        """Continue execution with a modified plan"""
        # Implementation would modify current plan to work around failure
        # For example, if navigation fails, try alternative route
        print("Continuing with modified plan")
        return True

    def _revert_to_safe_pose(self, context: Dict[str, Any]) -> bool:
        """Move robot to a predefined safe pose"""
        print("Reverting to safe pose")
        # Move joints to safe positions that ensure stability
        # This would interface with robot's motion control system
        return True

    def _request_human_assistance(self, context: Dict[str, Any]) -> bool:
        """Request human assistance through UI or communication system"""
        print("Requesting human assistance")
        # This might trigger an alert, send notification, or activate help interface
        return True

    def _return_to_home(self, context: Dict[str, Any]) -> bool:
        """Navigate back to home/base position"""
        print("Returning to home position")
        # Navigate to a safe, predefined location
        return True

    def _safe_shutdown(self, context: Dict[str, Any]) -> bool:
        """Execute safe shutdown procedure"""
        print("Executing safe shutdown")
        # Save current state, stop all motion, power down safely
        return True

    def _switch_to_simplified_behavior(self, context: Dict[str, Any]) -> bool:
        """Switch to simplified behavior mode using fewer capabilities"""
        print("Switching to simplified behavior mode")
        # Disable complex behaviors, use only basic functions
        return True

class FallbackManager:
    """Main manager for fallback behaviors"""

    def __init__(self):
        self.fallback_system = FallbackBehavior()
        self.safety_monitor = SafetyMonitor()
        self.active_failures = set()
        self.fallback_queue = []

    def register_failure(self, failure_type: str, details: Dict[str, Any]):
        """Register a failure and trigger appropriate fallback"""
        self.active_failures.add(failure_type)

        strategy = self.fallback_system.determine_fallback(failure_type)
        if strategy:
            self.fallback_queue.append((strategy, details))
            self.execute_next_fallback()

    def execute_next_fallback(self):
        """Execute the next fallback in the queue"""
        if self.fallback_queue:
            strategy, context = self.fallback_queue.pop(0)
            success = self.fallback_system.execute_fallback(strategy, context)

            if not success:
                # If fallback failed, try more conservative fallback
                self._escalate_fallback()

    def _escalate_fallback(self):
        """Escalate to more conservative fallback strategy"""
        # Move to more conservative safety strategy
        self.safety_monitor.request_safety_action(SafetyLevel.DANGER)
```

## Risk Assessment and Management

### Dynamic Risk Assessment

```python
# risk_assessment.py
import numpy as np
from typing import Dict, List, Tuple
from enum import Enum
import math

class RiskLevel(Enum):
    """Risk assessment levels"""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

class RiskAssessment:
    """Dynamic risk assessment system"""

    def __init__(self):
        self.risk_factors = {
            'human_proximity': 0.3,
            'environment_complexity': 0.2,
            'robot_velocity': 0.2,
            'task_complexity': 0.15,
            'system_uncertainty': 0.15
        }

        self.risk_history = []

    def assess_environmental_risk(self,
                                 human_positions: List[Tuple[float, float, float]],
                                 robot_position: Tuple[float, float, float],
                                 robot_velocity: float) -> RiskLevel:
        """Assess environmental risk based on human proximity and robot motion"""

        # Calculate minimum distance to humans
        min_distance = float('inf')
        for human_pos in human_positions:
            dist = self._calculate_3d_distance(robot_position, human_pos)
            min_distance = min(min_distance, dist)

        # Calculate risk based on distance and velocity
        proximity_risk = max(0, (1 - min_distance / 2.0))  # Higher risk when close
        velocity_risk = min(1.0, robot_velocity / 1.0)  # Risk increases with velocity

        # Combine risk factors
        total_risk = (
            proximity_risk * self.risk_factors['human_proximity'] +
            velocity_risk * self.risk_factors['robot_velocity']
        )

        return self._map_to_risk_level(total_risk)

    def assess_task_risk(self,
                        task_complexity: float,
                        system_confidence: float,
                        environmental_conditions: Dict[str, float]) -> RiskLevel:
        """Assess risk of executing a specific task"""

        # Task complexity increases risk
        complexity_factor = min(1.0, task_complexity)

        # Low system confidence increases risk
        uncertainty_factor = 1.0 - system_confidence

        # Environmental factors (lighting, noise, etc.)
        env_factor = sum(environmental_conditions.values()) / len(environmental_conditions) if environmental_conditions else 0.0

        # Combined risk
        task_risk = (
            complexity_factor * self.risk_factors['task_complexity'] +
            uncertainty_factor * self.risk_factors['system_uncertainty'] +
            env_factor * self.risk_factors['environment_complexity']
        )

        return self._map_to_risk_level(task_risk)

    def _calculate_3d_distance(self, pos1: Tuple[float, float, float],
                             pos2: Tuple[float, float, float]) -> float:
        """Calculate 3D Euclidean distance"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _map_to_risk_level(self, risk_score: float) -> RiskLevel:
        """Map risk score to risk level"""
        if risk_score < 0.3:
            return RiskLevel.LOW
        elif risk_score < 0.6:
            return RiskLevel.MEDIUM
        elif risk_score < 0.8:
            return RiskLevel.HIGH
        else:
            return RiskLevel.CRITICAL

    def update_safety_constraints(self, risk_level: RiskLevel) -> Dict[str, float]:
        """Update safety constraints based on risk level"""
        constraints = {
            'max_velocity': 1.0,  # Default max velocity
            'min_distance': 0.5,  # Default minimum distance
            'reaction_time': 0.1,  # Default reaction time
            'max_torque': 50.0     # Default max torque
        }

        if risk_level == RiskLevel.LOW:
            # Minimal restrictions
            pass
        elif risk_level == RiskLevel.MEDIUM:
            # Moderate restrictions
            constraints['max_velocity'] *= 0.7
            constraints['min_distance'] *= 1.2
        elif risk_level == RiskLevel.HIGH:
            # Significant restrictions
            constraints['max_velocity'] *= 0.4
            constraints['min_distance'] *= 1.5
            constraints['reaction_time'] *= 0.7
        elif risk_level == RiskLevel.CRITICAL:
            # Maximum restrictions - robot should stop
            constraints['max_velocity'] *= 0.1
            constraints['min_distance'] *= 2.0
            constraints['max_torque'] *= 0.5

        return constraints

class RiskBasedController:
    """Controller that adapts behavior based on risk assessment"""

    def __init__(self):
        self.risk_assessor = RiskAssessment()
        self.current_risk_level = RiskLevel.LOW
        self.safety_constraints = {}

    def update_risk_and_constraints(self,
                                  human_positions: List[Tuple[float, float, float]],
                                  robot_state: Dict[str, float],
                                  task_info: Dict[str, float]):
        """Update risk assessment and safety constraints"""

        # Assess environmental risk
        env_risk = self.risk_assessor.assess_environmental_risk(
            human_positions,
            (robot_state.get('x', 0), robot_state.get('y', 0), robot_state.get('z', 0)),
            robot_state.get('velocity', 0)
        )

        # Assess task risk
        task_risk = self.risk_assessor.assess_task_risk(
            task_info.get('complexity', 0.5),
            task_info.get('confidence', 0.8),
            task_info.get('environmental_factors', {})
        )

        # Use the higher risk level
        self.current_risk_level = max([env_risk, task_risk], key=lambda x: ['low', 'medium', 'high', 'critical'].index(x.value))

        # Update safety constraints
        self.safety_constraints = self.risk_assessor.update_safety_constraints(self.current_risk_level)

        return self.current_risk_level, self.safety_constraints
```

## Emergency Procedures

### Emergency Stop System

```python
# emergency_system.py
import signal
import sys
import threading
from typing import List, Callable, Any
import time

class EmergencyStopSystem:
    """Comprehensive emergency stop system"""

    def __init__(self):
        self.emergency_active = False
        self.emergency_callbacks = []  # Functions to call during emergency
        self.emergency_lock = threading.Lock()
        self.emergency_thread = None

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def register_emergency_callback(self, callback: Callable[[], Any]):
        """Register a function to be called during emergency stop"""
        self.emergency_callbacks.append(callback)

    def trigger_emergency_stop(self):
        """Trigger emergency stop sequence"""
        with self.emergency_lock:
            if self.emergency_active:
                return  # Already in emergency state

            self.emergency_active = True
            print("EMERGENCY STOP TRIGGERED")

            # Execute all registered emergency callbacks
            for callback in self.emergency_callbacks:
                try:
                    callback()
                except Exception as e:
                    print(f"Emergency callback error: {e}")

            # Additional emergency procedures
            self._execute_emergency_procedures()

    def _signal_handler(self, signum, frame):
        """Handle system signals for emergency stop"""
        print(f"Received signal {signum}, initiating emergency stop")
        self.trigger_emergency_stop()
        sys.exit(0)

    def _execute_emergency_procedures(self):
        """Execute specific emergency procedures"""
        # 1. Cut power to non-essential systems
        self._power_down_non_essential_systems()

        # 2. Preserve critical safety systems
        self._preserve_safety_systems()

        # 3. Log emergency event
        self._log_emergency_event()

    def _power_down_non_essential_systems(self):
        """Power down non-essential systems"""
        print("Powering down non-essential systems...")
        # Implementation would cut power to non-critical components
        pass

    def _preserve_safety_systems(self):
        """Preserve critical safety systems"""
        print("Preserving safety systems...")
        # Ensure safety systems remain active
        pass

    def _log_emergency_event(self):
        """Log emergency event for analysis"""
        print("Logging emergency event...")
        # Implementation would log event to persistent storage
        pass

    def is_emergency_active(self) -> bool:
        """Check if emergency stop is active"""
        return self.emergency_active

    def reset_emergency(self):
        """Reset emergency state (requires manual confirmation)"""
        with self.emergency_lock:
            if self.emergency_active:
                # In real implementation, this would require manual confirmation
                # from a human operator
                self.emergency_active = False
                print("Emergency reset - system operational")
                return True
        return False

class SafetySupervisor:
    """Supervises all safety systems and coordinates responses"""

    def __init__(self):
        self.safety_monitor = SafetyMonitor()
        self.fallback_manager = FallbackManager()
        self.risk_controller = RiskBasedController()
        self.emergency_system = EmergencyStopSystem()

        self.system_operational = True
        self.safety_thread = None
        self.running = False

    def start_safety_monitoring(self):
        """Start continuous safety monitoring"""
        self.running = True
        self.safety_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.safety_thread.start()

    def stop_safety_monitoring(self):
        """Stop safety monitoring"""
        self.running = False
        if self.safety_thread:
            self.safety_thread.join()

    def _monitor_loop(self):
        """Main safety monitoring loop"""
        while self.running:
            try:
                # Check for safety events
                events = self._collect_safety_events()

                if events:
                    current_level = self.safety_monitor.process_safety_events(events)

                    # If danger level reached, trigger emergency if critical
                    if current_level >= SafetyLevel.DANGER:
                        self.emergency_system.trigger_emergency_stop()
                    elif current_level >= SafetyLevel.WARNING:
                        # Execute appropriate fallback
                        self._handle_warning_level(current_level)

                # Update risk-based constraints
                self._update_risk_constraints()

                # Sleep briefly to avoid busy waiting
                time.sleep(0.1)

            except Exception as e:
                print(f"Error in safety monitoring: {e}")
                time.sleep(0.1)  # Brief delay before continuing

    def _collect_safety_events(self) -> List[SafetyEvent]:
        """Collect safety events from various sources"""
        events = []

        # This would interface with real sensor data
        # For simulation, we'll generate some events
        if self._should_generate_test_event():
            events.append(SafetyEvent(
                condition=SafetyCondition.HUMAN_PROXIMITY,
                level=SafetyLevel.WARNING,
                timestamp=time.time(),
                source="test_sensor",
                description="Test proximity event",
                parameters={"distance": 0.3}
            ))

        return events

    def _should_generate_test_event(self) -> bool:
        """Generate test events for demonstration"""
        import random
        return random.random() < 0.01  # 1% chance per cycle

    def _handle_warning_level(self, level: SafetyLevel):
        """Handle warning level safety events"""
        print(f"Handling safety level: {level}")
        # Execute appropriate response based on warning level
        pass

    def _update_risk_constraints(self):
        """Update risk-based safety constraints"""
        # This would update based on real environmental data
        pass
```

## Implementation Examples

### ROS2 Safety Node

```python
# safety_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Time

from safety_framework import SafetyMonitor, SafetyLevel, SafetyCondition
from fallback_system import FallbackManager
from risk_assessment import RiskBasedController
from emergency_system import EmergencyStopSystem

class SafetyNode(Node):
    """ROS2 node for robot safety monitoring"""

    def __init__(self):
        super().__init__('safety_node')

        # Initialize safety systems
        self.safety_monitor = SafetyMonitor()
        self.fallback_manager = FallbackManager()
        self.risk_controller = RiskBasedController()
        self.emergency_system = EmergencyStopSystem()

        # Register emergency callback
        self.emergency_system.register_emergency_callback(self._emergency_callback)

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10)
        )

        self.velocity_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            QoSProfile(depth=10)
        )

        self.joint_state_sub = self.create_subscription(
            String,  # In practice, use sensor_msgs/JointState
            '/joint_states',
            self.joint_state_callback,
            QoSProfile(depth=10)
        )

        # Create publishers
        self.safety_pub = self.create_publisher(
            String,
            '/safety_status',
            QoSProfile(depth=10)
        )

        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            QoSProfile(depth=1)
        )

        self.safety_constraint_pub = self.create_publisher(
            Twist,
            '/safety_constrained_vel',
            QoSProfile(depth=10)
        )

        # Timer for continuous monitoring
        self.safety_timer = self.create_timer(0.1, self.safety_monitor_callback)

        self.get_logger().info('Safety node initialized')

    def scan_callback(self, msg):
        """Process laser scan data for safety monitoring"""
        try:
            # Process scan data to detect obstacles/humans
            human_positions = self._extract_human_positions(msg)

            # Evaluate proximity safety
            safety_event = self.safety_monitor.evaluate_proximity_safety(human_positions)
            if safety_event:
                events = [safety_event]
                self.safety_monitor.process_safety_events(events)

        except Exception as e:
            self.get_logger().error(f'Scan callback error: {e}')

    def velocity_callback(self, msg):
        """Process velocity commands for safety monitoring"""
        velocity = (msg.linear.x**2 + msg.linear.y**2 + msg.angular.z**2)**0.5

        if velocity > 1.0:  # Check velocity threshold
            event = SafetyEvent(
                condition=SafetyCondition.VELOCITY_LIMIT_EXCEEDED,
                level=SafetyLevel.WARNING,
                timestamp=self.get_clock().now().nanoseconds / 1e9,
                source="velocity_monitor",
                description=f"Velocity limit exceeded: {velocity:.2f}",
                parameters={"velocity": velocity}
            )

            self.safety_monitor.process_safety_events([event])

    def joint_state_callback(self, msg):
        """Process joint states for safety monitoring"""
        # Parse joint state message and evaluate joint safety
        # Implementation would check joint limits, velocities, etc.
        pass

    def safety_monitor_callback(self):
        """Continuous safety monitoring callback"""
        try:
            # Get current safety level
            current_level = self.safety_monitor.get_current_safety_level()

            # Publish safety status
            status_msg = String()
            status_msg.data = f"SAFETY_LEVEL: {current_level.name}"
            self.safety_pub.publish(status_msg)

            # Check if emergency stop is needed
            if current_level >= SafetyLevel.DANGER:
                emergency_msg = Bool()
                emergency_msg.data = True
                self.emergency_stop_pub.publish(emergency_msg)

            # Update risk-based constraints
            self._update_safety_constraints()

        except Exception as e:
            self.get_logger().error(f'Safety monitor error: {e}')

    def _extract_human_positions(self, scan_msg):
        """Extract potential human positions from laser scan"""
        # Simple clustering approach to identify humans
        points = []
        for i, range_val in enumerate(scan_msg.ranges):
            if range_val < scan_msg.range_max and range_val > scan_msg.range_min:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append((x, y, 0.0))  # Assuming 2D plane

        # Cluster points that might represent humans
        human_positions = []
        clusters = self._cluster_points(points)

        for cluster in clusters:
            if len(cluster) >= 3:  # Minimum points for a human-like cluster
                # Calculate centroid
                avg_x = sum(p[0] for p in cluster) / len(cluster)
                avg_y = sum(p[1] for p in cluster) / len(cluster)
                human_positions.append((avg_x, avg_y, 0.0))

        return human_positions

    def _cluster_points(self, points, threshold=0.3):
        """Cluster points that are close together"""
        clusters = []
        unassigned = set(range(len(points)))

        while unassigned:
            cluster = []
            start_idx = unassigned.pop()
            cluster.append(points[start_idx])

            # Find all points within threshold distance
            to_check = [start_idx]

            while to_check:
                current_idx = to_check.pop()
                current_point = points[current_idx]

                # Check remaining unassigned points
                remaining = list(unassigned)
                for idx in remaining:
                    other_point = points[idx]
                    dist = math.sqrt(
                        (current_point[0] - other_point[0])**2 +
                        (current_point[1] - other_point[1])**2
                    )
                    if dist < threshold:
                        cluster.append(other_point)
                        unassigned.remove(idx)
                        to_check.append(idx)

            clusters.append(cluster)

        return clusters

    def _update_safety_constraints(self):
        """Update safety constraints based on current situation"""
        # This would publish constrained velocity commands
        # based on current safety level and risk assessment
        pass

    def _emergency_callback(self):
        """Callback when emergency is triggered"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
        # Additional emergency-specific actions

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()

    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety Testing and Validation

### Safety Test Framework

```python
# safety_test.py
import unittest
import numpy as np
from safety_framework import SafetyMonitor, SafetyLevel, SafetyCondition
from fallback_system import FallbackManager, FallbackStrategy
from risk_assessment import RiskAssessment, RiskLevel

class TestSafetyFramework(unittest.TestCase):
    def setUp(self):
        self.safety_monitor = SafetyMonitor()
        self.fallback_manager = FallbackManager()
        self.risk_assessment = RiskAssessment()

    def test_safety_monitor_initial_state(self):
        """Test initial safety monitor state"""
        self.assertEqual(self.safety_monitor.get_current_safety_level(), SafetyLevel.SAFE)

    def test_proximity_safety_detection(self):
        """Test proximity safety detection"""
        # Test with human very close
        close_human = [(0.1, 0.0, 0.0)]
        event = self.safety_monitor.evaluate_proximity_safety(close_human)

        self.assertIsNotNone(event)
        self.assertEqual(event.condition, SafetyCondition.HUMAN_PROXIMITY)
        self.assertEqual(event.level, SafetyLevel.DANGER)

    def test_fallback_strategy_selection(self):
        """Test fallback strategy selection"""
        strategy = self.fallback_manager.fallback_system.determine_fallback('navigation_failure')
        self.assertEqual(strategy, FallbackStrategy.RETURN_TO_HOME)

    def test_risk_level_mapping(self):
        """Test risk level mapping"""
        risk_level = self.risk_assessment._map_to_risk_level(0.85)
        self.assertEqual(risk_level, RiskLevel.CRITICAL)

    def test_safety_constraint_updates(self):
        """Test safety constraint updates based on risk level"""
        constraints = self.risk_assessment.update_safety_constraints(RiskLevel.HIGH)

        # With HIGH risk, velocity should be reduced
        self.assertLess(constraints['max_velocity'], 1.0)

class SafetyValidator:
    """Validates safety system behavior"""

    def __init__(self):
        self.test_results = []
        self.risk_scenarios = [
            # (human_positions, robot_velocity, expected_risk)
            ([(1.0, 0.0, 0.0)], 0.5, RiskLevel.LOW),
            ([(0.3, 0.0, 0.0)], 0.5, RiskLevel.HIGH),
            ([(0.1, 0.0, 0.0)], 1.0, RiskLevel.CRITICAL),
        ]

    def validate_risk_assessment(self):
        """Validate risk assessment accuracy"""
        results = []

        for human_pos, velocity, expected_risk in self.risk_scenarios:
            robot_pos = (0.0, 0.0, 0.0)
            risk_level = self.risk_assessment.assess_environmental_risk(
                human_pos, robot_pos, velocity
            )

            success = risk_level == expected_risk
            results.append({
                'human_positions': human_pos,
                'robot_velocity': velocity,
                'expected_risk': expected_risk,
                'actual_risk': risk_level,
                'success': success
            })

        return results

    def run_safety_validation_suite(self):
        """Run comprehensive safety validation"""
        validation_results = {
            'risk_assessment': self.validate_risk_assessment(),
            'unit_tests_passed': self._run_unit_tests(),
        }

        return validation_results

    def _run_unit_tests(self):
        """Run unit tests for safety components"""
        test_suite = unittest.TestLoader().loadTestsFromTestCase(TestSafetyFramework)
        runner = unittest.TextTestRunner(stream=open('/dev/null', 'w'))
        result = runner.run(test_suite)

        return result.wasSuccessful()

def safety_validation_main():
    """Main function to run safety validation"""
    validator = SafetyValidator()
    results = validator.run_safety_validation_suite()

    print("Safety Validation Results:")
    print(f"Unit tests passed: {results['unit_tests_passed']}")
    print(f"Risk assessment accuracy: {sum(1 for r in results['risk_assessment'] if r['success'])}/{len(results['risk_assessment'])}")

if __name__ == "__main__":
    safety_validation_main()
```

## Summary

Safety and fallback behaviors are fundamental to the reliable operation of humanoid robots in human environments. The comprehensive safety framework includes:

1. **Multi-layered safety**: Hardware, control, behavioral, and system-level safety measures
2. **Real-time monitoring**: Continuous assessment of safety conditions and risk levels
3. **Intelligent fallbacks**: Context-aware fallback strategies that maintain safety while preserving functionality
4. **Emergency procedures**: Robust emergency stop systems with proper callback handling
5. **Risk-based adaptation**: Dynamic adjustment of robot behavior based on assessed risk
6. **Validation and testing**: Comprehensive validation frameworks to ensure safety system reliability

The integration of these safety systems with ROS2 and the broader humanoid robot architecture ensures that robots can operate safely while maintaining the ability to handle failures gracefully and continue operation when possible. Proper safety implementation is not just about preventing accidents—it's about enabling humanoid robots to work effectively alongside humans with appropriate risk management.