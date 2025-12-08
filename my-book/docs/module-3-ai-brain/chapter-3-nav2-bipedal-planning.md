---
title: "Nav2 for bipedal path planning"
description: "Implementing navigation 2 for humanoid robot path planning with bipedal locomotion constraints"
learning_objectives:
  - "Understand Nav2 architecture and components for humanoid navigation"
  - "Configure Nav2 for bipedal locomotion constraints"
  - "Implement custom path planners for humanoid robots"
  - "Integrate perception data with navigation system"
---

# Nav2 for bipedal path planning

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand Nav2 architecture and components for humanoid navigation
- Configure Nav2 for bipedal locomotion constraints
- Implement custom path planners for humanoid robots
- Integrate perception data with navigation system

## Introduction

Navigation 2 (Nav2) is ROS 2's state-of-the-art navigation framework that provides path planning, path execution, and obstacle avoidance capabilities. For humanoid robots, Nav2 requires special configuration to account for bipedal locomotion constraints, balance requirements, and unique kinematic properties. This chapter will guide you through adapting Nav2 for humanoid robot navigation, including custom planners, costmaps optimized for bipedal movement, and integration with perception systems.

## Understanding Nav2 Architecture

### Nav2 Components Overview

Nav2 consists of several key components that work together to provide navigation capabilities:

- **Navigation Server**: Main orchestrator that coordinates all navigation tasks
- **Planner Server**: Global path planning component
- **Controller Server**: Local path following and obstacle avoidance
- **Recovery Server**: Recovery behaviors for handling navigation failures
- **BT Navigator**: Behavior tree-based navigation executor
- **Lifecycle Manager**: Manages the lifecycle of navigation components

### Standard Nav2 Configuration

```yaml
# config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_consistent_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 24
      control_horizon: 12
      trajectory_resolution: 0.25
      frequency: 20.0
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true
      critic_names: [
        "BaseObstacleCritic",
        "GoalCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathProgressCritic",
        "GoalAngleCritic",
        "OscillationCritic",
        "PreferForwardCritic"]

      BaseObstacleCritic:
        plugin: "nav2_mppi_controller::BaseObstacleCritic"
        threshold_to_consider: 0.05
        scaling_factor: 0.0
        inflation_cost_scaling_factor: 3.0

      GoalCritic:
        plugin: "nav2_mppi_controller::GoalCritic"
        threshold_to_consider: 1.0
        scaling_factor: 1.0

      PathAlignCritic:
        plugin: "nav2_mppi_controller::PathAlignCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.5
        path_angle_thresh: 0.785

      PathFollowCritic:
        plugin: "nav2_mppi_controller::PathFollowCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.5

      PathProgressCritic:
        plugin: "nav2_mppi_controller::PathProgressCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.5

      GoalAngleCritic:
        plugin: "nav2_mppi_controller::GoalAngleCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.3

      OscillationCritic:
        plugin: "nav2_mppi_controller::OscillationCritic"
        threshold_to_consider: 0.3
        scaling_factor: 1.0

      PreferForwardCritic:
        plugin: "nav2_mppi_controller::PreferForwardCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.2
        penalty_angle: 1.57

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Humanoid-specific radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: False
      robot_radius: 0.3  # Humanoid-specific radius
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## Bipedal-Specific Nav2 Configuration

### Humanoid Navigation Parameters

For humanoid robots, we need to modify the standard Nav2 configuration to account for bipedal locomotion:

```yaml
# config/humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.1  # Reduced for more stable humanoid pose estimation
    alpha2: 0.1
    alpha3: 0.1
    alpha4: 0.1
    alpha5: 0.05  # Lower for better balance-aware pose estimation
    base_frame_id: "base_footprint"  # Humanoid-specific
    global_frame_id: "map"
    odom_frame_id: "odom"
    robot_model_type: "nav2_amcl::OmniMotionModel"  # Consider omnidirectional for humanoid
    save_pose_rate: 0.2  # Slower updates for stability
    sigma_hit: 0.3  # Larger uncertainty for humanoid dynamics
    tf_broadcast: true
    transform_tolerance: 1.5  # More tolerance for humanoid balance
    update_min_a: 0.1  # Less frequent updates for stability
    update_min_d: 0.1  # Less frequent updates for stability
    z_hit: 0.7  # Higher weight for expected measurements
    z_max: 0.05
    z_rand: 0.25  # More randomness for dynamic humanoid
    z_short: 0.05

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 10.0  # Lower frequency for humanoid stability
    min_x_velocity_threshold: 0.01  # Very low for careful movement
    min_y_velocity_threshold: 0.01  # Very low for careful movement
    min_theta_velocity_threshold: 0.01  # Very low for careful rotation
    failure_tolerance: 0.5  # Higher tolerance for humanoid balance recovery
    progress_checker_plugin: "humanoid_progress_checker"
    goal_checker_plugin: "humanoid_goal_checker"
    controller_plugins: ["HumanoidMppiController"]

    # Humanoid-specific controller with balance considerations
    HumanoidMppiController:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 30  # Longer horizon for balance planning
      control_horizon: 15  # Longer control horizon
      trajectory_resolution: 0.1  # Higher resolution for precise movement
      frequency: 10.0  # Lower frequency for stability
      xy_goal_tolerance: 0.3  # Larger tolerance for humanoid positioning
      yaw_goal_tolerance: 0.3  # Larger tolerance for humanoid orientation
      stateful: true
      critic_names: [
        "HumanoidObstacleCritic",
        "BalanceCritic",
        "GoalCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathProgressCritic",
        "GoalAngleCritic",
        "OscillationCritic",
        "PreferForwardCritic",
        "StepConstraintCritic"]

      # Humanoid-specific critics
      HumanoidObstacleCritic:
        plugin: "nav2_mppi_controller::BaseObstacleCritic"
        threshold_to_consider: 0.1  # Higher threshold for safety
        scaling_factor: 0.0
        inflation_cost_scaling_factor: 5.0  # Higher for safety

      BalanceCritic:
        plugin: "humanoid_nav2_plugins::BalanceCritic"
        threshold_to_consider: 0.5
        scaling_factor: 2.0  # High weight for balance
        com_height_threshold: 0.8  # Expected CoM height for humanoid
        stability_margin: 0.15  # Stability margin for bipedal

      GoalCritic:
        plugin: "nav2_mppi_controller::GoalCritic"
        threshold_to_consider: 1.0
        scaling_factor: 1.0

      PathAlignCritic:
        plugin: "nav2_mppi_controller::PathAlignCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.5
        path_angle_thresh: 0.5  # More conservative turning for humanoid

      PathFollowCritic:
        plugin: "nav2_mppi_controller::PathFollowCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.5

      PathProgressCritic:
        plugin: "nav2_mppi_controller::PathProgressCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.5

      GoalAngleCritic:
        plugin: "nav2_mppi_controller::GoalAngleCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.3

      OscillationCritic:
        plugin: "nav2_mppi_controller::OscillationCritic"
        threshold_to_consider: 0.5  # Higher for humanoid stability
        scaling_factor: 1.0

      PreferForwardCritic:
        plugin: "nav2_mppi_controller::PreferForwardCritic"
        threshold_to_consider: 0.5
        scaling_factor: 0.1  # Lower weight to allow more turning if needed

      StepConstraintCritic:
        plugin: "humanoid_nav2_plugins::StepConstraintCritic"
        threshold_to_consider: 0.5
        scaling_factor: 1.5  # Consider step constraints
        max_step_width: 0.3  # Maximum step width for humanoid
        max_step_height: 0.15  # Maximum step height for humanoid

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 3.0  # Lower for humanoid stability
      publish_frequency: 1.0
      global_frame: "odom"
      robot_base_frame: "base_footprint"  # Humanoid-specific
      use_sim_time: False
      rolling_window: true
      width: 8  # Larger window for humanoid planning
      height: 8
      resolution: 0.05  # High resolution for precise planning
      robot_radius: 0.4  # Larger for humanoid safety margin
      plugins: ["voxel_layer", "inflation_layer", "humanoid_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0  # Higher for safety
        inflation_radius: 0.8  # Larger inflation for humanoid
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.15  # Higher resolution in Z for humanoid
        z_voxels: 12  # More voxels for better height representation
        max_obstacle_height: 2.5  # Higher for humanoid environment
        mark_threshold: 0
        observation_sources: scan point_cloud
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.5
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0  # Longer range for humanoid planning
          raytrace_min_range: 0.1
          obstacle_max_range: 4.0  # Longer range for humanoid planning
          obstacle_min_range: 0.1
        point_cloud:
          topic: "/depth_camera/points"
          max_obstacle_height: 2.5
          clearing: True
          marking: True
          data_type: "PointCloud2"
          min_obstacle_height: 0.1
          obstacle_range: 4.0
          raytrace_range: 5.0
      humanoid_layer:
        plugin: "humanoid_nav2_plugins::HumanoidLayer"
        enabled: True
        footprint_padding: 0.1
        max_z: 2.0
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: "imu_data"
        obstacle_range: 4.0
        raytrace_range: 5.0
        origin_z: 0.0
        z_resolution: 0.1
        z_voxels: 20
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5  # Much lower for global map
      publish_frequency: 0.2
      global_frame: "map"
      robot_base_frame: "base_footprint"  # Humanoid-specific
      use_sim_time: False
      robot_radius: 0.4  # Larger safety radius
      resolution: 0.05  # High resolution
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "humanoid_traversable_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.5
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0  # Higher for safety
        inflation_radius: 0.8  # Larger for humanoid safety
      humanoid_traversable_layer:
        plugin: "humanoid_nav2_plugins::HumanoidTraversableLayer"
        enabled: True
        traversable_cost: 50  # Cost for traversable but challenging terrain
        non_traversable_cost: 254  # Maximum cost for non-traversable
        step_height_threshold: 0.1  # Maximum step height humanoid can handle
        slope_threshold: 0.3  # Maximum slope humanoid can handle
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # Lower frequency for humanoid planning
    planner_plugins: ["HumanoidGridPlanner"]
    HumanoidGridPlanner:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.8  # Higher tolerance for humanoid path planning
      use_astar: true  # A* for better path quality
      allow_unknown: true
      visualize_potential: false
```

## Custom Path Planners for Humanoid Robots

### Humanoid-Aware Path Planner

```python
# humanoid_planners.py - Custom path planners for humanoid robots
import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
from rclpy.action import ActionServer, ActionGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Lock
import numpy as np
import math
from typing import List, Tuple, Optional

class HumanoidPathPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_path_planner')

        # Action server for path computation
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self._compute_path_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Service clients for costmap access
        self.costmap_client = self.create_client(
            GetCostmap,
            'global_costmap/get_costmap',
            callback_group=ReentrantCallbackGroup()
        )

        # Robot parameters for humanoid navigation
        self.robot_params = {
            'step_length': 0.3,      # Maximum step length
            'step_width': 0.4,       # Maximum step width
            'step_height': 0.15,     # Maximum step height
            'turn_radius': 0.5,      # Minimum turning radius
            'max_slope': 0.3,        # Maximum traversable slope
            'base_width': 0.6,       # Robot base width
            'base_length': 0.4,      # Robot base length
            'com_height': 0.8        # Center of mass height
        }

        # Costmap storage
        self.costmap = None
        self.costmap_lock = Lock()

        # Initialize costmap
        self.get_logger().info('Humanoid Path Planner initialized')

    def _compute_path_callback(self, goal_handle: ActionGoalHandle) -> ComputePathToPose.Result:
        """Compute path for humanoid robot with bipedal constraints"""
        self.get_logger().info('Received path planning request')

        # Get goal pose
        goal_pose = goal_handle.request.goal
        planner_id = goal_handle.request.planner_id
        tolerance = goal_handle.request.tolerance

        # Get current costmap
        with self.costmap_lock:
            if self.costmap is None:
                self.get_logger().error('Costmap not available')
                result = ComputePathToPose.Result()
                result.path = Path()
                goal_handle.succeed()
                return result

        # Plan path considering humanoid constraints
        path = self.plan_humanoid_path(
            goal_pose.pose,
            tolerance,
            self.costmap
        )

        # Create result
        result = ComputePathToPose.Result()
        result.path = path

        if len(path.poses) > 0:
            goal_handle.succeed()
            self.get_logger().info('Path planning succeeded')
        else:
            goal_handle.abort()
            self.get_logger().error('Path planning failed')

        return result

    def plan_humanoid_path(self, goal_pose, tolerance, costmap) -> Path:
        """Plan path considering humanoid-specific constraints"""
        # Convert costmap to numpy array for path planning
        costmap_array = self.costmap_to_array(costmap)

        # Get start pose (assuming current robot pose)
        start_pose = self.get_current_robot_pose()

        # Perform A* path planning with humanoid constraints
        path_points = self.humanoid_astar(
            start_pose,
            goal_pose,
            costmap_array,
            costmap.info.resolution,
            tolerance
        )

        # Smooth path for humanoid locomotion
        smoothed_path = self.smooth_humanoid_path(path_points)

        # Convert to Path message
        path_msg = self.path_points_to_msg(smoothed_path, costmap.header.frame_id)

        return path_msg

    def humanoid_astar(self, start_pose, goal_pose, costmap, resolution, tolerance) -> List[Tuple[float, float]]:
        """A* algorithm modified for humanoid constraints"""
        # Convert poses to grid coordinates
        start_x, start_y = self.pose_to_grid(start_pose, costmap, resolution)
        goal_x, goal_y = self.pose_to_grid(goal_pose, costmap, resolution)

        # Check if start and goal are valid
        if not self.is_valid_position(start_x, start_y, costmap) or \
           not self.is_valid_position(goal_x, goal_y, costmap):
            return []

        # A* algorithm implementation
        open_set = [(start_x, start_y)]
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic((start_x, start_y), (goal_x, goal_y))}

        while open_set:
            # Find node with lowest f_score
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))

            if self.heuristic(current, (goal_x, goal_y)) <= tolerance / resolution:
                # Reconstruct path
                path = self.reconstruct_path(came_from, current)
                return path

            open_set.remove(current)

            # Check 8-connected neighbors
            for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_valid_position(neighbor[0], neighbor[1], costmap):
                    continue

                # Check humanoid-specific constraints
                if not self.is_humanoid_traversable(current, neighbor, costmap, resolution):
                    continue

                # Calculate tentative g_score
                movement_cost = self.calculate_movement_cost(current, neighbor, costmap)
                tentative_g_score = g_score[current] + movement_cost

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, (goal_x, goal_y))

                    if neighbor not in open_set:
                        open_set.append(neighbor)

        # No path found
        return []

    def is_humanoid_traversable(self, current, neighbor, costmap, resolution) -> bool:
        """Check if movement between two points is traversable for humanoid"""
        # Check basic costmap constraints
        if costmap[neighbor[1], neighbor[0]] >= 254:  # Occupied
            return False

        # Check step constraints
        step_distance = math.sqrt((neighbor[0] - current[0])**2 + (neighbor[1] - current[1])**2) * resolution
        if step_distance > self.robot_params['step_length']:
            return False

        # Check if the path between current and neighbor is clear
        # (For diagonal moves, check intermediate points)
        if abs(neighbor[0] - current[0]) == 1 and abs(neighbor[1] - current[1]) == 1:
            # Diagonal move - check both adjacent cells
            if costmap[current[1], neighbor[0]] >= 254 or costmap[neighbor[1], current[0]] >= 254:
                return False

        return True

    def calculate_movement_cost(self, current, neighbor, costmap):
        """Calculate movement cost considering humanoid constraints"""
        base_cost = costmap[neighbor[1], neighbor[0]]

        # Add penalty for high-cost areas
        if base_cost > 200:  # High cost area
            return base_cost * 2
        elif base_cost > 150:  # Medium cost area
            return base_cost * 1.5
        else:
            return base_cost

    def heuristic(self, pos1, pos2):
        """Calculate heuristic distance (Euclidean)"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def is_valid_position(self, x, y, costmap):
        """Check if position is valid in costmap"""
        return 0 <= x < costmap.shape[1] and 0 <= y < costmap.shape[0]

    def pose_to_grid(self, pose, costmap, resolution):
        """Convert pose to grid coordinates"""
        # This is a simplified conversion
        # In practice, you'd use proper transformation
        origin_x = 0  # This would come from costmap info
        origin_y = 0

        grid_x = int((pose.position.x - origin_x) / resolution)
        grid_y = int((pose.position.y - origin_y) / resolution)

        return grid_x, grid_y

    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def smooth_humanoid_path(self, path_points):
        """Smooth path for humanoid locomotion"""
        if len(path_points) < 3:
            return path_points

        # Apply path smoothing that considers humanoid constraints
        smoothed_path = [path_points[0]]

        i = 0
        while i < len(path_points) - 2:
            j = i + 2

            # Find the furthest point that can be reached directly
            while j < len(path_points):
                if self.is_direct_path_clear(path_points[i], path_points[j], path_points):
                    j += 1
                else:
                    break

            # Add the previous valid point
            smoothed_path.append(path_points[j-1])
            i = j - 1

        # Add the goal if it's not already included
        if smoothed_path[-1] != path_points[-1]:
            smoothed_path.append(path_points[-1])

        return smoothed_path

    def is_direct_path_clear(self, start, end, all_points):
        """Check if direct path between start and end is clear"""
        # This is a simplified check
        # In practice, you'd implement line-of-sight checking
        return True

    def path_points_to_msg(self, path_points, frame_id):
        """Convert path points to ROS Path message"""
        path_msg = Path()
        path_msg.header.frame_id = frame_id

        for point in path_points:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            # Convert grid coordinates back to world coordinates
            # This is simplified - you'd use proper transformation
            pose.pose.position.x = point[0] * 0.05  # Assuming 0.05m resolution
            pose.pose.position.y = point[1] * 0.05
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        return path_msg

    def get_current_robot_pose(self):
        """Get current robot pose (simplified)"""
        # In practice, you'd get this from TF or localization
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def costmap_to_array(self, costmap_msg):
        """Convert costmap message to numpy array"""
        width = costmap_msg.metadata.size_x
        height = costmap_msg.metadata.size_y
        data = np.array(costmap_msg.data).reshape((height, width))
        return data

class HumanoidControllerServer(Node):
    def __init__(self):
        super().__init__('humanoid_controller_server')

        # This would implement the controller server interface
        # with humanoid-specific control algorithms
        self.get_logger().info('Humanoid Controller Server initialized')

    def follow_path(self, path, goal_checker, progress_checker):
        """Follow path with humanoid-specific control"""
        # Implementation would include:
        # - Bipedal locomotion control
        # - Balance maintenance
        # - Step planning
        # - Fall prevention
        pass

def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    planner = HumanoidPathPlanner()

    # Use multi-threaded executor to handle multiple requests
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(planner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Perception Integration with Navigation

### Integrating Perception Data

```python
# perception_integration.py - Integrate perception with navigation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge

class PerceptionToNavigationIntegrator(Node):
    def __init__(self):
        super().__init__('perception_to_navigation_integrator')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Navigation-related parameters
        self.humanoid_radius = 0.4  # Safety radius for humanoid
        self.dynamic_object_buffer = 0.6  # Buffer for moving objects

        # Detected objects storage
        self.detected_objects = {}
        self.object_timestamps = {}
        self.object_velocities = {}

        # QoS for sensor data (best effort for performance)
        self.sensor_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Publishers for navigation system
        self.dynamic_costmap_pub = self.create_publisher(
            MarkerArray, 'navigation/dynamic_obstacles', 10
        )

        # Subscribers for perception data
        self.detections_sub = self.create_subscription(
            Detection2DArray, 'perception/detections',
            self.detections_callback, self.sensor_qos
        )

        self.segmentation_sub = self.create_subscription(
            Image, 'segmentation/mask',
            self.segmentation_callback, self.sensor_qos
        )

        self.depth_sub = self.create_subscription(
            PointCloud2, 'depth_camera/points',
            self.pointcloud_callback, 10
        )

        # Timer for processing and publishing
        self.process_timer = self.create_timer(0.1, self.process_and_publish)  # 10 Hz

        self.get_logger().info('Perception to Navigation Integrator initialized')

    def detections_callback(self, msg):
        """Process object detections and update navigation system"""
        current_time = self.get_clock().now()

        for detection in msg.detections:
            if len(detection.results) > 0:
                # Get the most confident result
                best_result = max(detection.results, key=lambda x: x.hypothesis.score)

                object_class = best_result.hypothesis.class_id
                confidence = best_result.hypothesis.score
                bbox = detection.bbox

                # Only consider high-confidence detections of interest
                if confidence > 0.7 and self.is_object_of_interest(object_class):
                    # Calculate object position in 3D (simplified)
                    obj_3d_pos = self.calculate_3d_position(bbox, msg.header.frame_id)

                    if obj_3d_pos is not None:
                        object_id = f"{object_class}_{len(self.detected_objects)}"

                        # Store object information
                        self.detected_objects[object_id] = {
                            'class': object_class,
                            'position': obj_3d_pos,
                            'bbox': bbox,
                            'confidence': confidence,
                            'timestamp': current_time
                        }

                        # Calculate velocity if we have previous position
                        if object_id in self.object_timestamps:
                            time_diff = (current_time - self.object_timestamps[object_id]).nanoseconds / 1e9
                            if time_diff > 0:
                                pos_diff = np.array([obj_3d_pos.x, obj_3d_pos.y, obj_3d_pos.z]) - \
                                          np.array([self.detected_objects[object_id]['position'].x,
                                                   self.detected_objects[object_id]['position'].y,
                                                   self.detected_objects[object_id]['position'].z])
                                velocity = pos_diff / time_diff
                                self.object_velocities[object_id] = velocity

                        # Update timestamp
                        self.object_timestamps[object_id] = current_time

    def segmentation_callback(self, msg):
        """Process segmentation masks for navigation"""
        try:
            # Convert segmentation mask to OpenCV
            seg_mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')

            # Process segmentation for navigable areas
            navigable_areas = self.extract_navigable_areas(seg_mask)

            # Update navigation costmap with traversability information
            self.update_traversability_costmap(navigable_areas)

        except Exception as e:
            self.get_logger().error(f'Error processing segmentation: {e}')

    def pointcloud_callback(self, msg):
        """Process point cloud data for 3D obstacle detection"""
        # Process point cloud for 3D obstacle detection
        # This would involve processing PointCloud2 message
        # and updating 3D costmap for navigation
        pass

    def calculate_3d_position(self, bbox, frame_id):
        """Calculate 3D position of detected object"""
        # This is a simplified approach
        # In practice, you'd use stereo triangulation or depth information
        center_x = int(bbox.center.x)
        center_y = int(bbox.center.y)

        # For now, return a placeholder position
        # You would need depth information to calculate actual 3D position
        pos = Point()
        pos.x = float(center_x * 0.01)  # Placeholder conversion
        pos.y = float(center_y * 0.01)
        pos.z = 0.0
        return pos

    def is_object_of_interest(self, object_class):
        """Check if object class is relevant for navigation"""
        # Define classes that affect navigation
        navigation_relevant_classes = [
            'person', 'car', 'truck', 'bus', 'bicycle', 'motorcycle',
            'chair', 'couch', 'dining table', 'bed', 'toilet', 'tv'
        ]
        return object_class in navigation_relevant_classes

    def extract_navigable_areas(self, seg_mask):
        """Extract navigable areas from segmentation mask"""
        # Define which semantic classes are navigable
        navigable_classes = {
            'floor': 1,
            'carpet': 2,
            'grass': 3,
            'road': 4,
            'sidewalk': 5
        }

        navigable_mask = np.zeros_like(seg_mask, dtype=bool)

        for class_id in navigable_classes.values():
            navigable_mask |= (seg_mask == class_id)

        return navigable_mask

    def update_traversability_costmap(self, navigable_areas):
        """Update navigation costmap based on traversability"""
        # This would update the costmap used by Nav2
        # with information about traversable vs non-traversable areas
        pass

    def process_and_publish(self):
        """Process detected objects and publish to navigation system"""
        if not self.detected_objects:
            return

        # Create marker array for visualization
        marker_array = MarkerArray()

        for obj_id, obj_data in self.detected_objects.items():
            # Create marker for the object
            marker = Marker()
            marker.header.frame_id = "map"  # This should be properly transformed
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dynamic_obstacles"
            marker.id = hash(obj_id) % 10000  # Ensure unique ID
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Set position
            marker.pose.position = obj_data['position']
            marker.pose.orientation.w = 1.0

            # Set size (radius and height based on object class)
            if obj_data['class'] == 'person':
                marker.scale.x = 0.6  # Diameter
                marker.scale.y = 0.6
                marker.scale.z = 1.8  # Height
            else:
                marker.scale.x = 0.8
                marker.scale.y = 0.8
                marker.scale.z = 1.0

            # Set color based on object type
            color = self.get_object_color(obj_data['class'])
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8

            # Set lifetime (how long the marker persists)
            marker.lifetime.sec = 2
            marker.lifetime.nanosec = 0

            marker_array.markers.append(marker)

            # Also create a text marker with object info
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "object_labels"
            text_marker.id = marker.id + 10000  # Different ID space
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = marker.pose
            text_marker.pose.position.z += marker.scale.z / 2 + 0.2  # Above the object
            text_marker.text = f"{obj_data['class']}\n{obj_data['confidence']:.2f}"
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        # Publish the marker array
        if marker_array.markers:
            self.dynamic_costmap_pub.publish(marker_array)

    def get_object_color(self, obj_class):
        """Get color for visualizing different object classes"""
        colors = {
            'person': (1.0, 0.0, 0.0),      # Red
            'car': (0.0, 0.0, 1.0),         # Blue
            'chair': (1.0, 1.0, 0.0),       # Yellow
            'couch': (1.0, 0.0, 1.0),       # Magenta
            'dining table': (0.0, 1.0, 1.0), # Cyan
            'bicycle': (0.5, 0.5, 0.5),     # Gray
            'motorcycle': (0.0, 0.5, 1.0),  # Light blue
        }
        return colors.get(obj_class, (0.5, 0.5, 0.5))  # Default gray

def main(args=None):
    rclpy.init(args=args)
    integrator = PerceptionToNavigationIntegrator()

    try:
        rclpy.spin(integrator)
    except KeyboardInterrupt:
        pass
    finally:
        integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid Navigation Launch Files

### Complete Navigation Setup

```python
# launch/humanoid_navigation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('my_humanoid_navigation'),
            'config',
            'humanoid_nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Whether to use composed bringup'
    )

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition'
    )

    # Include the main Nav2 launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'container_name': container_name
        }.items()
    )

    # Humanoid-specific path planner
    humanoid_planner = Node(
        package='my_humanoid_navigation',
        executable='humanoid_path_planner',
        name='humanoid_path_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    # Perception to navigation integrator
    perception_integrator = Node(
        package='my_humanoid_navigation',
        executable='perception_to_navigation_integrator',
        name='perception_to_navigation_integrator',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    # Lifecycle manager for navigation
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['humanoid_path_planner',
                                  'perception_to_navigation_integrator',
                                  'controller_server',
                                  'planner_server',
                                  'recoveries_server',
                                  'bt_navigator',
                                  'waypoint_follower',
                                  'velocity_smoother']}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_use_composition,
        declare_container_name,
        nav2_bringup_launch,
        humanoid_planner,
        perception_integrator,
        lifecycle_manager
    ])
```

## Practical Exercise: Implement Humanoid Navigation System

Create a complete humanoid navigation system:

1. **Create the package structure**:
```bash
cd ros2_ws/src
ros2 pkg create --build-type ament_python my_humanoid_navigation
```

2. **Create a custom controller plugin**:

```python
# my_humanoid_navigation/humanoid_controller.py
import rclpy
from rclpy.node import Node
from nav2_core.controller import Controller
from nav2_util import NodeLifecycle
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import numpy as np

class HumanoidController(Controller):
    def __init__(self, name):
        super().__init__(name)
        self.logger = self.get_logger()
        self.logger.info(f'{self.__class__.__name__} is ready!')

    def configure(self, plugin_name, node, tf, costmap_ros):
        """Configure the controller with ROS parameters"""
        self.name = plugin_name
        self.node = node
        self.tf = tf
        self.costmap_ros = costmap_ros
        self.costmap = costmap_ros.get_costmap()

        # Humanoid-specific parameters
        self.step_size = 0.2  # Maximum step size for humanoid
        self.max_linear_speed = 0.3  # Slower for stability
        self.max_angular_speed = 0.5
        self.balance_threshold = 0.1  # Balance maintenance threshold

        self.logger.info(f'{self.name} fully configured')

    def cleanup(self):
        """Clean up resources"""
        self.logger.info(f'{self.name} was cleaned up')

    def activate(self):
        """Activate the controller"""
        self.logger.info(f'{self.name} was activated')

    def deactivate(self):
        """Deactivate the controller"""
        self.logger.info(f'{self.name} was deactivated')

    def setPlan(self, path: Path):
        """Set the plan for the controller to follow"""
        self.logger.info(f'Received new plan for {self.name}')
        self.path = path

    def computeVelocityCommands(self, pose: PoseStamped, velocity: Twist) -> Twist:
        """Compute velocity commands to follow the path"""
        # Create output velocity command
        cmd_vel = Twist()

        # For humanoid robots, we need to consider bipedal constraints
        # This is a simplified implementation

        # Calculate distance to goal
        if len(self.path.poses) > 0:
            goal_pose = self.path.poses[-1].pose
            current_pose = pose.pose

            dx = goal_pose.position.x - current_pose.position.x
            dy = goal_pose.position.y - current_pose.position.y
            distance_to_goal = np.sqrt(dx*dx + dy*dy)

            # Calculate angle to goal
            angle_to_goal = np.arctan2(dy, dx)
            current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)

            # Calculate angle difference
            angle_diff = angle_to_goal - current_yaw
            # Normalize angle to [-pi, pi]
            while angle_diff > np.pi:
                angle_diff -= 2*np.pi
            while angle_diff < -np.pi:
                angle_diff += 2*np.pi

            # Simple proportional control for humanoid
            # With constraints for bipedal stability
            linear_speed = min(self.max_linear_speed, distance_to_goal * 0.5)
            angular_speed = min(self.max_angular_speed, angle_diff * 1.0)

            # Apply humanoid-specific constraints
            cmd_vel.linear.x = max(0.05, linear_speed)  # Minimum speed to keep moving
            cmd_vel.angular.z = angular_speed

            # Additional constraints for bipedal locomotion
            # Limit acceleration for stability
            cmd_vel.linear.x = self.limit_acceleration(
                velocity.linear.x, cmd_vel.linear.x, 0.1  # 0.1 m/s^2 max acceleration
            )
            cmd_vel.angular.z = self.limit_angular_acceleration(
                velocity.angular.z, cmd_vel.angular.z, 0.5  # 0.5 rad/s^2 max angular acceleration
            )

        return cmd_vel

    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion"""
        import math
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def limit_acceleration(self, current_vel, desired_vel, max_acc):
        """Limit acceleration for humanoid stability"""
        if desired_vel > current_vel:
            limited_vel = min(desired_vel, current_vel + max_acc * 0.1)  # 0.1s time step
        else:
            limited_vel = max(desired_vel, current_vel - max_acc * 0.1)
        return limited_vel

    def limit_angular_acceleration(self, current_vel, desired_vel, max_acc):
        """Limit angular acceleration for humanoid stability"""
        if desired_vel > current_vel:
            limited_vel = min(desired_vel, current_vel + max_acc * 0.1)
        else:
            limited_vel = max(desired_vel, current_vel - max_acc * 0.1)
        return limited_vel
```

3. **Test the navigation system**:
```bash
# Build the packages
cd ros2_ws
colcon build --packages-select my_humanoid_navigation
source install/setup.bash

# Launch the navigation system
ros2 launch my_humanoid_navigation humanoid_navigation.launch.py

# Send a navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Troubleshooting Common Navigation Issues

### Path Planning Issues
- **Infeasible paths**: Check costmap inflation and robot footprint
- **Oscillation**: Adjust controller parameters and goal checkers
- **Failure to find path**: Verify map quality and costmap parameters

### Controller Issues
- **Unstable movement**: Reduce control frequency and adjust gains
- **Poor tracking**: Tune trajectory controller parameters
- **Frequent recovery**: Improve sensor quality and localization

### Perception Integration Issues
- **Delayed obstacle detection**: Optimize perception pipeline performance
- **False obstacles**: Adjust detection thresholds and filtering
- **Coordinate frame issues**: Verify TF tree and transformations

## Summary

In this chapter, we've explored implementing Nav2 for humanoid robot navigation with bipedal path planning. We covered the Nav2 architecture, configured it for humanoid-specific constraints, implemented custom path planners, and integrated perception data with the navigation system. Humanoid navigation requires special considerations for balance, step constraints, and bipedal locomotion patterns.

## Next Steps

- Configure Nav2 with the humanoid-specific parameters
- Implement and test custom path planners
- Integrate perception outputs with navigation
- Fine-tune controller parameters for stable humanoid movement
- Test navigation in various environments and scenarios