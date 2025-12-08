---
title: "Reinforcement learning for robot control"
description: "Implementing reinforcement learning algorithms for humanoid robot control and locomotion"
learning_objectives:
  - "Understand reinforcement learning fundamentals for robotics"
  - "Implement RL algorithms for humanoid locomotion control"
  - "Create reward functions for humanoid robot behaviors"
  - "Train and deploy RL policies for robot control"
---

# Reinforcement learning for robot control

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand reinforcement learning fundamentals for robotics
- Implement RL algorithms for humanoid locomotion control
- Create reward functions for humanoid robot behaviors
- Train and deploy RL policies for robot control

## Introduction

Reinforcement Learning (RL) has emerged as a powerful approach for learning complex control policies in robotics, particularly for humanoid robots that require sophisticated locomotion and manipulation skills. Unlike traditional control methods that rely on explicit mathematical models, RL enables robots to learn optimal behaviors through interaction with their environment. For humanoid robots, RL can be used to learn walking gaits, balance recovery, manipulation skills, and adaptive behaviors. This chapter will guide you through implementing RL algorithms specifically tailored for humanoid robot control.

## Reinforcement Learning Fundamentals for Robotics

### RL Framework Components

The RL framework consists of four main components:
- **Agent**: The humanoid robot learning to perform tasks
- **Environment**: The physical or simulated world where the robot operates
- **State**: The current configuration of the robot and environment
- **Action**: The control commands sent to the robot
- **Reward**: Feedback signal indicating the quality of actions

### Markov Decision Process (MDP)

Robot control can be formulated as an MDP with:
- State space S: Robot joint positions, velocities, IMU readings, etc.
- Action space A: Joint torques, desired positions, or high-level commands
- Transition dynamics P: How the robot moves from one state to another
- Reward function R: How to evaluate the quality of robot behavior
- Discount factor γ: How much to value future rewards

### Robotics-Specific Considerations

```python
# rl_basics.py - Basic RL concepts for robotics
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import gym
from gym import spaces
from collections import deque
import random

class HumanoidRobotEnv(gym.Env):
    """
    Custom gym environment for humanoid robot control
    This is a simplified example - real implementation would interface with simulation/real robot
    """
    def __init__(self, robot_interface=None):
        super(HumanoidRobotEnv, self).__init__()

        # Define action space: joint torques or position targets
        # For a humanoid with 12 DOF (6 for each leg, 6 for arms)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(12,), dtype=np.float32
        )

        # Define observation space: joint positions, velocities, IMU, etc.
        obs_dim = 36  # Example: 12 pos + 12 vel + 6 IMU + 6 other sensors
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        # Robot interface (simulator or real robot)
        self.robot = robot_interface
        self.max_episode_steps = 1000
        self.current_step = 0

        # Robot parameters
        self.target_position = np.array([5.0, 0.0])  # Target to walk to
        self.robot_position = np.array([0.0, 0.0])   # Current robot position
        self.initial_position = np.array([0.0, 0.0])

    def reset(self, seed=None):
        """Reset the environment to initial state"""
        super().reset(seed=seed)

        # Reset robot to initial position
        if self.robot:
            self.robot.reset_to_initial_pose()

        self.current_step = 0
        self.robot_position = self.initial_position.copy()

        # Return initial observation
        observation = self._get_observation()
        return observation

    def step(self, action):
        """Execute one step of the environment"""
        # Apply action to robot
        if self.robot:
            self.robot.apply_action(action)

        # Get new state from robot
        new_observation = self._get_observation()

        # Calculate reward
        reward = self._calculate_reward()

        # Check if episode is done
        self.current_step += 1
        done = self._check_termination()
        truncated = self.current_step >= self.max_episode_steps

        # Additional info
        info = {
            'robot_position': self.robot_position.copy(),
            'distance_to_target': np.linalg.norm(self.robot_position - self.target_position)
        }

        return new_observation, reward, done, truncated, info

    def _get_observation(self):
        """Get current observation from robot"""
        if self.robot:
            # Get joint positions and velocities
            joint_positions = self.robot.get_joint_positions()
            joint_velocities = self.robot.get_joint_velocities()

            # Get IMU data
            imu_orientation = self.robot.get_imu_orientation()
            imu_angular_velocity = self.robot.get_imu_angular_velocity()
            imu_linear_acceleration = self.robot.get_imu_linear_acceleration()

            # Get other sensors (contact sensors, etc.)
            contact_sensors = self.robot.get_contact_sensors()

            # Concatenate all observations
            observation = np.concatenate([
                joint_positions,
                joint_velocities,
                imu_orientation,
                imu_angular_velocity,
                imu_linear_acceleration,
                contact_sensors
            ])
        else:
            # For simulation, return dummy observation
            observation = np.random.randn(self.observation_space.shape[0]).astype(np.float32)

        return observation

    def _calculate_reward(self):
        """Calculate reward based on current state"""
        # Distance to target reward (positive for moving towards target)
        distance_to_target = np.linalg.norm(self.robot_position - self.target_position)
        distance_reward = -0.1 * distance_to_target  # Negative because closer is better

        # Survival reward (for staying upright and moving)
        survival_reward = 0.1

        # Velocity reward (for moving forward)
        # This would require tracking robot's forward velocity
        velocity_reward = 0.0  # Placeholder

        # Balance reward (for maintaining upright posture)
        imu_orientation = self._get_imu_orientation()  # Simplified
        upright_reward = 0.0
        if len(imu_orientation) >= 3:
            # Reward for staying upright (z-axis should be close to 1)
            upright_reward = 0.2 * imu_orientation[2]  # Assuming [x, y, z] orientation

        # Penalty for falling
        fall_penalty = 0.0
        if self._is_fallen():
            fall_penalty = -10.0

        total_reward = distance_reward + survival_reward + velocity_reward + upright_reward + fall_penalty
        return total_reward

    def _is_fallen(self):
        """Check if robot has fallen"""
        # Simplified fall detection
        # In practice, this would check robot's orientation and joint positions
        imu_orientation = self._get_imu_orientation()
        if len(imu_orientation) >= 3:
            # If z-component of orientation is too low, robot may have fallen
            return imu_orientation[2] < 0.5  # Threshold for "fallen"
        return False

    def _get_imu_orientation(self):
        """Get simplified IMU orientation"""
        # Placeholder - in real implementation, this would come from robot
        return np.array([0.0, 0.0, 1.0])  # Upright

    def _check_termination(self):
        """Check if episode should terminate"""
        return self._is_fallen()

# Example of a simple neural network for policy
class ActorNetwork(nn.Module):
    """Actor network for policy learning"""
    def __init__(self, state_dim, action_dim, max_action):
        super(ActorNetwork, self).__init__()

        self.l1 = nn.Linear(state_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, action_dim)

        self.max_action = max_action

    def forward(self, state):
        a = F.relu(self.l1(state))
        a = F.relu(self.l2(a))
        return self.max_action * torch.tanh(self.l3(a))

class CriticNetwork(nn.Module):
    """Critic network for value estimation"""
    def __init__(self, state_dim, action_dim):
        super(CriticNetwork, self).__init__()

        # Q1 architecture
        self.l1 = nn.Linear(state_dim + action_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, 1)

        # Q2 architecture
        self.l4 = nn.Linear(state_dim + action_dim, 256)
        self.l5 = nn.Linear(256, 256)
        self.l6 = nn.Linear(256, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)

        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)

        q2 = F.relu(self.l4(sa))
        q2 = F.relu(self.l5(q2))
        q2 = self.l6(q2)
        return q1, q2

    def Q1(self, state, action):
        sa = torch.cat([state, action], 1)

        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)
        return q1
```

## Deep Reinforcement Learning Algorithms for Humanoid Control

### Deep Deterministic Policy Gradient (DDPG)

DDPG is well-suited for continuous control tasks like humanoid locomotion:

```python
# ddpg_humanoid.py - DDPG implementation for humanoid control
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from collections import namedtuple, deque
import random

# Experience replay buffer
Transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward', 'done'))

class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, next_state, reward, done):
        self.buffer.append(Transition(state, action, next_state, reward, done))

    def sample(self, batch_size):
        return random.sample(self.buffer, batch_size)

    def __len__(self):
        return len(self.buffer)

class DDPGAgent:
    def __init__(self, state_dim, action_dim, max_action, lr_actor=1e-4, lr_critic=1e-3,
                 gamma=0.99, tau=0.005, buffer_size=int(1e6)):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Networks
        self.actor = ActorNetwork(state_dim, action_dim, max_action).to(self.device)
        self.actor_target = ActorNetwork(state_dim, action_dim, max_action).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)

        self.critic = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_target = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr_critic)

        # Hyperparameters
        self.gamma = gamma  # discount factor
        self.tau = tau      # soft update parameter
        self.max_action = max_action

        # Replay buffer
        self.memory = ReplayBuffer(buffer_size)

        # Noise for exploration
        self.noise_std = 0.1
        self.noise_max = 0.2

    def select_action(self, state, add_noise=True):
        """Select action using the current policy"""
        state = torch.FloatTensor(state).to(self.device).unsqueeze(0)
        action = self.actor(state).cpu().data.numpy().flatten()

        if add_noise:
            # Add Ornstein-Uhlenbeck noise for exploration
            noise = np.random.normal(0, self.noise_std, size=action.shape)
            noise = np.clip(noise, -self.noise_max, self.noise_max)
            action = action + noise

        return np.clip(action, -self.max_action, self.max_action)

    def train(self, batch_size=100):
        """Train the agent on a batch of experiences"""
        if len(self.memory) < batch_size:
            return

        transitions = self.memory.sample(batch_size)
        batch = Transition(*zip(*transitions))

        state_batch = torch.FloatTensor(batch.state).to(self.device)
        action_batch = torch.FloatTensor(batch.action).to(self.device)
        next_state_batch = torch.FloatTensor(batch.next_state).to(self.device)
        reward_batch = torch.FloatTensor(batch.reward).to(self.device).unsqueeze(1)
        done_batch = torch.BoolTensor(batch.done).to(self.device).unsqueeze(1)

        # Compute target Q values
        with torch.no_grad():
            next_action = self.actor_target(next_state_batch)
            target_Q1, target_Q2 = self.critic_target(next_state_batch, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward_batch + (self.gamma * target_Q * ~done_batch)

        # Compute current Q values
        current_Q1, current_Q2 = self.critic(state_batch, action_batch)

        # Compute critic loss
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

        # Optimize critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Compute actor loss
        actor_loss = -self.critic.Q1(state_batch, self.actor(state_batch)).mean()

        # Optimize actor
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Soft update target networks
        for target_param, param in zip(self.actor_target.parameters(), self.actor.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

    def save(self, filename):
        """Save the model"""
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'actor_optimizer_state_dict': self.actor_optimizer.state_dict(),
            'critic_optimizer_state_dict': self.critic_optimizer.state_dict(),
        }, filename)

    def load(self, filename):
        """Load the model"""
        checkpoint = torch.load(filename)
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer_state_dict'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer_state_dict'])
```

### Twin Delayed DDPG (TD3)

TD3 improves upon DDPG with better stability:

```python
# td3_humanoid.py - TD3 implementation for humanoid control
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

class TD3Agent:
    def __init__(self, state_dim, action_dim, max_action, lr_actor=1e-4, lr_critic=1e-3,
                 gamma=0.99, tau=0.005, policy_noise=0.2, noise_clip=0.5, policy_freq=2):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.actor = ActorNetwork(state_dim, action_dim, max_action).to(self.device)
        self.actor_target = ActorNetwork(state_dim, action_dim, max_action).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)

        self.critic_1 = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_1_target = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_1_target.load_state_dict(self.critic_1.state_dict())
        self.critic_1_optimizer = optim.Adam(self.critic_1.parameters(), lr=lr_critic)

        self.critic_2 = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_2_target = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_2_target.load_state_dict(self.critic_2.state_dict())
        self.critic_2_optimizer = optim.Adam(self.critic_2.parameters(), lr=lr_critic)

        # Hyperparameters
        self.gamma = gamma
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_freq = policy_freq
        self.max_action = max_action
        self.total_it = 0

        # Replay buffer
        self.memory = ReplayBuffer(int(1e6))

    def select_action(self, state, add_noise=True):
        """Select action with optional noise"""
        state = torch.FloatTensor(state).to(self.device).unsqueeze(0)
        action = self.actor(state).cpu().data.numpy().flatten()

        if add_noise:
            noise = np.random.normal(0, self.policy_noise, size=action.shape)
            noise = np.clip(noise, -self.noise_clip, self.noise_clip)
            action = action + noise

        return np.clip(action, -self.max_action, self.max_action)

    def train(self, batch_size=100):
        """Train the TD3 agent"""
        self.total_it += 1

        if len(self.memory) < batch_size:
            return

        transitions = self.memory.sample(batch_size)
        batch = Transition(*zip(*transitions))

        state_batch = torch.FloatTensor(batch.state).to(self.device)
        action_batch = torch.FloatTensor(batch.action).to(self.device)
        next_state_batch = torch.FloatTensor(batch.next_state).to(self.device)
        reward_batch = torch.FloatTensor(batch.reward).to(self.device).unsqueeze(1)
        done_batch = torch.BoolTensor(batch.done).to(self.device).unsqueeze(1)

        # Select next action with noise (for target policy)
        with torch.no_grad():
            noise = torch.FloatTensor(action_batch).data.normal_(0, self.policy_noise).to(self.device)
            noise = noise.clamp(-self.noise_clip, self.noise_clip)

            next_action = (self.actor_target(next_state_batch) + noise).clamp(-self.max_action, self.max_action)

            # Compute target Q-value
            target_Q1, target_Q2 = self.critic_1_target(next_state_batch, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward_batch + (self.gamma * target_Q * ~done_batch)

        # Update critic networks
        current_Q1, current_Q2 = self.critic_1(state_batch, action_batch)
        current_Q3, current_Q4 = self.critic_2(state_batch, action_batch)

        critic_loss_1 = F.mse_loss(current_Q1, target_Q)
        critic_loss_2 = F.mse_loss(current_Q2, target_Q)
        critic_loss_3 = F.mse_loss(current_Q3, target_Q)
        critic_loss_4 = F.mse_loss(current_Q4, target_Q)

        critic_loss = critic_loss_1 + critic_loss_2 + critic_loss_3 + critic_loss_4

        self.critic_1_optimizer.zero_grad()
        critic_loss_1.backward()
        self.critic_1_optimizer.step()

        self.critic_2_optimizer.zero_grad()
        critic_loss_4.backward()  # Using Q4 for critic 2
        self.critic_2_optimizer.step()

        # Delayed policy updates
        if self.total_it % self.policy_freq == 0:
            # Compute actor loss
            actor_loss = -self.critic_1.Q1(state_batch, self.actor(state_batch)).mean()

            # Optimize actor
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            # Soft update target networks
            for param, target_param in zip(self.critic_1.parameters(), self.critic_1_target.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

            for param, target_param in zip(self.critic_2.parameters(), self.critic_2_target.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

            for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

    def save(self, filename):
        """Save the model"""
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'actor_target_state_dict': self.actor_target.state_dict(),
            'critic_1_state_dict': self.critic_1.state_dict(),
            'critic_1_target_state_dict': self.critic_1_target.state_dict(),
            'critic_2_state_dict': self.critic_2.state_dict(),
            'critic_2_target_state_dict': self.critic_2_target.state_dict(),
            'actor_optimizer_state_dict': self.actor_optimizer.state_dict(),
            'critic_1_optimizer_state_dict': self.critic_1_optimizer.state_dict(),
            'critic_2_optimizer_state_dict': self.critic_2_optimizer.state_dict(),
        }, filename)

    def load(self, filename):
        """Load the model"""
        checkpoint = torch.load(filename)
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.actor_target.load_state_dict(checkpoint['actor_target_state_dict'])
        self.critic_1.load_state_dict(checkpoint['critic_1_state_dict'])
        self.critic_1_target.load_state_dict(checkpoint['critic_1_target_state_dict'])
        self.critic_2.load_state_dict(checkpoint['critic_2_state_dict'])
        self.critic_2_target.load_state_dict(checkpoint['critic_2_target_state_dict'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer_state_dict'])
        self.critic_1_optimizer.load_state_dict(checkpoint['critic_1_optimizer_state_dict'])
        self.critic_2_optimizer.load_state_dict(checkpoint['critic_2_optimizer_state_dict'])
```

## Reward Function Design for Humanoid Robots

### Designing Effective Reward Functions

The reward function is crucial for successful RL training in humanoid robotics:

```python
# reward_functions.py - Reward function design for humanoid robots
import numpy as np

class HumanoidRewardFunction:
    """Class to compute various reward components for humanoid robots"""

    def __init__(self):
        # Weight parameters for different reward components
        self.weights = {
            'progress': 1.0,      # Reward for moving toward goal
            'survival': 0.1,      # Reward for staying alive/active
            'balance': 0.5,       # Reward for maintaining balance
            'smoothness': 0.1,    # Penalty for jerky movements
            'energy': 0.05,       # Penalty for high energy usage
            'upright': 1.0,       # Reward for staying upright
            'footstep': 0.2       # Reward for proper footstep placement
        }

        # Reference values
        self.reference_upright = np.array([0, 0, 1])  # World z-axis (up)
        self.max_joint_velocity = 5.0  # rad/s
        self.max_torque = 100.0        # Nm

    def compute_reward(self, state, action, next_state, info):
        """
        Compute total reward for humanoid robot
        state: current state vector
        action: action taken
        next_state: resulting state
        info: additional information
        """
        reward = 0.0

        # Parse state components (these would depend on your specific state representation)
        robot_pos = self._get_robot_position(state)
        robot_vel = self._get_robot_velocity(state)
        imu_orientation = self._get_imu_orientation(state)
        imu_angular_vel = self._get_imu_angular_velocity(state)
        joint_positions = self._get_joint_positions(state)
        joint_velocities = self._get_joint_velocities(state)
        joint_torques = self._get_joint_torques(action)  # From action or computed

        # 1. Progress reward - reward for moving toward goal
        progress_reward = self._compute_progress_reward(robot_pos, info)
        reward += self.weights['progress'] * progress_reward

        # 2. Survival reward - reward for staying active
        survival_reward = self.weights['survival']
        reward += survival_reward

        # 3. Balance reward - reward for maintaining stable pose
        balance_reward = self._compute_balance_reward(imu_orientation, imu_angular_vel)
        reward += self.weights['balance'] * balance_reward

        # 4. Smoothness penalty - penalty for jerky movements
        smoothness_penalty = self._compute_smoothness_penalty(joint_velocities)
        reward -= self.weights['smoothness'] * smoothness_penalty

        # 5. Energy penalty - penalty for high energy consumption
        energy_penalty = self._compute_energy_penalty(joint_torques, joint_velocities)
        reward -= self.weights['energy'] * energy_penalty

        # 6. Upright posture reward - reward for staying upright
        upright_reward = self._compute_upright_reward(imu_orientation)
        reward += self.weights['upright'] * upright_reward

        # 7. Footstep reward - reward for proper stepping (if applicable)
        footstep_reward = self._compute_footstep_reward(state, action, info)
        reward += self.weights['footstep'] * footstep_reward

        return reward

    def _get_robot_position(self, state):
        """Extract robot position from state vector"""
        # Assuming position is in the first 2-3 elements of state
        # Adjust based on your state representation
        return state[:2]  # x, y position

    def _get_robot_velocity(self, state):
        """Extract robot velocity from state vector"""
        # Assuming velocity follows position in state vector
        start_idx = len(self._get_robot_position(state))
        return state[start_idx:start_idx+2]  # vx, vy

    def _get_imu_orientation(self, state):
        """Extract IMU orientation from state vector"""
        # This would typically be after joint states
        # Adjust indices based on your state representation
        # Example: after 12 joint positions + 12 joint velocities
        start_idx = 24  # Adjust based on your state layout
        return state[start_idx:start_idx+3]  # x, y, z orientation components

    def _get_imu_angular_velocity(self, state):
        """Extract IMU angular velocity from state vector"""
        start_idx = 27  # After orientation
        return state[start_idx:start_idx+3]  # wx, wy, wz

    def _get_joint_positions(self, state):
        """Extract joint positions from state vector"""
        # Adjust based on your state representation
        return state[0:12]  # First 12 elements for joint positions

    def _get_joint_velocities(self, state):
        """Extract joint velocities from state vector"""
        # After joint positions
        return state[12:24]  # Next 12 elements

    def _get_joint_torques(self, action):
        """Extract joint torques from action vector"""
        # For torque control, action is torques
        # For position control, torques would be computed
        return action

    def _compute_progress_reward(self, robot_pos, info):
        """Reward for moving toward the goal"""
        if 'goal_position' in info:
            goal_pos = info['goal_position']
            current_dist = np.linalg.norm(robot_pos - goal_pos)
            previous_dist = info.get('previous_distance', float('inf'))

            # Reward for reducing distance to goal
            progress = previous_dist - current_dist
            return max(0, progress)  # Only positive progress is rewarded
        return 0.0

    def _compute_balance_reward(self, orientation, angular_vel):
        """Reward for maintaining balance"""
        # Reward for low angular velocity (stability)
        angular_vel_magnitude = np.linalg.norm(angular_vel)
        stability_reward = np.exp(-angular_vel_magnitude)  # Higher reward for lower angular velocity

        return stability_reward

    def _compute_smoothness_penalty(self, joint_velocities):
        """Penalty for jerky movements"""
        # Penalize high joint velocities
        velocity_magnitude = np.linalg.norm(joint_velocities)
        # Use squared magnitude for stronger penalty on large velocities
        return velocity_magnitude ** 2

    def _compute_energy_penalty(self, joint_torques, joint_velocities):
        """Penalty for high energy consumption"""
        # Energy is roughly proportional to torque * velocity
        power = np.abs(joint_torques * joint_velocities)
        total_energy = np.sum(power)
        return total_energy

    def _compute_upright_reward(self, orientation):
        """Reward for staying upright"""
        # Orientation should align with gravity (z-axis)
        # Assuming orientation is a 3D vector representing body orientation
        upright_alignment = orientation[2]  # z-component should be close to 1
        # Reward between 0 and 1
        return max(0, upright_alignment)

    def _compute_footstep_reward(self, state, action, info):
        """Reward for proper footstep placement"""
        # This would be specific to walking controllers
        # For now, return 0
        return 0.0

    def _compute_foot_contact_reward(self, state):
        """Reward for proper foot contact timing"""
        # Check if feet are contacting ground at appropriate times
        # This would require contact sensors
        contact_sensors = state[30:36]  # Example indices for contact sensors
        left_foot_contact = contact_sensors[0]  # Example
        right_foot_contact = contact_sensors[1]  # Example

        # Reward for appropriate contact patterns during walking
        # This is a simplified example
        return 0.0

# Example usage in training loop
class HumanoidRLTrainer:
    def __init__(self, agent, env, reward_function):
        self.agent = agent
        self.env = env
        self.reward_function = reward_function
        self.episode_rewards = []
        self.episode_lengths = []

    def train_episode(self, max_steps=1000):
        """Train for one episode"""
        state = self.env.reset()
        episode_reward = 0
        step_count = 0

        while step_count < max_steps:
            # Select action
            action = self.agent.select_action(state)

            # Take action in environment
            next_state, env_reward, done, truncated, info = self.env.step(action)

            # Compute custom reward using our reward function
            custom_reward = self.reward_function.compute_reward(
                state, action, next_state, info
            )

            # Store experience in replay buffer
            self.agent.memory.push(
                state, action, next_state, custom_reward, done or truncated
            )

            # Train agent
            self.agent.train()

            state = next_state
            episode_reward += custom_reward
            step_count += 1

            if done or truncated:
                break

        self.episode_rewards.append(episode_reward)
        self.episode_lengths.append(step_count)

        return episode_reward, step_count
```

## Sim-to-Real Transfer Considerations

### Addressing Reality Gap

The sim-to-real transfer is crucial for deploying RL policies from simulation to real robots:

```python
# sim_to_real_transfer.py - Techniques for sim-to-real transfer
import numpy as np
import torch
import random

class DomainRandomization:
    """Apply domain randomization to improve sim-to-real transfer"""

    def __init__(self, env):
        self.env = env
        self.randomization_params = {
            'mass_range': [0.8, 1.2],      # 80% to 120% of original mass
            'friction_range': [0.5, 1.5],  # Friction coefficient range
            'actuator_range': [0.9, 1.1],  # Actuator strength
            'sensor_noise_range': [0.0, 0.1],  # Sensor noise level
            'latency_range': [0, 0.02],    # Control latency in seconds
        }

    def randomize_environment(self):
        """Randomize environment parameters"""
        # Randomize robot mass
        mass_multiplier = random.uniform(
            self.randomization_params['mass_range'][0],
            self.randomization_params['mass_range'][1]
        )
        self._set_robot_mass(mass_multiplier)

        # Randomize friction coefficients
        friction = random.uniform(
            self.randomization_params['friction_range'][0],
            self.randomization_params['friction_range'][1]
        )
        self._set_friction(friction)

        # Randomize actuator properties
        actuator_scale = random.uniform(
            self.randomization_params['actuator_range'][0],
            self.randomization_params['actuator_range'][1]
        )
        self._set_actuator_scale(actuator_scale)

        # Add sensor noise
        sensor_noise = random.uniform(
            self.randomization_params['sensor_noise_range'][0],
            self.randomization_params['sensor_noise_range'][1]
        )
        self._set_sensor_noise(sensor_noise)

        # Add control latency
        latency = random.uniform(
            self.randomization_params['latency_range'][0],
            self.randomization_params['latency_range'][1]
        )
        self._set_control_latency(latency)

    def _set_robot_mass(self, multiplier):
        """Set robot mass with multiplier"""
        # Implementation depends on your simulation environment
        pass

    def _set_friction(self, friction):
        """Set friction coefficients"""
        # Implementation depends on your simulation environment
        pass

    def _set_actuator_scale(self, scale):
        """Set actuator scaling"""
        # Implementation depends on your simulation environment
        pass

    def _set_sensor_noise(self, noise_level):
        """Set sensor noise level"""
        # Implementation depends on your simulation environment
        pass

    def _set_control_latency(self, latency):
        """Set control latency"""
        # Implementation depends on your simulation environment
        pass

class CurriculumLearning:
    """Implement curriculum learning for humanoid control"""

    def __init__(self, tasks):
        self.tasks = tasks  # List of tasks from simple to complex
        self.current_task_idx = 0
        self.performance_threshold = 0.8  # Threshold to advance to next task

    def evaluate_performance(self, agent, task_idx, num_episodes=10):
        """Evaluate agent performance on a specific task"""
        total_reward = 0
        success_count = 0

        for _ in range(num_episodes):
            env = self.tasks[task_idx]
            state = env.reset()
            episode_reward = 0
            done = False

            while not done:
                action = agent.select_action(state, add_noise=False)
                state, reward, done, truncated, info = env.step(action)
                episode_reward += reward

                if info.get('success', False):
                    success_count += 1

            total_reward += episode_reward

        avg_reward = total_reward / num_episodes
        success_rate = success_count / num_episodes
        return avg_reward, success_rate

    def advance_curriculum(self, agent):
        """Advance to the next task if performance is sufficient"""
        if self.current_task_idx < len(self.tasks) - 1:
            avg_reward, success_rate = self.evaluate_performance(
                agent, self.current_task_idx
            )

            if success_rate >= self.performance_threshold:
                self.current_task_idx += 1
                print(f"Advanced to task {self.current_task_idx}: {self.tasks[self.current_task_idx]}")
                return True

        return False

class RobustControl:
    """Implement robust control techniques for sim-to-real"""

    def __init__(self, nominal_policy, uncertainty_bounds):
        self.nominal_policy = nominal_policy
        self.uncertainty_bounds = uncertainty_bounds
        self.adaptive_gain = 1.0

    def adapt_to_real_world(self, state, action, error_feedback):
        """Adapt policy based on real-world error feedback"""
        # Adjust control action based on observed errors
        adaptive_term = self.adaptive_gain * error_feedback
        adapted_action = action + adaptive_term

        # Ensure action remains within bounds
        max_action = self.nominal_policy.max_action
        adapted_action = np.clip(adapted_action, -max_action, max_action)

        return adapted_action
```

## Training and Deployment Pipeline

### Complete Training Pipeline

```python
# training_pipeline.py - Complete RL training pipeline for humanoid robots
import os
import json
import numpy as np
import torch
from datetime import datetime

class HumanoidRLTrainingPipeline:
    def __init__(self, agent, env, reward_function, config):
        self.agent = agent
        self.env = env
        self.reward_function = reward_function
        self.config = config

        # Training metrics
        self.training_metrics = {
            'episode_rewards': [],
            'episode_lengths': [],
            'episode_times': [],
            'training_losses': [],
            'eval_rewards': []
        }

        # Checkpoint directory
        self.checkpoint_dir = config.get('checkpoint_dir', './checkpoints')
        os.makedirs(self.checkpoint_dir, exist_ok=True)

        # Domain randomization
        if config.get('use_domain_randomization', False):
            self.domain_randomizer = DomainRandomization(env)
        else:
            self.domain_randomizer = None

    def train(self, total_timesteps):
        """Main training loop"""
        print(f"Starting training for {total_timesteps} timesteps")

        timestep = 0
        episode_num = 0
        episode_reward = 0
        episode_timesteps = 0
        state = self.env.reset()

        while timestep < total_timesteps:
            # Randomize environment if using domain randomization
            if self.domain_randomizer:
                self.domain_randomizer.randomize_environment()

            # Select action
            action = self.agent.select_action(state)

            # Execute action in environment
            next_state, env_reward, done, truncated, info = self.env.step(action)

            # Compute custom reward
            reward = self.reward_function.compute_reward(
                state, action, next_state, info
            )

            # Store experience
            done_bool = float(done or truncated)
            self.agent.memory.push(state, action, next_state, reward, done_bool)

            # Train agent
            if timestep >= self.config['start_timesteps']:
                self.agent.train(batch_size=self.config['batch_size'])

            # Update counters
            state = next_state
            episode_reward += reward
            episode_timesteps += 1
            timestep += 1

            # Check for episode end
            if done or truncated or episode_timesteps >= self.config['max_episode_steps']:
                # Log episode metrics
                self.training_metrics['episode_rewards'].append(episode_reward)
                self.training_metrics['episode_lengths'].append(episode_timesteps)

                print(f"Total T: {timestep} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward:.3f}")

                # Reset environment
                state, done = self.env.reset(), False
                episode_reward = 0
                episode_timesteps = 0
                episode_num += 1

                # Evaluate periodically
                if episode_num % self.config['eval_freq'] == 0:
                    eval_reward = self.evaluate()
                    self.training_metrics['eval_rewards'].append(eval_reward)
                    print(f"Evaluation Reward: {eval_reward:.3f}")

                    # Save checkpoint
                    checkpoint_path = os.path.join(
                        self.checkpoint_dir,
                        f"checkpoint_{episode_num}.pt"
                    )
                    self.agent.save(checkpoint_path)

                    # Save training metrics
                    metrics_path = os.path.join(
                        self.checkpoint_dir,
                        f"metrics_{episode_num}.json"
                    )
                    with open(metrics_path, 'w') as f:
                        json.dump(self.training_metrics, f)

    def evaluate(self, eval_episodes=10):
        """Evaluate the trained policy"""
        total_reward = 0
        for _ in range(eval_episodes):
            state = self.env.reset()
            episode_reward = 0
            done = False

            while not done:
                action = self.agent.select_action(state, add_noise=False)
                state, reward, done, truncated, info = self.env.step(action)
                episode_reward += reward

                if done or truncated:
                    break

            total_reward += episode_reward

        avg_reward = total_reward / eval_episodes
        return avg_reward

    def save_final_model(self, filename=None):
        """Save the final trained model"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"humanoid_rl_model_{timestamp}.pt"

        self.agent.save(os.path.join(self.checkpoint_dir, filename))
        print(f"Model saved to {filename}")

def main_training():
    """Main training function"""
    # Configuration
    config = {
        'env_name': 'HumanoidRobot-v1',
        'seed': 0,
        'batch_size': 100,
        'max_episode_steps': 1000,
        'start_timesteps': 10000,
        'eval_freq': 5000,
        'max_timesteps': 1000000,
        'exploration_noise': 0.1,
        'checkpoint_dir': './checkpoints',
        'use_domain_randomization': True
    }

    # Initialize environment (this would be your actual humanoid env)
    env = HumanoidRobotEnv()  # Replace with your environment

    # Initialize agent
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    max_action = float(env.action_space.high[0])

    agent = TD3Agent(state_dim, action_dim, max_action)

    # Initialize reward function
    reward_function = HumanoidRewardFunction()

    # Initialize training pipeline
    trainer = HumanoidRLTrainingPipeline(agent, env, reward_function, config)

    # Start training
    trainer.train(config['max_timesteps'])

    # Save final model
    trainer.save_final_model()

if __name__ == "__main__":
    main_training()
```

## Practical Exercise: Train a Humanoid Walking Policy

Create a complete training example:

1. **Set up the training environment**:

```python
# train_humanoid_walker.py - Complete training example
import gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from stable_baselines3 import TD3
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
import os

def create_humanoid_walking_env():
    """Create a humanoid walking environment"""
    # This would typically use a physics simulator like PyBullet, Mujoco, or Isaac Gym
    # For this example, we'll use a placeholder
    return HumanoidRobotEnv()  # Your custom environment

def train_humanoid_walker():
    """Train a humanoid robot to walk"""
    # Create environment
    env = create_humanoid_walking_env()

    # Set up evaluation callback
    eval_callback = EvalCallback(
        env,
        callback_on_new_best=None,
        n_eval_episodes=5,
        best_model_save_path="./logs/humanoid_walker/",
        log_path="./logs/humanoid_walker/",
        eval_freq=10000,
        deterministic=True,
        render=False
    )

    # Create TD3 agent
    model = TD3(
        "MlpPolicy",
        env,
        learning_rate=1e-3,
        buffer_size=100000,
        learning_starts=1000,
        batch_size=100,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        action_noise=None,
        replay_buffer_class=None,
        replay_buffer_kwargs=None,
        optimize_memory_usage=False,
        policy_kwargs=None,
        verbose=1,
        device="auto",
        tensorboard_log="./tb_logs/humanoid_walker/",
        policy_kwargs=dict(net_arch=[256, 256])
    )

    # Train the model
    model.learn(
        total_timesteps=1000000,
        callback=eval_callback,
        log_interval=4
    )

    # Save the model
    model.save("humanoid_walker_td3")
    print("Model saved!")

# Run training
if __name__ == "__main__":
    train_humanoid_walker()
```

2. **Deploy the trained policy**:

```python
# deploy_policy.py - Deploy trained policy to robot
import torch
import numpy as np

class PolicyDeployer:
    def __init__(self, model_path):
        # Load trained model
        self.model = torch.load(model_path)
        self.model.eval()  # Set to evaluation mode

    def get_action(self, observation):
        """Get action from trained policy"""
        # Convert observation to tensor
        obs_tensor = torch.FloatTensor(observation).unsqueeze(0)

        # Get action from policy
        with torch.no_grad():
            action = self.model(obs_tensor)

        # Convert to numpy and return
        return action.numpy().flatten()

def deploy_to_robot():
    """Deploy policy to real humanoid robot"""
    # Initialize policy deployer
    deployer = PolicyDeployer("humanoid_walker_td3.pt")

    # Connect to robot
    robot = connect_to_humanoid_robot()  # Your robot interface

    # Run control loop
    while True:
        # Get current robot state
        state = robot.get_current_state()

        # Get action from policy
        action = deployer.get_action(state)

        # Apply action to robot
        robot.apply_action(action)

        # Sleep for control frequency
        time.sleep(0.02)  # 50 Hz control

if __name__ == "__main__":
    deploy_to_robot()
```

## Troubleshooting Common RL Issues

### Training Stability Issues
- **Exploding gradients**: Use gradient clipping and proper weight initialization
- **Learning instability**: Reduce learning rate and use target networks
- **Poor exploration**: Adjust noise parameters and reward shaping

### Sim-to-Real Transfer Issues
- **Reality gap**: Apply domain randomization and system identification
- **Sensor differences**: Add noise and delay to simulation sensors
- **Actuator differences**: Model actuator dynamics in simulation

### Performance Optimization
- **Slow training**: Use parallel environments and optimized RL libraries
- **Memory issues**: Implement efficient replay buffers and gradient computation
- **Real-time constraints**: Optimize policy inference and control frequency

## Summary

In this chapter, we've explored implementing reinforcement learning for humanoid robot control. We covered RL fundamentals, deep RL algorithms like DDPG and TD3, reward function design for humanoid behaviors, sim-to-real transfer techniques, and a complete training pipeline. RL provides a powerful approach for learning complex humanoid behaviors that are difficult to engineer with traditional control methods.

## Next Steps

- Implement and train RL policies for specific humanoid tasks
- Experiment with different network architectures and hyperparameters
- Apply domain randomization for better sim-to-real transfer
- Test trained policies on real humanoid robots
- Explore advanced RL techniques like hierarchical RL and multi-task learning