---
title: "Sim-to-Real concepts: Transfer learning, domain adaptation"
description: "Understanding and implementing Sim-to-Real transfer for humanoid robotics applications"
learning_objectives:
  - "Understand the reality gap in robotics simulation"
  - "Implement domain randomization techniques for better transfer"
  - "Apply transfer learning methods for Sim-to-Real applications"
  - "Evaluate and validate Sim-to-Real performance"
---

# Sim-to-Real concepts: Transfer learning, domain adaptation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the reality gap in robotics simulation
- Implement domain randomization techniques for better transfer
- Apply transfer learning methods for Sim-to-Real applications
- Evaluate and validate Sim-to-Real performance

## Introduction

The Sim-to-Real (Simulation to Reality) transfer is one of the most challenging aspects of robotics development. While simulation provides a safe, cost-effective, and controllable environment for developing and testing robot algorithms, there's always a gap between simulated and real-world behavior. This gap, known as the "reality gap," arises from differences in physics models, sensor noise, actuator dynamics, environmental conditions, and other factors. For humanoid robots, this gap is particularly challenging due to the complexity of their multi-degree-of-freedom systems and the importance of precise control for balance and locomotion. This chapter explores techniques to bridge this gap and successfully transfer policies learned in simulation to real humanoid robots.

## Understanding the Reality Gap

### Sources of the Reality Gap

The reality gap in humanoid robotics stems from several key sources:

1. **Dynamics Mismatch**: Differences in friction, contact models, and inertial properties
2. **Sensor Noise**: Simulated sensors often lack the noise and imperfections of real sensors
3. **Actuator Imperfections**: Real actuators have delays, backlash, and limited precision
4. **Environmental Factors**: Lighting, surface conditions, and external disturbances
5. **Modeling Errors**: Imperfect CAD models and unmodeled dynamics
6. **Control Frequency**: Differences in control loop frequencies between simulation and reality

### Quantifying the Reality Gap

```python
# reality_gap_analysis.py - Analyzing and quantifying the reality gap
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import seaborn as sns

class RealityGapAnalyzer:
    """Analyze and quantify differences between simulation and reality"""

    def __init__(self):
        self.simulation_data = []
        self.real_world_data = []
        self.gap_metrics = {}

    def collect_simulation_data(self, data):
        """Collect simulation data for analysis"""
        self.simulation_data.append(data)

    def collect_real_world_data(self, data):
        """Collect real-world data for comparison"""
        self.real_world_data.append(data)

    def calculate_distribution_distances(self):
        """Calculate statistical distances between sim and real distributions"""
        if len(self.simulation_data) == 0 or len(self.real_world_data) == 0:
            return {}

        # Convert to numpy arrays
        sim_array = np.array(self.simulation_data)
        real_array = np.array(self.real_world_data)

        # Calculate various gap metrics
        metrics = {}

        # 1. Mean Absolute Difference
        mad = np.mean(np.abs(sim_array - real_array))
        metrics['mean_absolute_difference'] = mad

        # 2. Root Mean Square Error
        rmse = np.sqrt(np.mean((sim_array - real_array)**2))
        metrics['rmse'] = rmse

        # 3. Correlation Coefficient
        correlation = np.corrcoef(sim_array.flatten(), real_array.flatten())[0, 1]
        metrics['correlation'] = correlation

        # 4. Kolmogorov-Smirnov statistic (for comparing distributions)
        from scipy.stats import ks_2samp
        if sim_array.size > 0 and real_array.size > 0:
            # Flatten for KS test
            sim_flat = sim_array.flatten()
            real_flat = real_array.flatten()
            ks_stat, ks_p_value = ks_2samp(sim_flat, real_flat)
            metrics['ks_statistic'] = ks_stat
            metrics['ks_p_value'] = ks_p_value

        # 5. Maximum Mean Discrepancy (simplified version)
        metrics['mmd_approx'] = self._approximate_mmd(sim_array, real_array)

        return metrics

    def _approximate_mmd(self, sim_data, real_data):
        """Approximate Maximum Mean Discrepancy between datasets"""
        # Simplified MMD calculation using RBF kernel
        # In practice, you'd use a more sophisticated approach
        sim_flat = sim_data.flatten()[:1000]  # Limit for computational efficiency
        real_flat = real_data.flatten()[:1000]

        # Compute pairwise distances
        sim_to_sim = np.mean((sim_flat[:, None] - sim_flat[None, :])**2)
        real_to_real = np.mean((real_flat[:, None] - real_flat[None, :])**2)
        sim_to_real = np.mean((sim_flat[:, None] - real_flat[None, :])**2)

        # MMD approximation
        mmd = sim_to_sim + real_to_real - 2 * sim_to_real
        return np.sqrt(max(0, mmd))

    def plot_comparison(self):
        """Plot comparison between simulation and real data"""
        if len(self.simulation_data) == 0 or len(self.real_world_data) == 0:
            print("No data to plot")
            return

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))

        sim_array = np.array(self.simulation_data)
        real_array = np.array(self.real_world_data)

        # Plot 1: Histogram comparison
        axes[0, 0].hist(sim_array.flatten(), bins=50, alpha=0.5, label='Simulation', density=True)
        axes[0, 0].hist(real_array.flatten(), bins=50, alpha=0.5, label='Real World', density=True)
        axes[0, 0].set_title('Distribution Comparison')
        axes[0, 0].legend()

        # Plot 2: Scatter plot
        min_len = min(len(sim_array.flatten()), len(real_array.flatten()))
        axes[0, 1].scatter(sim_array.flatten()[:min_len], real_array.flatten()[:min_len], alpha=0.5)
        axes[0, 1].plot([sim_array.min(), sim_array.max()], [sim_array.min(), sim_array.max()], 'r--', lw=2)
        axes[0, 1].set_xlabel('Simulation Values')
        axes[0, 1].set_ylabel('Real World Values')
        axes[0, 1].set_title('Simulation vs Real World')

        # Plot 3: Time series comparison (if applicable)
        if len(sim_array) > 1 and len(real_array) > 1:
            axes[1, 0].plot(sim_array[:min(len(sim_array), len(real_array))], label='Simulation', alpha=0.7)
            axes[1, 0].plot(real_array[:min(len(sim_array), len(real_array))], label='Real World', alpha=0.7)
            axes[1, 0].set_title('Time Series Comparison')
            axes[1, 0].legend()

        # Plot 4: Error distribution
        error = sim_array.flatten()[:min_len] - real_array.flatten()[:min_len]
        axes[1, 1].hist(error, bins=50, alpha=0.7)
        axes[1, 1].set_title('Error Distribution')
        axes[1, 1].set_xlabel('Simulation - Real World')

        plt.tight_layout()
        plt.show()

    def generate_report(self):
        """Generate a comprehensive reality gap report"""
        metrics = self.calculate_distribution_distances()

        report = f"""
Reality Gap Analysis Report
===========================

Data Statistics:
- Simulation samples: {len(self.simulation_data)}
- Real world samples: {len(self.real_world_data)}

Gap Metrics:
- Mean Absolute Difference: {metrics.get('mean_absolute_difference', 'N/A'): .4f}
- RMSE: {metrics.get('rmse', 'N/A'): .4f}
- Correlation: {metrics.get('correlation', 'N/A'): .4f}
- KS Statistic: {metrics.get('ks_statistic', 'N/A'): .4f}
- MMD Approximation: {metrics.get('mmd_approx', 'N/A'): .4f}

Interpretation:
- Lower values for MAD, RMSE, and KS statistic indicate better alignment
- Higher correlation indicates better relationship between sim and real
- Lower MMD indicates more similar distributions
        """

        print(report)
        return report

# Example usage
def analyze_robot_joint_states(sim_joint_positions, real_joint_positions):
    """Analyze joint state differences between simulation and reality"""
    analyzer = RealityGapAnalyzer()

    for sim_pos, real_pos in zip(sim_joint_positions, real_joint_positions):
        analyzer.collect_simulation_data(sim_pos)
        analyzer.collect_real_world_data(real_pos)

    report = analyzer.generate_report()
    analyzer.plot_comparison()

    return analyzer
```

## Domain Randomization Techniques

### Physics Parameter Randomization

Domain randomization involves varying simulation parameters to make the policy robust to parameter variations:

```python
# domain_randomization.py - Domain randomization for Sim-to-Real transfer
import numpy as np
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

@dataclass
class DomainRandomizationParams:
    """Parameters for domain randomization"""
    # Robot parameters
    mass_range: Tuple[float, float] = (0.8, 1.2)  # Mass multiplier range
    friction_range: Tuple[float, float] = (0.5, 1.5)  # Friction coefficient range
    damping_range: Tuple[float, float] = (0.8, 1.2)  # Damping coefficient range
    stiffness_range: Tuple[float, float] = (0.8, 1.2)  # Joint stiffness range

    # Actuator parameters
    actuator_strength_range: Tuple[float, float] = (0.9, 1.1)  # Actuator strength multiplier
    actuator_delay_range: Tuple[float, float] = (0.0, 0.02)  # Control delay in seconds
    actuator_noise_range: Tuple[float, float] = (0.0, 0.05)  # Actuator noise level

    # Sensor parameters
    sensor_noise_range: Tuple[float, float] = (0.0, 0.1)  # Sensor noise level
    sensor_bias_range: Tuple[float, float] = (-0.05, 0.05)  # Sensor bias range
    sensor_delay_range: Tuple[float, float] = (0.0, 0.01)  # Sensor delay in seconds

    # Environmental parameters
    gravity_range: Tuple[float, float] = (-9.91, -9.71)  # Gravity range (m/s²)
    ground_friction_range: Tuple[float, float] = (0.8, 1.2)  # Ground friction range
    wind_force_range: Tuple[float, float] = (-0.5, 0.5)  # Wind force range (N)

class PhysicsRandomizer:
    """Randomize physics parameters to improve sim-to-real transfer"""

    def __init__(self, randomization_params: DomainRandomizationParams):
        self.params = randomization_params
        self.current_values = {}

    def randomize_robot_properties(self, robot_interface):
        """Randomize robot physical properties"""
        # Randomize link masses
        for link_name in robot_interface.get_link_names():
            original_mass = robot_interface.get_link_mass(link_name)
            mass_multiplier = random.uniform(
                self.params.mass_range[0],
                self.params.mass_range[1]
            )
            new_mass = original_mass * mass_multiplier
            robot_interface.set_link_mass(link_name, new_mass)

        # Randomize joint friction and damping
        for joint_name in robot_interface.get_joint_names():
            # Friction
            original_friction = robot_interface.get_joint_friction(joint_name)
            friction_multiplier = random.uniform(
                self.params.friction_range[0],
                self.params.friction_range[1]
            )
            new_friction = original_friction * friction_multiplier
            robot_interface.set_joint_friction(joint_name, new_friction)

            # Damping
            original_damping = robot_interface.get_joint_damping(joint_name)
            damping_multiplier = random.uniform(
                self.params.damping_range[0],
                self.params.damping_range[1]
            )
            new_damping = original_damping * damping_multiplier
            robot_interface.set_joint_damping(joint_name, new_damping)

            # Stiffness
            original_stiffness = robot_interface.get_joint_stiffness(joint_name)
            stiffness_multiplier = random.uniform(
                self.params.stiffness_range[0],
                self.params.stiffness_range[1]
            )
            new_stiffness = original_stiffness * stiffness_multiplier
            robot_interface.set_joint_stiffness(joint_name, new_stiffness)

    def randomize_actuators(self, robot_interface):
        """Randomize actuator properties"""
        # Store original actuator parameters
        if not hasattr(self, 'original_actuators'):
            self.original_actuators = {}
            for joint_name in robot_interface.get_joint_names():
                self.original_actuators[joint_name] = {
                    'strength': robot_interface.get_actuator_strength(joint_name),
                    'delay': robot_interface.get_actuator_delay(joint_name),
                    'noise': robot_interface.get_actuator_noise(joint_name)
                }

        # Apply randomization
        for joint_name in robot_interface.get_joint_names():
            original = self.original_actuators[joint_name]

            # Strength
            strength_multiplier = random.uniform(
                self.params.actuator_strength_range[0],
                self.params.actuator_strength_range[1]
            )
            new_strength = original['strength'] * strength_multiplier
            robot_interface.set_actuator_strength(joint_name, new_strength)

            # Delay
            delay = random.uniform(
                self.params.actuator_delay_range[0],
                self.params.actuator_delay_range[1]
            )
            robot_interface.set_actuator_delay(joint_name, delay)

            # Noise
            noise_level = random.uniform(
                self.params.actuator_noise_range[0],
                self.params.actuator_noise_range[1]
            )
            robot_interface.set_actuator_noise(joint_name, noise_level)

    def randomize_sensors(self, robot_interface):
        """Randomize sensor properties"""
        # IMU randomization
        imu_noise = random.uniform(
            self.params.sensor_noise_range[0],
            self.params.sensor_noise_range[1]
        )
        imu_bias = random.uniform(
            self.params.sensor_bias_range[0],
            self.params.sensor_bias_range[1]
        )
        imu_delay = random.uniform(
            self.params.sensor_delay_range[0],
            self.params.sensor_delay_range[1]
        )

        robot_interface.set_imu_noise(imu_noise)
        robot_interface.set_imu_bias(imu_bias)
        robot_interface.set_imu_delay(imu_delay)

        # Joint encoder randomization
        for joint_name in robot_interface.get_joint_names():
            encoder_noise = random.uniform(
                self.params.sensor_noise_range[0],
                self.params.sensor_noise_range[1]
            )
            encoder_bias = random.uniform(
                self.params.sensor_bias_range[0],
                self.params.sensor_bias_range[1]
            )
            robot_interface.set_joint_encoder_noise(joint_name, encoder_noise)
            robot_interface.set_joint_encoder_bias(joint_name, encoder_bias)

    def randomize_environment(self, sim_interface):
        """Randomize environmental parameters"""
        # Gravity
        gravity_z = random.uniform(
            self.params.gravity_range[0],
            self.params.gravity_range[1]
        )
        sim_interface.set_gravity((0, 0, gravity_z))

        # Ground friction
        ground_friction = random.uniform(
            self.params.ground_friction_range[0],
            self.params.ground_friction_range[1]
        )
        sim_interface.set_ground_friction(ground_friction)

        # Wind force
        wind_force = random.uniform(
            self.params.wind_force_range[0],
            self.params.wind_force_range[1]
        )
        sim_interface.set_wind_force((wind_force, 0, 0))

    def randomize_all(self, robot_interface, sim_interface):
        """Apply all randomizations"""
        self.randomize_robot_properties(robot_interface)
        self.randomize_actuators(robot_interface)
        self.randomize_sensors(robot_interface)
        self.randomize_environment(sim_interface)

        # Store current randomization values for reference
        self.current_values = {
            'mass_multiplier': random.uniform(self.params.mass_range[0], self.params.mass_range[1]),
            'friction_multiplier': random.uniform(self.params.friction_range[0], self.params.friction_range[1]),
            'gravity_z': random.uniform(self.params.gravity_range[0], self.params.gravity_range[1])
        }

class DomainRandomizationScheduler:
    """Schedule domain randomization during training"""

    def __init__(self, randomizer: PhysicsRandomizer, schedule_type: str = "uniform"):
        self.randomizer = randomizer
        self.schedule_type = schedule_type  # 'uniform', 'curriculum', 'adaptive'
        self.training_step = 0

    def should_randomize(self, current_step: int) -> bool:
        """Determine if randomization should be applied at current step"""
        if self.schedule_type == "uniform":
            # Randomize every episode
            return True
        elif self.schedule_type == "curriculum":
            # Start with low randomization, increase over time
            randomization_probability = min(0.1 + current_step * 0.00001, 1.0)
            return random.random() < randomization_probability
        elif self.schedule_type == "adaptive":
            # Adjust based on training performance
            # This would require monitoring training metrics
            return True
        else:
            return True

    def apply_randomization(self, robot_interface, sim_interface, current_step: int):
        """Apply randomization based on current schedule"""
        if self.should_randomize(current_step):
            self.randomizer.randomize_all(robot_interface, sim_interface)
            self.training_step = current_step
            return True
        return False

# Example usage in training loop
def training_with_domain_randomization():
    """Example training loop with domain randomization"""
    # Initialize randomizer
    params = DomainRandomizationParams()
    randomizer = PhysicsRandomizer(params)
    scheduler = DomainRandomizationScheduler(randomizer, schedule_type="curriculum")

    # Training loop
    for episode in range(10000):
        # Apply domain randomization
        if scheduler.should_randomize(episode):
            randomizer.randomize_all(robot_interface, sim_interface)

        # Run episode with randomized environment
        # ... training code ...
```

### Visual Domain Randomization

Visual domain randomization addresses differences in appearance between simulation and reality:

```python
# visual_domain_randomization.py - Visual domain randomization techniques
import numpy as np
import cv2
import random
from PIL import Image, ImageEnhance, ImageFilter
import torch
import torchvision.transforms as transforms

class VisualRandomizer:
    """Apply visual domain randomization to improve sim-to-real transfer"""

    def __init__(self):
        self.visual_params = {
            'brightness_range': (0.5, 1.5),
            'contrast_range': (0.5, 1.5),
            'saturation_range': (0.5, 1.5),
            'hue_range': (-0.1, 0.1),
            'blur_range': (0, 2),
            'noise_range': (0, 0.1),
            'occlusion_probability': 0.3,
            'texture_randomization': True,
            'lighting_randomization': True
        }

    def randomize_image(self, image):
        """Apply visual randomization to an image"""
        # Convert to PIL Image if it's a numpy array
        if isinstance(image, np.ndarray):
            if image.dtype == np.uint8:
                pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            else:
                # Assume float32 in [0, 1] range
                pil_image = Image.fromarray((image * 255).astype(np.uint8))
        else:
            pil_image = image

        # Apply brightness adjustment
        brightness_factor = random.uniform(
            self.visual_params['brightness_range'][0],
            self.visual_params['brightness_range'][1]
        )
        enhancer = ImageEnhance.Brightness(pil_image)
        pil_image = enhancer.enhance(brightness_factor)

        # Apply contrast adjustment
        contrast_factor = random.uniform(
            self.visual_params['contrast_range'][0],
            self.visual_params['contrast_range'][1]
        )
        enhancer = ImageEnhance.Contrast(pil_image)
        pil_image = enhancer.enhance(contrast_factor)

        # Apply saturation adjustment
        saturation_factor = random.uniform(
            self.visual_params['saturation_range'][0],
            self.visual_params['saturation_range'][1]
        )
        enhancer = ImageEnhance.Color(pil_image)
        pil_image = enhancer.enhance(saturation_factor)

        # Apply blur
        blur_radius = random.uniform(
            self.visual_params['blur_range'][0],
            self.visual_params['blur_range'][1]
        )
        pil_image = pil_image.filter(ImageFilter.GaussianBlur(radius=blur_radius))

        # Add noise
        noise_level = random.uniform(
            self.visual_params['noise_range'][0],
            self.visual_params['noise_range'][1]
        )
        if noise_level > 0:
            pil_image = self.add_noise(pil_image, noise_level)

        # Convert back to numpy array
        image_array = np.array(pil_image)
        if len(image_array.shape) == 3 and image_array.shape[2] == 3:
            image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)

        return image_array

    def add_noise(self, image, noise_level):
        """Add random noise to image"""
        # Convert to numpy array
        img_array = np.array(image, dtype=np.float32) / 255.0

        # Generate noise
        noise = np.random.normal(0, noise_level, img_array.shape).astype(np.float32)

        # Add noise
        noisy_img = img_array + noise
        noisy_img = np.clip(noisy_img, 0, 1)

        # Convert back to PIL Image
        noisy_img = (noisy_img * 255).astype(np.uint8)
        return Image.fromarray(noisy_img)

    def randomize_lighting(self, sim_image, lighting_conditions=None):
        """Randomize lighting conditions in simulation"""
        if lighting_conditions is None:
            # Generate random lighting conditions
            lighting_conditions = {
                'intensity': random.uniform(0.5, 2.0),
                'direction': (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)),
                'color_temperature': random.uniform(3000, 8000)  # Kelvin
            }

        # Apply lighting randomization
        # This would typically be done in the simulation engine
        # For demonstration, we'll apply a simple intensity adjustment
        adjusted_image = sim_image * lighting_conditions['intensity']
        adjusted_image = np.clip(adjusted_image, 0, 1)

        return adjusted_image, lighting_conditions

    def randomize_textures(self, sim_interface):
        """Randomize textures in simulation"""
        # This would typically involve changing material properties in the simulation
        # For now, we'll just return a random texture assignment
        texture_options = [
            'wood', 'metal', 'concrete', 'tile', 'carpet', 'grass', 'asphalt'
        ]

        randomized_textures = {}
        for surface_name in sim_interface.get_surface_names():
            texture = random.choice(texture_options)
            sim_interface.set_surface_texture(surface_name, texture)
            randomized_textures[surface_name] = texture

        return randomized_textures

class AdversarialDomainRandomization:
    """Use adversarial techniques to improve domain randomization"""

    def __init__(self):
        # This would involve training a domain classifier to guide randomization
        # For now, we'll implement a simplified version
        self.domain_classifier = None
        self.randomization_optimizer = None

    def train_domain_classifier(self, sim_data, real_data):
        """Train a classifier to distinguish simulation from real data"""
        # This is a simplified example - in practice, you'd use a neural network
        # The idea is to train a classifier to distinguish sim vs real
        # Then adjust randomization to fool the classifier
        pass

    def adaptive_randomization(self, sim_interface, robot_interface, discriminator_performance):
        """Adapt randomization based on discriminator performance"""
        # If discriminator can easily tell sim from real, increase randomization
        # If discriminator struggles, decrease randomization
        if discriminator_performance > 0.7:  # Discriminator is doing well
            # Increase randomization range
            print("Increasing randomization - discriminator performing well")
        else:
            # Decrease randomization range
            print("Decreasing randomization - discriminator struggling")
```

## Transfer Learning Techniques

### Pre-training in Simulation

```python
# transfer_learning.py - Transfer learning for Sim-to-Real applications
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from typing import Optional, Dict, Any

class SimToRealTransferLearner:
    """Transfer learning framework for Sim-to-Real applications"""

    def __init__(self, source_policy, target_policy, transfer_method='fine_tuning'):
        self.source_policy = source_policy  # Policy trained in simulation
        self.target_policy = target_policy  # Policy for real robot
        self.transfer_method = transfer_method
        self.transfer_metrics = {}

    def initialize_target_from_source(self):
        """Initialize target policy with source policy weights"""
        # Copy weights from source to target
        source_dict = self.source_policy.state_dict()
        target_dict = self.target_policy.state_dict()

        # Only copy matching layers
        for name, param in source_dict.items():
            if name in target_dict and param.shape == target_dict[name].shape:
                target_dict[name].copy_(param)

        # Update target policy with copied weights
        self.target_policy.load_state_dict(target_dict)

    def fine_tune_policy(self, real_robot_data, learning_rate=1e-5, epochs=100):
        """Fine-tune policy with real robot data"""
        optimizer = optim.Adam(self.target_policy.parameters(), lr=learning_rate)
        criterion = nn.MSELoss()

        self.target_policy.train()
        for epoch in range(epochs):
            total_loss = 0
            for batch in real_robot_data:
                # Extract states, actions, rewards from real robot data
                states = batch['states']
                actions = batch['actions']

                # Get actions from target policy
                predicted_actions = self.target_policy(states)

                # Calculate loss
                loss = criterion(predicted_actions, actions)

                # Backpropagate
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                total_loss += loss.item()

            avg_loss = total_loss / len(real_robot_data)
            print(f"Epoch {epoch}, Average Loss: {avg_loss:.6f}")

    def domain_adversarial_training(self, sim_data, real_data, lambda_reg=0.1):
        """Use domain adversarial training for better transfer"""
        # This implements Domain-Adversarial Training of Neural Networks (DANN)
        # The idea is to train a feature extractor that produces domain-invariant features

        # Initialize domain classifier
        domain_classifier = nn.Sequential(
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 2),  # Binary classification: sim vs real
            nn.Softmax(dim=1)
        )

        # Optimizers
        feature_optimizer = optim.Adam(
            list(self.target_policy.encoder.parameters()) +
            list(domain_classifier.parameters()),
            lr=1e-4
        )
        domain_optimizer = optim.Adam(domain_classifier.parameters(), lr=1e-4)

        for epoch in range(100):
            # Train on simulation data (label as 0)
            sim_features = self.target_policy.encode_features(sim_data['states'])
            sim_domain_pred = domain_classifier(sim_features)
            sim_domain_loss = nn.CrossEntropyLoss()(sim_domain_pred,
                                                  torch.zeros(len(sim_data), dtype=torch.long))

            # Train on real data (label as 1)
            real_features = self.target_policy.encode_features(real_data['states'])
            real_domain_pred = domain_classifier(real_features)
            real_domain_loss = nn.CrossEntropyLoss()(real_domain_pred,
                                                   torch.ones(len(real_data), dtype=torch.long))

            # Combined domain loss (want to confuse the domain classifier)
            domain_loss = -(sim_domain_loss + real_domain_loss)  # Negative because we want to maximize confusion

            # Task loss (want to maintain good performance)
            sim_actions_pred = self.target_policy.predict_with_features(sim_features)
            task_loss = nn.MSELoss()(sim_actions_pred, sim_data['actions'])

            # Combined loss
            total_loss = task_loss + lambda_reg * domain_loss

            # Update parameters
            feature_optimizer.zero_grad()
            total_loss.backward(retain_graph=True)
            feature_optimizer.step()

            # Update domain classifier separately
            domain_optimizer.zero_grad()
            domain_loss.backward()
            domain_optimizer.step()

    def progressive_nets(self, sim_policy, new_layers_config):
        """Implement Progressive Networks for transfer learning"""
        # Progressive Nets allow for transfer while protecting important features
        # from the source task
        pass

    def evaluate_transfer_performance(self, real_env):
        """Evaluate how well the transferred policy performs"""
        self.target_policy.eval()
        total_reward = 0
        episodes = 10

        for episode in range(episodes):
            state = real_env.reset()
            episode_reward = 0
            done = False

            while not done:
                with torch.no_grad():
                    action = self.target_policy(torch.FloatTensor(state).unsqueeze(0))
                    action = action.numpy().flatten()

                state, reward, done, info = real_env.step(action)
                episode_reward += reward

            total_reward += episode_reward

        avg_reward = total_reward / episodes
        self.transfer_metrics['avg_reward'] = avg_reward
        self.transfer_metrics['success_rate'] = self._calculate_success_rate(real_env)

        return avg_reward

    def _calculate_success_rate(self, env):
        """Calculate success rate based on task-specific criteria"""
        # This would be specific to your task
        # For humanoid walking, success might be staying upright for a distance
        return 0.0  # Placeholder

class CurriculumTransfer:
    """Curriculum-based transfer learning"""

    def __init__(self, source_tasks, target_task):
        self.source_tasks = source_tasks  # List of simpler tasks
        self.target_task = target_task    # Complex target task
        self.current_task_idx = 0

    def should_advance_curriculum(self, performance):
        """Determine if we should advance to the next task"""
        threshold = 0.8  # Require 80% performance
        return performance >= threshold

    def transfer_progressive_tasks(self, real_robot):
        """Transfer progressively from simpler to complex tasks"""
        for task_idx, task in enumerate(self.source_tasks):
            print(f"Transferring from task {task_idx}: {task}")

            # Train on current task in simulation
            # Transfer to real robot
            # Evaluate performance

            # If performance is good, advance to next task
            # Otherwise, continue training on current task
```

## Domain Adaptation Techniques

### Unsupervised Domain Adaptation

```python
# domain_adaptation.py - Domain adaptation techniques for Sim-to-Real
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset
import numpy as np

class DomainAdaptationNetwork(nn.Module):
    """Neural network with domain adaptation capabilities"""

    def __init__(self, input_dim, hidden_dims, output_dim):
        super(DomainAdaptationNetwork, self).__init__()

        # Feature extractor
        layers = []
        prev_dim = input_dim
        for dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, dim),
                nn.ReLU(),
                nn.Dropout(0.2)
            ])
            prev_dim = dim

        self.feature_extractor = nn.Sequential(*layers)

        # Task-specific predictor
        self.task_predictor = nn.Linear(prev_dim, output_dim)

        # Domain classifier
        self.domain_classifier = nn.Sequential(
            nn.Linear(prev_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 2),  # Sim vs Real
            nn.Softmax(dim=1)
        )

    def forward(self, x, return_features=False):
        features = self.feature_extractor(x)

        if return_features:
            return features

        task_output = self.task_predictor(features)
        domain_output = self.domain_classifier(features)

        return task_output, domain_output

class DomainAdversarialAdapter:
    """Domain adversarial adapter for Sim-to-Real transfer"""

    def __init__(self, model: DomainAdaptationNetwork):
        self.model = model
        self.grl_lambda = 0.1  # Gradient reversal layer lambda

    def gradient_reversal(self, x):
        """Reverse gradients for domain adaptation"""
        return GradientReversal.apply(x, self.grl_lambda)

    def train_adaptation(self, sim_loader, real_loader, epochs=100):
        """Train with domain adaptation"""
        optimizer = torch.optim.Adam(self.model.parameters(), lr=1e-3)

        for epoch in range(epochs):
            for (sim_batch, real_batch) in zip(sim_loader, real_loader):
                # Sim data (domain label = 0)
                sim_states, sim_actions = sim_batch
                sim_features = self.model.feature_extractor(sim_states)

                # Real data (domain label = 1)
                real_states, real_actions = real_batch
                real_features = self.model.feature_extractor(real_states)

                # Task loss on simulation data
                sim_task_pred = self.model.task_predictor(sim_features)
                task_loss = F.mse_loss(sim_task_pred, sim_actions)

                # Domain classification loss (want to minimize)
                all_features = torch.cat([sim_features, real_features], dim=0)
                domain_preds = self.model.domain_classifier(all_features)

                sim_labels = torch.zeros(len(sim_features), dtype=torch.long)
                real_labels = torch.ones(len(real_features), dtype=torch.long)
                all_labels = torch.cat([sim_labels, real_labels])

                domain_loss = F.cross_entropy(domain_preds, all_labels)

                # Domain adversarial loss (want to maximize confusion)
                adversarial_loss = -domain_loss

                # Total loss
                total_loss = task_loss + 0.1 * adversarial_loss

                optimizer.zero_grad()
                total_loss.backward()
                optimizer.step()

class GradientReversal(torch.autograd.Function):
    """Gradient reversal layer"""

    @staticmethod
    def forward(ctx, input, lambda_val):
        ctx.lambda_val = lambda_val
        return input

    @staticmethod
    def backward(ctx, grad_output):
        lambda_val = ctx.lambda_val
        return lambda_val * grad_output.neg(), None

class SelfSupervisedAdapter:
    """Self-supervised learning for domain adaptation"""

    def __init__(self, base_network):
        self.base_network = base_network
        self.auxiliary_tasks = []

    def add_auxiliary_task(self, task_name, task_module):
        """Add auxiliary task for self-supervised learning"""
        self.auxiliary_tasks.append((task_name, task_module))

    def compute_auxiliary_losses(self, states, predictions):
        """Compute losses for auxiliary tasks"""
        losses = {}

        for task_name, task_module in self.auxiliary_tasks:
            if task_name == 'temporal_coherence':
                # Encourage temporal consistency
                loss = self.temporal_coherence_loss(states, predictions)
            elif task_name == 'kinematic_consistency':
                # Use robot kinematic model
                loss = self.kinematic_consistency_loss(states, predictions)
            else:
                continue

            losses[task_name] = loss

        return losses

    def temporal_coherence_loss(self, states, predictions):
        """Loss for temporal coherence in predictions"""
        # Compare consecutive predictions for consistency
        if len(predictions) < 2:
            return torch.tensor(0.0, requires_grad=True)

        diff = torch.mean((predictions[1:] - predictions[:-1])**2)
        return diff

    def kinematic_consistency_loss(self, states, predictions):
        """Loss based on kinematic model consistency"""
        # Use robot kinematic model to check if predictions are feasible
        # This would require a kinematic model of the humanoid
        return torch.tensor(0.0, requires_grad=True)

def train_with_domain_adaptation():
    """Example training with domain adaptation"""
    # Initialize network
    net = DomainAdaptationNetwork(input_dim=36, hidden_dims=[256, 256], output_dim=12)
    adapter = DomainAdversarialAdapter(net)

    # Load simulation and real data
    # sim_loader = DataLoader(sim_dataset, batch_size=32)
    # real_loader = DataLoader(real_dataset, batch_size=32)

    # Train with domain adaptation
    # adapter.train_adaptation(sim_loader, real_loader)
```

## Validation and Evaluation

### Sim-to-Real Validation Framework

```python
# validation_framework.py - Framework for validating Sim-to-Real transfer
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error, r2_score
from scipy import stats
import json
from datetime import datetime

class SimToRealValidator:
    """Comprehensive validation framework for Sim-to-Real transfer"""

    def __init__(self):
        self.validation_results = {}
        self.metrics_history = []

    def validate_policy_transfer(self, sim_policy, real_robot, num_trials=10):
        """Validate policy transfer from simulation to real robot"""
        sim_rewards = []
        real_rewards = []
        sim_trajectories = []
        real_trajectories = []

        # Test in simulation
        for trial in range(num_trials):
            env = create_sim_env()  # Your sim environment
            state = env.reset()
            episode_reward = 0
            trajectory = []

            while True:
                action = sim_policy.get_action(state)
                next_state, reward, done, info = env.step(action)
                trajectory.append({
                    'state': state.copy(),
                    'action': action.copy(),
                    'reward': reward,
                    'next_state': next_state.copy()
                })
                episode_reward += reward
                state = next_state

                if done:
                    break

            sim_rewards.append(episode_reward)
            sim_trajectories.append(trajectory)

        # Test on real robot
        for trial in range(num_trials):
            state = real_robot.reset()
            episode_reward = 0
            trajectory = []

            while True:
                action = sim_policy.get_action(state)  # Same policy
                next_state, reward, done, info = real_robot.step(action)
                trajectory.append({
                    'state': state.copy(),
                    'action': action.copy(),
                    'reward': reward,
                    'next_state': next_state.copy()
                })
                episode_reward += reward
                state = next_state

                if done or info.get('timeout', False):
                    break

            real_rewards.append(episode_reward)
            real_trajectories.append(trajectory)

        # Calculate validation metrics
        metrics = {
            'sim_avg_reward': np.mean(sim_rewards),
            'real_avg_reward': np.mean(real_rewards),
            'performance_gap': np.mean(sim_rewards) - np.mean(real_rewards),
            'correlation': np.corrcoef(sim_rewards, real_rewards)[0, 1],
            'transfer_efficiency': np.mean(real_rewards) / np.mean(sim_rewards) if np.mean(sim_rewards) > 0 else 0,
            'sim_std': np.std(sim_rewards),
            'real_std': np.std(real_rewards),
        }

        self.validation_results = metrics
        return metrics

    def validate_sensor_data_correspondence(self, sim_sensor_data, real_sensor_data):
        """Validate correspondence between simulated and real sensor data"""
        # Calculate various correspondence metrics
        metrics = {}

        # Time series correlation
        if len(sim_sensor_data) == len(real_sensor_data):
            correlation = np.corrcoef(sim_sensor_data.flatten(), real_sensor_data.flatten())[0, 1]
            metrics['correlation'] = correlation

            # RMSE
            rmse = np.sqrt(mean_squared_error(sim_sensor_data.flatten(), real_sensor_data.flatten()))
            metrics['rmse'] = rmse

            # R² score
            r2 = r2_score(sim_sensor_data.flatten(), real_sensor_data.flatten())
            metrics['r2_score'] = r2

            # Phase correlation (for time series alignment)
            try:
                from scipy.signal import correlate
                cross_corr = correlate(sim_sensor_data.flatten(), real_sensor_data.flatten())
                max_corr = np.max(cross_corr)
                metrics['phase_correlation'] = max_corr
            except ImportError:
                metrics['phase_correlation'] = 0

        return metrics

    def validate_dynamics_correspondence(self, sim_states, real_states):
        """Validate dynamics correspondences between simulation and reality"""
        # Compare state evolution patterns
        metrics = {}

        # State prediction accuracy
        if len(sim_states) > 1 and len(real_states) > 1:
            sim_deltas = np.diff(sim_states, axis=0)
            real_deltas = np.diff(real_states, axis=0)

            delta_correlation = np.corrcoef(sim_deltas.flatten(), real_deltas.flatten())[0, 1]
            metrics['state_delta_correlation'] = delta_correlation

            delta_rmse = np.sqrt(mean_squared_error(sim_deltas.flatten(), real_deltas.flatten()))
            metrics['state_delta_rmse'] = delta_rmse

        return metrics

    def assess_robustness(self, policy, perturbation_levels=[0.0, 0.1, 0.2, 0.3]):
        """Assess policy robustness to perturbations"""
        robustness_results = {}

        for level in perturbation_levels:
            rewards = []
            for trial in range(5):  # Multiple trials per level
                env = create_real_env_with_perturbation(level)
                state = env.reset()
                total_reward = 0

                while True:
                    action = policy.get_action(state)
                    state, reward, done, info = env.step(action)
                    total_reward += reward

                    if done:
                        break

                rewards.append(total_reward)

            robustness_results[level] = {
                'avg_reward': np.mean(rewards),
                'std_reward': np.std(rewards),
                'success_rate': np.sum(np.array(rewards) > 0) / len(rewards)
            }

        return robustness_results

    def generate_validation_report(self):
        """Generate comprehensive validation report"""
        report = {
            'validation_date': datetime.now().isoformat(),
            'results': self.validation_results,
            'recommendations': self._generate_recommendations()
        }

        return report

    def _generate_recommendations(self):
        """Generate recommendations based on validation results"""
        recommendations = []

        if 'transfer_efficiency' in self.validation_results:
            eff = self.validation_results['transfer_efficiency']
            if eff < 0.5:
                recommendations.append(
                    "Transfer efficiency is low (<50%), consider improving domain randomization or collecting more real-world data."
                )
            elif eff < 0.8:
                recommendations.append(
                    "Moderate transfer efficiency, consider fine-tuning on real robot data."
                )
            else:
                recommendations.append(
                    "Good transfer efficiency, policy is ready for deployment with monitoring."
                )

        if 'performance_gap' in self.validation_results:
            gap = self.validation_results['performance_gap']
            if gap > 50:  # Arbitrary threshold
                recommendations.append(
                    "Large performance gap between sim and real, investigate reality gap sources."
                )

        return recommendations

    def plot_validation_results(self):
        """Plot validation results"""
        if not self.validation_results:
            print("No validation results to plot")
            return

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))

        # Performance comparison
        if 'sim_avg_reward' in self.validation_results and 'real_avg_reward' in self.validation_results:
            categories = ['Simulation', 'Real World']
            values = [self.validation_results['sim_avg_reward'], self.validation_results['real_avg_reward']]

            axes[0, 0].bar(categories, values)
            axes[0, 0].set_title('Performance Comparison')
            axes[0, 0].set_ylabel('Average Reward')

        # Transfer efficiency
        if 'transfer_efficiency' in self.validation_results:
            te = self.validation_results['transfer_efficiency']
            axes[0, 1].bar(['Transfer Efficiency'], [te])
            axes[0, 1].set_ylim([0, 1])
            axes[0, 1].set_title('Transfer Efficiency')
            axes[0, 1].set_ylabel('Efficiency')

        # Performance gap
        if 'performance_gap' in self.validation_results:
            gap = self.validation_results['performance_gap']
            axes[1, 0].bar(['Performance Gap'], [gap])
            axes[1, 0].set_title('Performance Gap (Sim - Real)')
            axes[1, 0].set_ylabel('Gap')

        # Correlation
        if 'correlation' in self.validation_results:
            corr = self.validation_results['correlation']
            axes[1, 1].bar(['Correlation'], [corr])
            axes[1, 1].set_ylim([-1, 1])
            axes[1, 1].set_title('Sim-Real Correlation')
            axes[1, 1].set_ylabel('Correlation')

        plt.tight_layout()
        plt.show()

# Example usage
def validate_sim_to_real_transfer():
    """Example validation workflow"""
    validator = SimToRealValidator()

    # Load your trained policy
    # sim_policy = load_trained_policy('sim_policy.pth')

    # Connect to real robot
    # real_robot = connect_to_real_humanoid_robot()

    # Validate policy transfer
    # metrics = validator.validate_policy_transfer(sim_policy, real_robot)

    # Validate sensor correspondence
    # sensor_metrics = validator.validate_sensor_data_correspondence(sim_sensors, real_sensors)

    # Assess robustness
    # robustness = validator.assess_robustness(sim_policy)

    # Generate report
    # report = validator.generate_validation_report()
    # print(json.dumps(report, indent=2))

    # Plot results
    # validator.plot_validation_results()

    return validator
```

## Practical Exercise: Implementing Domain Randomization

Create a complete domain randomization system:

1. **Set up the randomization system**:

```python
# complete_domain_randomization.py - Complete domain randomization implementation
import numpy as np
import torch
import torch.nn as nn
import gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback

class DomainRandomizationCallback(BaseCallback):
    """Callback to apply domain randomization during training"""

    def __init__(self, env, randomization_params, verbose=0):
        super(DomainRandomizationCallback, self).__init__(verbose)
        self.env = env
        self.randomization_params = randomization_params
        self.episode_count = 0

    def _on_step(self) -> bool:
        # Apply domain randomization every few episodes
        if self.num_timesteps % 1000 == 0:  # Randomize every 1000 timesteps
            self._apply_randomization()
        return True

    def _apply_randomization(self):
        """Apply domain randomization to the environment"""
        # This would call the randomization methods
        # For example:
        # randomize_physics_properties()
        # randomize_visual_properties()
        # randomize_sensors()
        pass

def setup_domain_randomization_training():
    """Set up training with domain randomization"""
    # Define randomization parameters
    randomization_params = {
        'mass_range': [0.8, 1.2],
        'friction_range': [0.5, 1.5],
        'sensor_noise_range': [0.0, 0.1],
        'actuator_delay_range': [0.0, 0.02],
        'gravity_range': [-9.91, -9.71]
    }

    # Create environment with domain randomization
    env = make_vec_env('Humanoid-v3', n_envs=4)  # Use multiple environments

    # Initialize callback
    dr_callback = DomainRandomizationCallback(env, randomization_params)

    # Create and train model
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log="./tb_logs/domain_randomization/",
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01
    )

    # Train the model
    model.learn(
        total_timesteps=1000000,
        callback=dr_callback
    )

    # Save the model
    model.save("humanoid_domain_randomized_ppo")

def deploy_to_real_robot():
    """Deploy domain-randomized policy to real robot"""
    # Load the trained policy
    model = PPO.load("humanoid_domain_randomized_ppo")

    # Connect to real robot
    # real_robot = connect_to_real_humanoid_robot()

    # Deploy policy
    obs = real_robot.reset()
    for i in range(1000):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = real_robot.step(action)

        if done:
            obs = real_robot.reset()

if __name__ == "__main__":
    setup_domain_randomization_training()
    # deploy_to_real_robot()  # Uncomment when ready to deploy
```

2. **Test the validation system**:

```bash
# Run validation
python -m validation_framework
```

## Troubleshooting Common Sim-to-Real Issues

### Transfer Performance Issues
- **Large performance gap**: Increase domain randomization range and diversity
- **Poor real-world performance**: Collect more real-world data for fine-tuning
- **Instability**: Reduce randomization range or add more safety constraints

### Domain Randomization Issues
- **Overfitting to randomization**: Reduce randomization range or use curriculum approach
- **Poor learning**: Ensure randomization bounds are realistic
- **Computational overhead**: Optimize randomization code and use efficient sampling

### Validation Issues
- **Inconsistent metrics**: Ensure comparable experimental conditions
- **Biased evaluation**: Use diverse test scenarios and environments
- **Insufficient data**: Collect more data for robust validation

## Summary

In this chapter, we've explored the critical concepts of Sim-to-Real transfer for humanoid robotics. We covered understanding the reality gap, implementing domain randomization techniques, applying transfer learning methods, and validating Sim-to-Real performance. Domain randomization and transfer learning are essential for bridging the gap between simulation and reality, making it possible to develop and test humanoid robot policies in simulation before deploying them to real robots.

## Next Steps

- Implement domain randomization in your simulation environment
- Apply transfer learning techniques to your specific humanoid tasks
- Validate your policies using the framework described
- Fine-tune policies with real robot data
- Continuously monitor and improve transfer performance