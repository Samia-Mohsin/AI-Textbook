import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  bookSidebar: [
    {
      type: 'category',
      label: 'Part 1: Foundations of Robotics & AI',
      link: {
        type: 'generated-index',
        title: 'Part 1 Overview',
        description: 'Foundational concepts of Physical AI and Humanoid Robotics, covering ROS2 and URDF.',
        slug: '/category/part-1-foundations',
      },
      items: [
        'chapter1',
        // 'chapter2', // Placeholder for future chapters - commented out since it doesn't exist yet
        // 'chapter3', // Placeholder for future chapters
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS2 Fundamentals',
      link: {
        type: 'generated-index',
        title: 'Module 1 Overview',
        description: 'Core ROS2 concepts including architecture, nodes, topics, and service implementation.',
        slug: '/category/module-1-ros2',
      },
      items: [
        'module-1-ros2/chapter-1-architecture',
        'module-1-ros2/chapter-2-rclpy',
        'module-1-ros2/chapter-3-packages',
        'module-1-ros2/chapter-4-urdf',
        'module-1-ros2/chapter-5-control-loops',
        'module-1-ros2/chapter-6-jetson-deployment',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      link: {
        type: 'generated-index',
        title: 'Module 2 Overview',
        description: 'Gazebo and Unity simulation environments, physics modeling, and sensor integration.',
        slug: '/category/module-2-simulation',
      },
      items: [
        'module-2-simulation/chapter-1-gazebo-setup',
        'module-2-simulation/chapter-2-urdf-gazebo-pipeline',
        'module-2-simulation/chapter-3-physics',
        'module-2-simulation/chapter-4-sensor-simulation',
        'module-2-simulation/chapter-5-unity-visuals',
        'module-2-simulation/chapter-6-interactive-testing',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI & Brain',
      link: {
        type: 'generated-index',
        title: 'Module 3 Overview',
        description: 'NVIDIA Isaac ecosystem, navigation, reinforcement learning, and sim-to-real transfer.',
        slug: '/category/module-3-ai-brain',
      },
      items: [
        'module-3-ai-brain/chapter-1-isaac-sim-setup',
        'module-3-ai-brain/chapter-2-isaac-ros-pipelines',
        'module-3-ai-brain/chapter-3-nav2-bipedal-planning',
        'module-3-ai-brain/chapter-4-reinforcement-learning',
        'module-3-ai-brain/chapter-5-sim-to-real-concepts',
        'module-3-ai-brain/chapter-6-integrating-isaac-outputs',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Systems',
      link: {
        type: 'generated-index',
        title: 'Module 4 Overview',
        description: 'Vision-Language-Action systems for robotic task understanding and execution.',
        slug: '/category/module-4-vla',
      },
      items: [
        'module-4-vla/chapter-1-vla-overview',
        'module-4-vla/chapter-2-voice-to-action',
        'module-4-vla/chapter-3-cognitive-planning',
        'module-4-vla/chapter-4-ros2-actions-integration',
        'module-4-vla/chapter-5-vision-language-grounding',
        'module-4-vla/chapter-6-safety-behaviors',
        'module-4-vla/chapter-7-capstone-autonomous-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project: Autonomous Humanoid Robot',
      link: {
        type: 'generated-index',
        title: 'Capstone Project Overview',
        description: 'Integrating all learned concepts in a comprehensive humanoid robotics application.',
        slug: '/category/capstone-project',
      },
      items: [
        'capstone-project/intro',
        'capstone-project/phase-1-robot-design',
        'capstone-project/phase-2-locomotion',
        'capstone-project/phase-3-perception',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
