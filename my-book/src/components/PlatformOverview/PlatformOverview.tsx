import React from 'react';
import clsx from 'clsx';
import Button from '../Button';
import Card from '../Card';
import styles from './PlatformOverview.module.css';

const PlatformOverview: React.FC = () => {
  const features = [
    {
      title: 'ROS2 Fundamentals',
      description: 'Master the Robot Operating System 2 with hands-on tutorials and practical examples',
      icon: '🤖',
      link: '/docs/module-1-ros2/chapter-1-architecture'
    },
    {
      title: 'Simulation Environments',
      description: 'Gazebo and Unity integration for realistic physics and visual simulation',
      icon: '🎮',
      link: '/docs/module-2-simulation/chapter-1-gazebo-setup'
    },
    {
      title: 'AI Perception Systems',
      description: 'NVIDIA Isaac integration for advanced computer vision and sensor processing',
      icon: '👁️',
      link: '/docs/module-3-ai-brain/chapter-1-isaac-sim-setup'
    },
    {
      title: 'Vision-Language-Action',
      description: 'Connect AI models to robots with advanced VLA systems for natural interaction',
      icon: '🗣️',
      link: '/docs/module-4-vla/chapter-1-vla-overview'
    },
    {
      title: 'Capstone Project',
      description: 'Build an autonomous humanoid robot that executes voice commands',
      icon: '🏆',
      link: '/docs/capstone-project/intro'
    },
    {
      title: 'Intelligent Assistance',
      description: 'Embedded RAG chatbot for chapter-specific explanations',
      icon: '💬',
      link: '/docs/intro'
    }
  ];

  const modules = [
    {
      title: 'Module 1: ROS2 Fundamentals',
      description: 'Core ROS2 concepts including architecture, nodes, topics, and service implementation.',
      chapters: 6,
      duration: '4 weeks'
    },
    {
      title: 'Module 2: Simulation',
      description: 'Gazebo and Unity simulation environments, physics modeling, and sensor integration.',
      chapters: 6,
      duration: '4 weeks'
    },
    {
      title: 'Module 3: AI Perception',
      description: 'NVIDIA Isaac ecosystem, navigation, reinforcement learning, and sim-to-real transfer.',
      chapters: 6,
      duration: '4 weeks'
    },
    {
      title: 'Module 4: VLA Systems',
      description: 'Vision-Language-Action systems for robotic task understanding and execution.',
      chapters: 7,
      duration: '4 weeks'
    }
  ];

  return (
    <div className={styles.container}>
      <div className={styles.hero}>
        <h1 className={styles.title}>Physical AI & Humanoid Robotics</h1>
        <p className={styles.subtitle}>
          Comprehensive 4-Module Book on Physical AI & Humanoid Robotics with Conversational AI Assistance
        </p>
        <div className={styles.heroButtons}>
          <Button variant="primary" size="large" href="/docs/intro">
            Start Learning
          </Button>
          <Button variant="outline" size="large" href="/docs/capstone-project/intro">
            Capstone Project
          </Button>
        </div>
      </div>

      <div className={styles.featuresSection}>
        <h2 className={styles.sectionTitle}>Key Features</h2>
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <Card
              key={index}
              title={feature.title}
              description={feature.description}
              linkUrl={feature.link}
              variant="feature"
              className={styles.featureCard}
            >
              <div className={styles.featureIcon}>{feature.icon}</div>
            </Card>
          ))}
        </div>
      </div>

      <div className={styles.modulesSection}>
        <h2 className={styles.sectionTitle}>Learning Modules</h2>
        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <Card
              key={index}
              title={module.title}
              description={module.description}
              className={styles.moduleCard}
            >
              <div className={styles.moduleDetails}>
                <div className={styles.detailItem}>
                  <span className={styles.detailLabel}>Chapters:</span>
                  <span className={styles.detailValue}>{module.chapters}</span>
                </div>
                <div className={styles.detailItem}>
                  <span className={styles.detailLabel}>Duration:</span>
                  <span className={styles.detailValue}>{module.duration}</span>
                </div>
              </div>
              <Button
                variant="secondary"
                size="small"
                href={`/docs/${module.title.toLowerCase().replace(' ', '-').replace(':', '')}/chapter-1`}
                className={styles.moduleButton}
              >
                Explore Module
              </Button>
            </Card>
          ))}
        </div>
      </div>

      <div className={styles.ctaSection}>
        <Card className={styles.ctaCard}>
          <h2 className={styles.ctaTitle}>Ready to Build Your Humanoid Robot?</h2>
          <p className={styles.ctaDescription}>
            Join thousands of students, developers, and educators mastering humanoid robotics with our comprehensive curriculum.
          </p>
          <div className={styles.ctaButtons}>
            <Button variant="primary" size="large" href="/docs/intro">
              Start Learning Today
            </Button>
            <Button variant="outline" size="large" href="/blog">
              Read Our Blog
            </Button>
          </div>
        </Card>
      </div>

      <div className={styles.techStackSection}>
        <h2 className={styles.sectionTitle}>Technology Stack</h2>
        <div className={styles.techGrid}>
          <div className={styles.techItem}>
            <h4>ROS 2</h4>
            <p>Middleware for humanoid robot control: nodes, topics, services, actions</p>
          </div>
          <div className={styles.techItem}>
            <h4>NVIDIA Isaac</h4>
            <p>AI perception, SLAM, navigation, and behavior systems</p>
          </div>
          <div className={styles.techItem}>
            <h4>Simulation</h4>
            <p>Gazebo & Unity for physics and realistic testing</p>
          </div>
          <div className={styles.techItem}>
            <h4>VLA Systems</h4>
            <p>Vision-Language-Action integration for voice control</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default PlatformOverview;