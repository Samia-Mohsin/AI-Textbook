import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useLocation } from '@docusaurus/router';
import { useUser } from '../contexts/UserContext';
import { useLanguage } from '../contexts/LanguageContext';
import IntelligentAssistant from '../components/IntelligentAssistant';
import ProgressIndicator from '../components/ProgressIndicator';
import CourseNavigation from '../components/CourseNavigation';
import styles from './chapterPage.module.css';

type Chapter = {
  id: string;
  title: string;
  description: string;
  moduleId: string;
  moduleTitle: string;
  content: string;
  progress: number; // 0-100 percentage
};

type Module = {
  id: string;
  title: string;
  description: string;
  chapters: Chapter[];
  progress: number; // 0-100 percentage
};

// Mock data - in a real implementation, this would come from an API or content files
const modules: Module[] = [
  {
    id: 'module-1-ros2',
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn about ROS 2 architecture, nodes, topics, services, and actions',
    progress: 100,
    chapters: [
      {
        id: 'chapter-1-architecture',
        title: 'ROS2 Architecture',
        description: 'Understanding nodes, topics, services, and actions',
        moduleId: 'module-1-ros2',
        moduleTitle: 'Module 1: The Robotic Nervous System (ROS 2)',
        content: 'This chapter covers the fundamental concepts of ROS2 architecture including nodes, topics, services, and actions. You will learn how to create and manage ROS2 nodes, publish and subscribe to topics, make service calls, and use actions for more complex interactions.',
        progress: 100
      },
      {
        id: 'chapter-2-rclpy',
        title: 'Python rclpy control',
        description: 'Controlling robots using Python and rclpy',
        moduleId: 'module-1-ros2',
        moduleTitle: 'Module 1: The Robotic Nervous System (ROS 2)',
        content: 'This chapter focuses on controlling robots using Python and the rclpy library. You will learn how to create ROS2 clients and services in Python, publish messages, and handle callbacks.',
        progress: 100
      },
      {
        id: 'chapter-3-packages',
        title: 'Packages, Launch files, Parameters',
        description: 'Creating and managing ROS 2 packages',
        moduleId: 'module-1-ros2',
        moduleTitle: 'Module 1: The Robotic Nervous System (ROS 2)',
        content: 'This chapter covers ROS2 packages, launch files, and parameters. You will learn how to create and organize packages, write launch files to start multiple nodes, and manage parameters for your nodes.',
        progress: 100
      },
      {
        id: 'chapter-4-urdf',
        title: 'URDF: Joints, Links, Sensors',
        description: 'Modeling robots with Unified Robot Description Format',
        moduleId: 'module-1-ros2',
        moduleTitle: 'Module 1: The Robotic Nervous System (ROS 2)',
        content: 'This chapter introduces URDF (Unified Robot Description Format) for modeling robots. You will learn how to define joints, links, and sensors in URDF files to describe your robot.',
        progress: 100
      },
      {
        id: 'chapter-5-control-loops',
        title: 'Basic control loop',
        description: 'Implementing locomotion and manipulation control',
        moduleId: 'module-1-ros2',
        moduleTitle: 'Module 1: The Robotic Nervous System (ROS 2)',
        content: 'This chapter covers implementing basic control loops for locomotion and manipulation. You will learn how to create control systems for robot movement and manipulation tasks.',
        progress: 100
      },
      {
        id: 'chapter-6-jetson-deployment',
        title: 'ROS2 + Jetson deployment',
        description: 'Deploying ROS2 applications on NVIDIA Jetson',
        moduleId: 'module-1-ros2',
        moduleTitle: 'Module 1: The Robotic Nervous System (ROS 2)',
        content: 'This chapter focuses on deploying ROS2 applications on NVIDIA Jetson platforms. You will learn about optimization techniques and deployment strategies for edge computing devices.',
        progress: 100
      }
    ]
  },
  {
    id: 'module-2-simulation',
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: 'Simulation environments, physics, and virtual testing',
    progress: 100,
    chapters: [
      {
        id: 'chapter-1-gazebo-setup',
        title: 'Gazebo setup',
        description: 'Setting up Gazebo simulation environment',
        moduleId: 'module-2-simulation',
        moduleTitle: 'Module 2: The Digital Twin (Gazebo & Unity)',
        content: 'This chapter covers setting up the Gazebo simulation environment. You will learn how to install and configure Gazebo for robot simulation.',
        progress: 100
      },
      {
        id: 'chapter-2-urdf-gazebo-pipeline',
        title: 'URDF → Gazebo pipeline',
        description: 'Converting URDF to simulation models',
        moduleId: 'module-2-simulation',
        moduleTitle: 'Module 2: The Digital Twin (Gazebo & Unity)',
        content: 'This chapter covers the pipeline for converting URDF models to Gazebo simulation models. You will learn how to prepare your robot models for simulation.',
        progress: 100
      },
      {
        id: 'chapter-3-physics',
        title: 'Physics: gravity, collisions, contacts',
        description: 'Understanding simulation physics',
        moduleId: 'module-2-simulation',
        moduleTitle: 'Module 2: The Digital Twin (Gazebo & Unity)',
        content: 'This chapter covers simulation physics including gravity, collisions, and contacts. You will learn how to configure physics parameters for realistic simulation.',
        progress: 100
      },
      {
        id: 'chapter-4-sensor-simulation',
        title: 'Sensor simulation',
        description: 'Simulating LiDAR, depth, IMU, RGB sensors',
        moduleId: 'module-2-simulation',
        moduleTitle: 'Module 2: The Digital Twin (Gazebo & Unity)',
        content: 'This chapter covers simulating various sensors including LiDAR, depth, IMU, and RGB sensors. You will learn how to configure and use these sensors in simulation.',
        progress: 100
      },
      {
        id: 'chapter-5-unity-visuals',
        title: 'Unity for realistic visuals',
        description: 'Enhanced visual rendering with Unity',
        moduleId: 'module-2-simulation',
        moduleTitle: 'Module 2: The Digital Twin (Gazebo & Unity)',
        content: 'This chapter covers using Unity for enhanced visual rendering in robotics simulation. You will learn how to create realistic visual environments.',
        progress: 100
      },
      {
        id: 'chapter-6-interactive-testing',
        title: 'Interactive humanoid testing',
        description: 'Testing environments for humanoid robots',
        moduleId: 'module-2-simulation',
        moduleTitle: 'Module 2: The Digital Twin (Gazebo & Unity)',
        content: 'This chapter covers creating testing environments for humanoid robots. You will learn how to design and implement interactive testing scenarios.',
        progress: 100
      }
    ]
  },
  {
    id: 'module-3-ai-brain',
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    description: 'Perception, SLAM, navigation, and AI behavior',
    progress: 100,
    chapters: [
      {
        id: 'chapter-1-isaac-sim-setup',
        title: 'Isaac Sim setup',
        description: 'USD scenes and synthetic data generation',
        moduleId: 'module-3-ai-brain',
        moduleTitle: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
        content: 'This chapter covers setting up NVIDIA Isaac Sim for USD scenes and synthetic data generation. You will learn how to create realistic simulation environments.',
        progress: 100
      },
      {
        id: 'chapter-2-isaac-ros-pipelines',
        title: 'Isaac ROS pipelines',
        description: 'VSLAM, perception, and segmentation',
        moduleId: 'module-3-ai-brain',
        moduleTitle: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
        content: 'This chapter covers Isaac ROS pipelines for VSLAM, perception, and segmentation. You will learn how to implement perception systems using Isaac ROS packages.',
        progress: 100
      },
      {
        id: 'chapter-3-nav2-bipedal-planning',
        title: 'Nav2 for bipedal path planning',
        description: 'Navigation for humanoid robots',
        moduleId: 'module-3-ai-brain',
        moduleTitle: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
        content: 'This chapter covers using Nav2 for bipedal path planning in humanoid robots. You will learn how to configure navigation systems for legged robots.',
        progress: 100
      },
      {
        id: 'chapter-4-reinforcement-learning',
        title: 'Reinforcement learning for robot control',
        description: 'Learning-based control strategies',
        moduleId: 'module-3-ai-brain',
        moduleTitle: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
        content: 'This chapter covers reinforcement learning techniques for robot control. You will learn how to implement learning-based control strategies.',
        progress: 100
      },
      {
        id: 'chapter-5-sim-to-real-concepts',
        title: 'Sim-to-Real concepts',
        description: 'Transferring learning to real robots',
        moduleId: 'module-3-ai-brain',
        moduleTitle: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
        content: 'This chapter covers Sim-to-Real concepts for transferring learning from simulation to real robots. You will learn about domain randomization and transfer techniques.',
        progress: 100
      },
      {
        id: 'chapter-6-integrating-isaac-outputs',
        title: 'Integrating Isaac outputs',
        description: 'Connecting Isaac with ROS2 controllers',
        moduleId: 'module-3-ai-brain',
        moduleTitle: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
        content: 'This chapter covers integrating Isaac outputs with ROS2 controllers. You will learn how to connect perception and planning outputs to control systems.',
        progress: 100
      }
    ]
  },
  {
    id: 'module-4-vla',
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Linking speech, vision, LLMs to robot actions',
    progress: 100,
    chapters: [
      {
        id: 'chapter-1-vla-overview',
        title: 'VLA overview',
        description: 'Language to perception to action pipeline',
        moduleId: 'module-4-vla',
        moduleTitle: 'Module 4: Vision-Language-Action (VLA)',
        content: 'This chapter provides an overview of Vision-Language-Action systems. You will learn about the pipeline from language understanding to perception to action execution.',
        progress: 100
      },
      {
        id: 'chapter-2-voice-to-action',
        title: 'Voice-to-Action',
        description: 'From microphone to ROS2 actions',
        moduleId: 'module-4-vla',
        moduleTitle: 'Module 4: Vision-Language-Action (VLA)',
        content: 'This chapter covers Voice-to-Action systems, converting microphone input to ROS2 actions. You will learn about speech recognition and action mapping.',
        progress: 100
      },
      {
        id: 'chapter-3-vla-ros2-integration',
        title: 'ROS2 Actions & Nav2 integration',
        description: 'Integrating with ROS2 action servers',
        moduleId: 'module-4-vla',
        moduleTitle: 'Module 4: Vision-Language-Action (VLA)',
        content: 'This chapter covers integrating VLA systems with ROS2 action servers and Nav2. You will learn how to connect high-level commands to low-level actions.',
        progress: 100
      },
      {
        id: 'chapter-4-advanced-vla-applications',
        title: 'Advanced VLA applications',
        description: 'Complex VLA system applications',
        moduleId: 'module-4-vla',
        moduleTitle: 'Module 4: Vision-Language-Action (VLA)',
        content: 'This chapter covers advanced applications of VLA systems. You will learn about complex command interpretation and multi-step task execution.',
        progress: 100
      }
    ]
  }
];

function ChapterPage(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const location = useLocation();

  // Extract module and chapter IDs from the URL
  const pathParts = location.pathname.split('/').filter(part => part);
  const moduleId = pathParts.length >= 2 ? pathParts[1] : '';
  const chapterId = pathParts.length >= 3 ? pathParts[2] : '';

  // Find the requested chapter
  let chapter: Chapter | undefined;
  let module: Module | undefined;

  for (const mod of modules) {
    if (mod.id === moduleId) {
      module = mod;
      for (const chap of mod.chapters) {
        if (chap.id === chapterId) {
          chapter = chap;
          break;
        }
      }
      break;
    }
  }

  if (!chapter || !module) {
    return (
      <Layout
        title={`Chapter Not Found | ${siteConfig.title}`}
        description="Chapter not found">
        <main className={styles.main}>
          <div className="container margin-vert--lg">
            <div className="row">
              <div className="col col--12">
                <div className="text--center padding-vert--xl">
                  <h1 className="hero__title">Chapter Not Found</h1>
                  <p className="hero__subtitle">The requested chapter does not exist.</p>
                  <Link className="button button--primary" to="/modules">
                    Browse All Modules
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  // Find previous and next chapters for navigation
  const chapterIndex = module.chapters.findIndex(chap => chap.id === chapterId);
  const prevChapter = chapterIndex > 0 ? module.chapters[chapterIndex - 1] : null;
  const nextChapter = chapterIndex < module.chapters.length - 1 ? module.chapters[chapterIndex + 1] : null;

  return (
    <Layout
      title={`${chapter.title} | ${siteConfig.title}`}
      description={chapter.description}>
      <main className={styles.main}>
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--12">
              <nav className="breadcrumbs" aria-label="breadcrumbs">
                <ul className="breadcrumbs__list">
                  <li className="breadcrumbs__item">
                    <Link className="breadcrumbs__link" to="/">Home</Link>
                  </li>
                  <li className="breadcrumbs__item">
                    <Link className="breadcrumbs__link" to="/modules">Modules</Link>
                  </li>
                  <li className="breadcrumbs__item">
                    <Link className="breadcrumbs__link" to={`/docs/${module.id}`}>{module.title}</Link>
                  </li>
                  <li className="breadcrumbs__item breadcrumbs__item--active">
                    <span className="breadcrumbs__link">{chapter.title}</span>
                  </li>
                </ul>
              </nav>
            </div>
          </div>

          <div className="row">
            <div className="col col--12">
              <header className="hero hero--primary margin-bottom--lg">
                <div className="container">
                  <h1 className="hero__title">{chapter.title}</h1>
                  <p className="hero__subtitle">{chapter.description}</p>
                </div>
              </header>
            </div>
          </div>

          <div className="row">
            <div className="col col--8">
              <div className="card margin-bottom--lg">
                <div className="card__header">
                  <h3>Chapter Progress</h3>
                </div>
                <div className="card__body">
                  <div className="margin-bottom--md">
                    <ProgressIndicator progress={chapter.progress} size="large" showPercentage={true} />
                  </div>
                  <div className="markdown">
                    {chapter.content.split('\n').map((paragraph, index) => (
                      <p key={index}>{paragraph}</p>
                    ))}
                  </div>
                </div>
              </div>
            </div>
            <div className="col col--4">
              <div className="card margin-bottom--lg">
                <div className="card__header">
                  <h3>AI Assistant</h3>
                </div>
                <div className="card__body">
                  <IntelligentAssistant />
                </div>
              </div>
            </div>
          </div>

          <div className="row">
            <div className="col col--3">
              <CourseNavigation />
            </div>
            <div className="col col--9">
              <div className="card">
                <div className="card__footer row">
                  <div className="col col--6">
                    {prevChapter ? (
                      <Link
                        className="button button--secondary button--outline"
                        to={`/docs/${prevChapter.moduleId}/${prevChapter.id}`}>
                        ← Previous: {prevChapter.title}
                      </Link>
                    ) : (
                      <Link
                        className="button button--secondary button--outline"
                        to={`/docs/${module.id}`}>
                        ← Back to {module.title}
                      </Link>
                    )}
                  </div>
                  <div className="col col--6" style={{ textAlign: 'right' }}>
                    {nextChapter ? (
                      <Link
                        className="button button--primary"
                        to={`/docs/${nextChapter.moduleId}/${nextChapter.id}`}>
                        Next: {nextChapter.title} →
                      </Link>
                    ) : (
                      <Link
                        className="button button--primary"
                        to={`/docs/${module.id}`}>
                        Complete Module →
                      </Link>
                    )}
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default ChapterPage;