import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import styles from './courseNavigation.module.css';

type Chapter = {
  id: string;
  title: string;
  description: string;
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
        description: 'Understanding nodes, topics, services, and actions'
      },
      {
        id: 'chapter-2-rclpy',
        title: 'Python rclpy control',
        description: 'Controlling robots using Python and rclpy'
      },
      {
        id: 'chapter-3-packages',
        title: 'Packages, Launch files, Parameters',
        description: 'Creating and managing ROS 2 packages'
      },
      {
        id: 'chapter-4-urdf',
        title: 'URDF: Joints, Links, Sensors',
        description: 'Modeling robots with Unified Robot Description Format'
      },
      {
        id: 'chapter-5-control-loops',
        title: 'Basic control loop',
        description: 'Implementing locomotion and manipulation control'
      },
      {
        id: 'chapter-6-jetson-deployment',
        title: 'ROS2 + Jetson deployment',
        description: 'Deploying ROS2 applications on NVIDIA Jetson'
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
        description: 'Setting up Gazebo simulation environment'
      },
      {
        id: 'chapter-2-urdf-gazebo-pipeline',
        title: 'URDF → Gazebo pipeline',
        description: 'Converting URDF to simulation models'
      },
      {
        id: 'chapter-3-physics',
        title: 'Physics: gravity, collisions, contacts',
        description: 'Understanding simulation physics'
      },
      {
        id: 'chapter-4-sensor-simulation',
        title: 'Sensor simulation',
        description: 'Simulating LiDAR, depth, IMU, RGB sensors'
      },
      {
        id: 'chapter-5-unity-visuals',
        title: 'Unity for realistic visuals',
        description: 'Enhanced visual rendering with Unity'
      },
      {
        id: 'chapter-6-interactive-testing',
        title: 'Interactive humanoid testing',
        description: 'Testing environments for humanoid robots'
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
        description: 'USD scenes and synthetic data generation'
      },
      {
        id: 'chapter-2-isaac-ros-pipelines',
        title: 'Isaac ROS pipelines',
        description: 'VSLAM, perception, and segmentation'
      },
      {
        id: 'chapter-3-nav2-bipedal-planning',
        title: 'Nav2 for bipedal path planning',
        description: 'Navigation for humanoid robots'
      },
      {
        id: 'chapter-4-reinforcement-learning',
        title: 'Reinforcement learning for robot control',
        description: 'Learning-based control strategies'
      },
      {
        id: 'chapter-5-sim-to-real-concepts',
        title: 'Sim-to-Real concepts',
        description: 'Transferring learning to real robots'
      },
      {
        id: 'chapter-6-integrating-isaac-outputs',
        title: 'Integrating Isaac outputs',
        description: 'Connecting Isaac with ROS2 controllers'
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
        description: 'Language to perception to action pipeline'
      },
      {
        id: 'chapter-2-voice-to-action',
        title: 'Voice-to-Action',
        description: 'From microphone to ROS2 actions'
      },
      {
        id: 'chapter-3-vla-ros2-integration',
        title: 'ROS2 Actions & Nav2 integration',
        description: 'Integrating with ROS2 action servers'
      },
      {
        id: 'chapter-4-advanced-vla-applications',
        title: 'Advanced VLA applications',
        description: 'Complex VLA system applications'
      }
    ]
  }
];

type CourseNavigationProps = {
  showMobileToggle?: boolean;
};

function CourseNavigation({ showMobileToggle = true }: CourseNavigationProps): JSX.Element {
  const location = useLocation();
  const [isMobileOpen, setIsMobileOpen] = useState(false);

  // Determine current module and chapter from URL
  const pathParts = location.pathname.split('/').filter(part => part);
  let currentModuleId = '';
  let currentChapterId = '';

  if (pathParts.length >= 2) {
    currentModuleId = pathParts[1];
    if (pathParts.length >= 3) {
      currentChapterId = pathParts[2];
    }
  }

  const toggleMobileMenu = () => {
    setIsMobileOpen(!isMobileOpen);
  };

  const getChapterProgress = (moduleId: string, chapterId: string) => {
    // In a real implementation, this would fetch actual progress data
    // For now, we'll return a mock value based on the module progress
    const module = modules.find(m => m.id === moduleId);
    if (module) {
      // Mock: chapters have similar progress to their module
      return module.progress;
    }
    return 0;
  };

  return (
    <div className={styles.courseNavigation}>
      {showMobileToggle && (
        <button
          className={styles.mobileToggle}
          onClick={toggleMobileMenu}
          aria-label={isMobileOpen ? 'Close navigation' : 'Open navigation'}
        >
          {isMobileOpen ? '×' : '☰'} Course Navigation
        </button>
      )}

      <div className={`${styles.navigationContainer} ${showMobileToggle && !isMobileOpen ? styles.hiddenMobile : ''}`}>
        <div className={styles.navigationHeader}>
          <h3>Course Content</h3>
          <Link to="/dashboard">View Dashboard</Link>
        </div>

        <div className={styles.modulesList}>
          {modules.map((module) => (
            <div
              key={module.id}
              className={`${styles.moduleItem} ${
                currentModuleId === module.id ? styles.currentModule : ''
              }`}
            >
              <Link
                to={`/docs/${module.id}`}
                className={`${styles.moduleLink} ${
                  currentModuleId === module.id ? styles.current : ''
                }`}
              >
                <span className={styles.moduleTitle}>{module.title}</span>
                <span className={styles.moduleProgress}>{module.progress}%</span>
              </Link>

              <div className={`${styles.chaptersList} ${currentModuleId === module.id ? styles.expanded : ''}`}>
                {module.chapters.map((chapter) => (
                  <Link
                    key={chapter.id}
                    to={`/docs/${module.id}/${chapter.id}`}
                    className={`${styles.chapterLink} ${
                      currentModuleId === module.id && currentChapterId === chapter.id
                        ? styles.current
                        : ''
                    }`}
                  >
                    <span className={styles.chapterTitle}>{chapter.title}</span>
                    <span className={styles.chapterProgress}>{getChapterProgress(module.id, chapter.id)}%</span>
                  </Link>
                ))}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

export default CourseNavigation;