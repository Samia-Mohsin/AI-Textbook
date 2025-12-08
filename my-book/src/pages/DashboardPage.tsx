import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ProgressIndicator from '../components/ProgressIndicator';
import CourseNavigation from '../components/CourseNavigation';
import styles from './dashboard.module.css';

type Module = {
  id: string;
  title: string;
  description: string;
  chapters: Array<{
    id: string;
    title: string;
    description: string;
  }>;
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
    progress: 85,
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
    progress: 60,
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
    progress: 30,
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

function DashboardCard({ title, value, description }: { title: string; value: string; description: string }) {
  return (
    <div className="card">
      <div className="card__body text--center">
        <h3>{title}</h3>
        <p className="text--success" style={{ fontSize: '2rem', fontWeight: 'bold' }}>{value}</p>
        <p>{description}</p>
      </div>
    </div>
  );
}

function ChapterProgress({ chapter, moduleId }: { chapter: any; moduleId: string }) {
  // In a real implementation, this would fetch actual progress data
  const chapterProgress = Math.floor(Math.random() * 100); // Mock progress

  return (
    <div className="row margin-bottom--sm">
      <div className="col col--8">
        <Link to={`/docs/${moduleId}/${chapter.id}`}>
          <strong>{chapter.title}</strong>
        </Link>
        <p className="margin-bottom--none">{chapter.description}</p>
      </div>
      <div className="col col--4">
        <ProgressIndicator progress={chapterProgress} size="small" showPercentage={true} />
      </div>
    </div>
  );
}

export default function DashboardPage(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  // Calculate overall progress
  const overallProgress = modules.reduce((sum, module) => sum + module.progress, 0) / modules.length;

  // Count completed modules
  const completedModules = modules.filter(module => module.progress === 100).length;

  return (
    <Layout
      title={`Learning Dashboard | ${siteConfig.title}`}
      description="Track your progress through the Physical AI & Humanoid Robotics course">
      <main className={styles.main}>
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--12">
              <header className="hero hero--primary margin-bottom--lg">
                <div className="container">
                  <h1 className="hero__title">Learning Dashboard</h1>
                  <p className="hero__subtitle">Track your progress through the Physical AI & Humanoid Robotics course</p>
                </div>
              </header>
            </div>
          </div>

          {/* Stats Overview */}
          <div className="row margin-bottom--lg">
            <div className="col col--3">
              <DashboardCard
                title="Overall Progress"
                value={`${Math.round(overallProgress)}%`}
                description="Average progress across all modules"
              />
            </div>
            <div className="col col--3">
              <DashboardCard
                title="Modules Completed"
                value={`${completedModules}/${modules.length}`}
                description="Number of modules completed"
              />
            </div>
            <div className="col col--3">
              <DashboardCard
                title="Total Chapters"
                value={`${modules.reduce((sum, module) => sum + module.chapters.length, 0)}`}
                description="All chapters in the course"
              />
            </div>
            <div className="col col--3">
              <DashboardCard
                title="Current Focus"
                value={modules.find(m => m.progress < 100)?.title.split(':')[0] || 'All Done!'}
                description="Module needing attention"
              />
            </div>
          </div>

          {/* Overall Progress */}
          <div className="row margin-bottom--lg">
            <div className="col col--12">
              <div className="card">
                <div className="card__header">
                  <h2>Course Progress</h2>
                </div>
                <div className="card__body">
                  <ProgressIndicator progress={overallProgress} size="large" label="Overall Course Progress" />
                </div>
              </div>
            </div>
          </div>

          {/* Module Progress */}
          <div className="row">
            <div className="col col--3">
              <CourseNavigation />
            </div>
            <div className="col col--9">
              <h2>Module Progress</h2>
              {modules.map((module) => (
                <div key={module.id} className="card margin-bottom--md">
                  <div className="card__header">
                    <h3>
                      <Link to={`/docs/${module.id}`}>
                        {module.title}
                      </Link>
                    </h3>
                  </div>
                  <div className="card__body">
                    <div className="row">
                      <div className="col col--8">
                        <p>{module.description}</p>

                        {/* Chapter List */}
                        <div className="margin-top--md">
                          <h4>Chapters:</h4>
                          {module.chapters.map((chapter) => (
                            <ChapterProgress
                              key={chapter.id}
                              chapter={chapter}
                              moduleId={module.id}
                            />
                          ))}
                        </div>
                      </div>
                      <div className="col col--4">
                        <ProgressIndicator progress={module.progress} size="large" label="Module Progress" />
                      </div>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}