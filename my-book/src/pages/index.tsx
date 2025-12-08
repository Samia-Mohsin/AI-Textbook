import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageHeader from '@site/src/components/HomepageHeader';
import Heading from '@theme/Heading';

import styles from './index.module.css';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home | ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Course - Complete Learning Path from ROS2 fundamentals to autonomous humanoid systems">
      <main>
        {/* Hero Banner Section */}
        <section className={styles.heroBanner}>
          <div className="container">
            <div className="row">
              <div className="col col--12 text--center">
                <img
                  className={styles.heroLogo}
                  src={useBaseUrl('/img/logo.svg')}
                  alt="Humanoid Robotics Book Logo"
                />
                <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
                <p className={styles.heroSubtitle}>
                  Complete Interactive Robotics Education - From ROS2 fundamentals to autonomous humanoid systems with voice control
                </p>
                <div className={styles.heroButtons}>
                  <Link
                    className={`${styles.heroButton} ${styles.primaryButton}`}
                    to="/docs/module-1-ros2/chapter-1-architecture">
                    Start Learning
                  </Link>
                  <Link
                    className={`${styles.heroButton} ${styles.secondaryButton}`}
                    to="/docs/chapter1">
                    Explore Curriculum
                  </Link>
                  <Link
                    className={`${styles.heroButton} ${styles.secondaryButton}`}
                    to="/docs">
                    View All Content
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Stats Section */}
        <section className={styles.statsSection}>
          <div className="container">
            <div className={styles.statsContainer}>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>4</div>
                <div className={styles.statLabel}>Comprehensive Modules</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>20+</div>
                <div className={styles.statLabel}>Hands-on Projects</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>50+</div>
                <div className={styles.statLabel}>Expert Tutorials</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>100%</div>
                <div className={styles.statLabel}>Practical Learning</div>
              </div>
            </div>
          </div>
        </section>

        {/* Key Features Section */}
        <section className={styles.features}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--lg">
              <h2 className={styles.featureTitle}>Complete Interactive Robotics Education</h2>
              <p className="hero__subtitle">From ROS2 fundamentals to autonomous humanoid systems with voice control</p>
            </div>

            <div className="row">
              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">4 Comprehensive Modules</h3>
                      </div>
                    </div>
                  </div>
                  <p>Progress from ROS2 architecture to Vision-Language-Action systems. Master humanoid robot development from middleware to AI brain.</p>
                </div>
              </div>

              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">Code-Ready Instructions</h3>
                      </div>
                    </div>
                  </div>
                  <p>Step-by-step pipelines with ready-to-use code examples for ROS2, Gazebo, Isaac Sim, Whisper, and GPT-based planners.</p>
                </div>
              </div>

              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">Capstone Project</h3>
                      </div>
                    </div>
                  </div>
                  <p>Build a simulated humanoid robot that executes voice commands, performs navigation, and manipulates objects in realistic environments.</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Technology Stack Section */}
        <section className={clsx(styles.features, styles.alternateBackground)}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--lg">
              <h2 className={styles.featureTitle}>Advanced Technology Stack</h2>
              <p className="hero__subtitle">Simulation-first approach with industry-standard tools</p>
            </div>

            <div className="row">
              <div className="col col--3">
                <div className="card padding--md text--center">
                  <h3>ROS 2</h3>
                  <p>Middleware for humanoid robot control: nodes, topics, services, actions</p>
                  <div className="padding-top--sm">
                    <span className="badge badge--primary">Architecture</span>
                    <span className="badge badge--secondary margin-left--sm">rclpy</span>
                  </div>
                </div>
              </div>

              <div className="col col--3">
                <div className="card padding--md text--center">
                  <h3>NVIDIA Isaac</h3>
                  <p>AI perception, SLAM, navigation, and behavior systems</p>
                  <div className="padding-top--sm">
                    <span className="badge badge--primary">VSLAM</span>
                    <span className="badge badge--secondary margin-left--sm">Segmentation</span>
                  </div>
                </div>
              </div>

              <div className="col col--3">
                <div className="card padding--md text--center">
                  <h3>Simulation</h3>
                  <p>Gazebo & Unity for physics and realistic testing</p>
                  <div className="padding-top--sm">
                    <span className="badge badge--primary">Physics</span>
                    <span className="badge badge--secondary margin-left--sm">Sensors</span>
                  </div>
                </div>
              </div>

              <div className="col col--3">
                <div className="card padding--md text--center">
                  <h3>VLA Systems</h3>
                  <p>Vision-Language-Action integration for voice control</p>
                  <div className="padding-top--sm">
                    <span className="badge badge--primary">Voice</span>
                    <span className="badge badge--secondary margin-left--sm">LLM</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Learning Path Section */}
        <section className={styles.learningPathSection}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--lg">
              <h2 className={styles.featureTitle}>Your Learning Journey</h2>
              <p className="hero__subtitle">Structured path from beginner to expert in humanoid robotics</p>
            </div>

            <div className={styles.learningPathSteps}>
              <div className={styles.learningStep}>
                <div className={styles.stepNumber}>1</div>
                <h3 className={styles.stepTitle}>Foundation</h3>
                <p className={styles.stepDescription}>Learn ROS2 fundamentals, architecture, and core concepts needed for humanoid robotics.</p>
              </div>

              <div className={styles.learningStep}>
                <div className={styles.stepNumber}>2</div>
                <h3 className={styles.stepTitle}>Simulation</h3>
                <p className={styles.stepDescription}>Master Gazebo and Unity environments for realistic robot testing and development.</p>
              </div>

              <div className={styles.learningStep}>
                <div className={styles.stepNumber}>3</div>
                <h3 className={styles.stepTitle}>AI Integration</h3>
                <p className={styles.stepDescription}>Implement perception, navigation, and decision-making systems using NVIDIA Isaac.</p>
              </div>

              <div className={styles.learningStep}>
                <div className={styles.stepNumber}>4</div>
                <h3 className={styles.stepTitle}>VLA Systems</h3>
                <p className={styles.stepDescription}>Build robots that understand and respond to human commands through voice and vision.</p>
              </div>
            </div>
          </div>
        </section>

        {/* Testimonials Section */}
        <section className={styles.testimonialsSection}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--lg">
              <h2 className={styles.featureTitle}>What Our Learners Say</h2>
              <p className="hero__subtitle">Join thousands of robotics enthusiasts mastering humanoid systems</p>
            </div>

            <div className="row">
              <div className="col col--4">
                <div className={styles.testimonialCard}>
                  <p className={styles.testimonialText}>"This course transformed my understanding of humanoid robotics. The hands-on approach with real code examples made complex concepts accessible."</p>
                  <p className={styles.testimonialAuthor}>- Alex Johnson, Robotics Engineer</p>
                </div>
              </div>

              <div className="col col--4">
                <div className={styles.testimonialCard}>
                  <p className={styles.testimonialText}>"The VLA systems module was particularly impressive. I built a robot that responds to voice commands in just two weeks!"</p>
                  <p className={styles.testimonialAuthor}>- Sarah Chen, AI Researcher</p>
                </div>
              </div>

              <div className="col col--4">
                <div className={styles.testimonialCard}>
                  <p className={styles.testimonialText}>"The capstone project gave me the portfolio pieces I needed to land my dream job at a robotics startup."</p>
                  <p className={styles.testimonialAuthor}>- Michael Rodriguez, Mechatronics Specialist</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Intelligent Assistance Section */}
        <section className={styles.features}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--lg">
              <h2 className={styles.featureTitle}>Intelligent Content Assistance</h2>
              <p className="hero__subtitle">Embedded RAG chatbot for chapter-specific explanations</p>
            </div>

            <div className="row">
              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">Conversational AI</h3>
                      </div>
                    </div>
                  </div>
                  <p>Get answers to your questions with our RAG chatbot that understands the book's content and provides contextual responses.</p>
                </div>
              </div>

              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">Chapter-Specific</h3>
                      </div>
                    </div>
                  </div>
                  <p>Receive accurate answers based on the specific chapter you're studying with references to relevant concepts and examples.</p>
                </div>
              </div>

              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">External Knowledge</h3>
                      </div>
                    </div>
                  </div>
                  <p>Access integrated external knowledge sources to deepen your understanding of robotics concepts and applications.</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Course Modules Section */}
        <section className={styles.modulesPreview}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--xl">
              <h2>Complete Course Curriculum</h2>
              <p className="hero__subtitle">Structured learning path from fundamentals to advanced concepts with hands-on projects</p>
            </div>

            <div className="row">
              <div className="col col--3">
                <div className="card module-card">
                  <div className="card__header text--center">
                    <h3>Module 1: The Robotic Nervous System (ROS 2)</h3>
                  </div>
                  <div className="card__body">
                    <p>Master ROS 2 architecture, nodes, topics, services, and actions. Build your first robotic applications with industry-standard tools.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-1-ros2/chapter-1-architecture">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>

              <div className="col col--3">
                <div className="card module-card">
                  <div className="card__header text--center">
                    <h3>Module 2: The Digital Twin (Gazebo & Unity)</h3>
                  </div>
                  <div className="card__body">
                    <p>Develop in realistic environments with Gazebo and Unity. Master physics simulation, sensor modeling, and virtual testing.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-2-simulation/chapter-1-gazebo-setup">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>

              <div className="col col--3">
                <div className="card module-card">
                  <div className="card__header text--center">
                    <h3>Module 3: The AI-Robot Brain (NVIDIA Isaac)</h3>
                  </div>
                  <div className="card__body">
                    <p>Implement perception systems, navigation, and AI behavior using NVIDIA Isaac. Create intelligent robot decision-making.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-3-ai-brain/chapter-1-isaac-sim-setup">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>

              <div className="col col--3">
                <div className="card module-card">
                  <div className="card__header text--center">
                    <h3>Module 4: Vision-Language-Action (VLA)</h3>
                  </div>
                  <div className="card__body">
                    <p>Integrate vision, language, and action systems. Build robots that understand and respond to human commands.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-4-vla/chapter-1-vla-overview">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>
            </div>

            {/* All Chapters Overview */}
            <div className="text--center padding-top--xl">
              <Link
                className="button button--secondary button--lg"
                to="/docs">
                Browse All Chapters & Content
              </Link>
            </div>
          </div>
        </section>

        {/* Capstone Project Section */}
        <section className={clsx(styles.features, styles.alternateBackground)}>
          <div className="container padding-vert--xl">
            <div className="row">
              <div className="col col--6">
                <h2>Capstone Project: Autonomous Humanoid Robot</h2>
                <p className="hero__subtitle">Integrate everything you've learned into a complete humanoid system</p>
                <ul>
                  <li>Voice command processing and natural language understanding</li>
                  <li>Environmental perception and spatial reasoning</li>
                  <li>Autonomous navigation and manipulation</li>
                  <li>Safe operation with fallback behaviors</li>
                  <li>Complete system integration and deployment</li>
                  <li>Working ROS2 project with simulation environment</li>
                  <li>Architecture diagram and demo video deliverables</li>
                </ul>
                <div className="padding-top--md">
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/capstone-project/intro">
                    Explore Capstone
                  </Link>
                </div>
              </div>
              <div className="col col--6 text--center">
                <div className="avatar avatar--vertical padding--lg">
                  <img
                    className="avatar__photo avatar__photo--xl"
                    src={useBaseUrl('/img/undraw_docusaurus_react.svg')}
                    alt="Humanoid Robot"
                  />
                  <div className="avatar__intro">
                    <h3 className="avatar__name">Complete Integration</h3>
                    <small>Bringing all systems together</small>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Community Section */}
        <section className={styles.communitySection}>
          <div className="container padding-vert--xl">
            <h2 className={styles.communityTitle}>Join Our Robotics Community</h2>
            <div className={styles.communityCards}>
              <div className={styles.communityCard}>
                <div className={styles.communityIcon}>👥</div>
                <h3>Active Forums</h3>
                <p>Connect with fellow learners and experts to discuss challenges and solutions in humanoid robotics.</p>
              </div>

              <div className={styles.communityCard}>
                <div className={styles.communityIcon}>🎓</div>
                <h3>Expert Mentorship</h3>
                <p>Get guidance from industry professionals with years of experience in robotics development.</p>
              </div>

              <div className={styles.communityCard}>
                <div className={styles.communityIcon}>🚀</div>
                <h3>Project Showcase</h3>
                <p>Share your creations and learn from others' innovative humanoid robot implementations.</p>
              </div>
            </div>
          </div>
        </section>

        {/* Advanced Features Section */}
        <section className={styles.features}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--lg">
              <h2 className={styles.featureTitle}>Advanced Learning Features</h2>
              <p className="hero__subtitle">Personalized education with analytics and international accessibility</p>
            </div>

            <div className="row">
              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">Advanced Analytics</h3>
                      </div>
                    </div>
                  </div>
                  <p>Track your progress with detailed analytics and receive personalized learning path recommendations based on your performance.</p>
                </div>
              </div>

              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">Localization</h3>
                      </div>
                    </div>
                  </div>
                  <p>Content available in multiple languages including Urdu to enable international accessibility and diverse learning communities.</p>
                </div>
              </div>

              <div className="col col--4">
                <div className="card padding--lg text--center">
                  <div className="text--center padding-bottom--md">
                    <div className="avatar avatar--vertical">
                      <div className="avatar__intro">
                        <h3 className="avatar__name">Enterprise Security</h3>
                      </div>
                    </div>
                  </div>
                  <p>Enterprise-level security with advanced threat protection and comprehensive data privacy measures for large-scale public access.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
