# Physical AI & Humanoid Robotics Book

An interactive educational platform for learning Physical AI and Humanoid Robotics using ROS2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) systems.

## Project Structure

This project consists of two main components:

1. **my-book/**: Docusaurus-based web platform for the educational content
2. **ros2_ws/**: ROS2 workspace containing robotics packages and simulation code

## Getting Started

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill or Iron Irwini
- Node.js 18+ and npm/yarn
- Python 3.10+
- NVIDIA Isaac Sim (optional, for advanced simulation)

### Setup

1. Clone the repository
2. Install ROS2 dependencies: `pip install -r requirements.txt`
3. Navigate to the web platform: `cd my-book`
4. Install web dependencies: `npm install`
5. Start the development server: `npm run start`

## Features

- 4 comprehensive modules covering ROS2, Simulation, AI Perception, and VLA
- Interactive content with code examples
- RAG-based chatbot for content assistance
- Capstone project with autonomous humanoid robot
- Localization support (Urdu and more)

## Development

The project follows a modular architecture with separate components for web content, robotics code, and simulation. See the [specification](specs/001-humanoid-robotics-book/spec.md) for detailed requirements.

## Contributing

See the [contribution guidelines](.specify/templates/CONTRIBUTING.md) for details on how to contribute to this project.