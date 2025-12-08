# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended)
- 32GB+ RAM
- NVIDIA GPU with CUDA support (RTX series recommended)
- 100GB+ free disk space
- Internet connection for package downloads

### Software Dependencies
- ROS2 Humble Hawksbill
- Python 3.10+
- Node.js 18+ and npm
- Docker and Docker Compose
- Git

## Setup Instructions

### 1. Clone and Initialize Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Setup ROS2 Environment
```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions

# Source ROS2 environment
source /opt/ros/humble/setup.bash
```

### 3. Setup Web Platform (Docusaurus)
```bash
cd my-book
npm install
```

### 4. Setup ROS2 Workspace
```bash
cd ros2_ws
# Create colcon workspace
mkdir -p src
# Source ROS2 before building
source /opt/ros/humble/setup.bash
# Build the workspace
colcon build --packages-select humanoid_control isaac_ros_vslam
```

### 5. Environment Configuration
```bash
# Create environment file
cp .env.example .env
# Edit .env with your configuration
nano .env
```

## Running the Book Platform

### 1. Start Web Platform
```bash
cd my-book
npm start
# Platform will be available at http://localhost:3000
```

### 2. Start ROS2 Simulation Environment
```bash
cd ros2_ws
source install/setup.bash
source /opt/ros/humble/setup.bash

# Launch basic simulation
ros2 launch humanoid_control simulation.launch.py
```

### 3. Run VLA (Vision-Language-Action) Demo
```bash
cd ros2_ws
source install/setup.bash
source /opt/ros/humble/setup.bash

# Run voice command demo
ros2 run humanoid_vla voice_command_demo
```

## Key Components

### 1. Module 1: ROS2 Fundamentals
```bash
# Navigate to ROS2 examples
cd ros2_ws/src/humanoid_control/humanoid_control/examples

# Run basic publisher/subscriber example
python3 talker.py &
python3 listener.py
```

### 2. Module 2: Simulation
```bash
# Launch Gazebo simulation
ros2 launch humanoid_control gazebo_simulation.launch.py

# Launch Isaac Sim (requires NVIDIA Isaac Sim installation)
# Instructions in docs/module-2-simulation/chapter-1-gazebo-setup.md
```

### 3. Module 3: AI Perception
```bash
# Run SLAM demo
ros2 launch isaac_ros_vslam slam_demo.launch.py

# Run object detection
ros2 run isaac_ros_segmentation object_detection_node
```

### 4. Module 4: Vision-Language-Action
```bash
# Run complete VLA pipeline
ros2 launch humanoid_vla complete_pipeline.launch.py
```

## Verification Steps

### 1. Check ROS2 Installation
```bash
# Verify ROS2 installation
ros2 --version

# Check available nodes
ros2 node list
```

### 2. Test Basic Functionality
```bash
# Run basic test
cd ros2_ws
source install/setup.bash
source /opt/ros/humble/setup.bash
colcon test --packages-select humanoid_control
```

### 3. Verify Web Platform
- Open browser to `http://localhost:3000`
- Navigate through book modules
- Test code examples
- Verify RAG chatbot functionality

## Common Issues and Solutions

### ROS2 Environment Not Found
- Ensure you've sourced the ROS2 environment: `source /opt/ros/humble/setup.bash`
- Check if ROS2 is properly installed: `dpkg -l | grep ros-humble`

### Simulation Performance Issues
- Ensure NVIDIA GPU drivers are properly installed
- Check available RAM and disk space
- Reduce simulation complexity if needed

### Web Platform Build Failures
- Verify Node.js version (18+ required)
- Clear npm cache: `npm cache clean --force`
- Delete node_modules and reinstall: `rm -rf node_modules && npm install`

## Next Steps

1. Complete Module 1: ROS2 Architecture chapter
2. Set up your first ROS2 package
3. Run the basic publisher/subscriber example
4. Proceed to Module 2 for simulation setup