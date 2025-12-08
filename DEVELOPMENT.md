# Development Environment Setup

This document provides detailed instructions for setting up the development environment for the Physical AI & Humanoid Robotics Book project.

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

## Complete Setup Process

### 1. Install System Dependencies

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install curl wget git build-essential cmake python3-dev python3-pip -y
```

### 2. Install ROS2 Humble Hawksbill

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base -y
sudo apt install python3-rosdep2 python3-colcon-common-extensions python3-rosinstall -y
```

### 3. Setup ROS2 Environment

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Add to bashrc for persistent setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 4. Install Python Dependencies

```bash
# Install project-specific Python dependencies
pip3 install -r requirements.txt
```

### 5. Setup Web Platform (Docusaurus)

```bash
# Navigate to web platform directory
cd my-book

# Install Node.js dependencies
npm install

# Verify installation
npm run build
```

### 6. Setup ROS2 Workspace

```bash
# Navigate to ROS2 workspace
cd ros2_ws

# Create source directory if not exists
mkdir -p src

# Source ROS2 before building
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --packages-select humanoid_control isaac_ros_vslam
```

### 7. Environment Configuration

```bash
# Create environment file from example
cp .env.example .env

# Edit .env with your specific configuration
nano .env  # or use your preferred editor
```

## Docker Setup (Optional)

If you prefer to use Docker for development, you can use the provided Docker configuration:

```bash
# Build the development container
docker build -f Dockerfile.dev -t humanoid-robotics-book:dev .

# Run the development container
docker run -it --gpus all --network host -v $(pwd):/workspace humanoid-robotics-book:dev
```

## Verification Steps

### 1. Check ROS2 Installation
```bash
# Verify ROS2 installation
ros2 --version

# Check available nodes
ros2 node list

# Check available topics
ros2 topic list
```

### 2. Test Basic Functionality
```bash
# Navigate to ROS2 workspace
cd ros2_ws
source install/setup.bash
source /opt/ros/humble/setup.bash

# Run basic test
colcon test --packages-select humanoid_control
```

### 3. Verify Web Platform
```bash
# Start web platform
cd my-book
npm start

# The platform should be available at http://localhost:3000
```

## Troubleshooting

### Common Issues

1. **Permission Denied for ROS2 Commands**
   - Make sure to source the ROS2 environment: `source /opt/ros/humble/setup.bash`
   - Check if ROS2 is properly installed: `dpkg -l | grep ros-humble`

2. **Node.js Memory Issues**
   - Increase Node.js memory: `export NODE_OPTIONS="--max-old-space-size=4096"`
   - Or add to your .bashrc for persistent setting

3. **Python Package Installation Issues**
   - Use virtual environment: `python3 -m venv venv && source venv/bin/activate`
   - Install packages in virtual environment: `pip install -r requirements.txt`

4. **Simulation Performance Issues**
   - Ensure NVIDIA GPU drivers are properly installed
   - Check available RAM and disk space
   - Reduce simulation complexity if needed

## Development Workflow

### Building and Testing
1. Make changes to your code
2. Build ROS2 packages: `cd ros2_ws && colcon build`
3. Test changes in simulation
4. Update documentation if needed
5. Commit changes with descriptive commit messages

### Code Standards
- Follow ROS2 coding standards for Python and C++ code
- Use Docusaurus standards for Markdown content
- Maintain consistent formatting across all files
- Write clear, descriptive comments and documentation

## Next Steps

After completing the environment setup:
1. Review the project architecture in the [plan.md](specs/001-humanoid-robotics-book/plan.md)
2. Start with Module 1 content in the [docs directory](my-book/docs/)
3. Explore the ROS2 workspace and run basic examples
4. Set up your development environment for content creation