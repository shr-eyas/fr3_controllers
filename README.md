# Franka ROS 2 Installation Guide

This guide provides step-by-step instructions to install and build the Franka ROS 2 workspace.

## Prerequisites
Ensure you have **ROS 2 Humble** installed on your system. If not, refer to the official ROS 2 installation guide: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

## Installation Steps
Follow the commands below to install and build the workspace:

### 1. Install ROS Development Tools
```bash
sudo apt install ros-dev-tools
```

### 2. Source ROS 2 Setup
```bash
source /opt/ros/humble/setup.sh
```

### 3. Create the Workspace and Clone the Repository
```bash
mkdir -p ~/franka_ros2_ws/src
cd ~/franka_ros2_ws/src
git clone https://github.com/frankaemika/franka_ros2.git
```

### 4. Import Dependencies
```bash
vcs import franka_ros2 < franka_ros2/franka.repos --recursive --skip-existing
```

### 5. Navigate to the Workspace Root
```bash
cd ..
```

### 6. Install Dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

### 7. Build the Workspace
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 8. Clone Controllers Repository (FR3 Controllers)
```bash
cd ~/franka_ros2_ws/src
git clone https://github.com/shr-eyas/fr3_controllers.git
```

### 9. Build the Workspace
```bash
cd ..
colcon build 
```

### 10. Source the Workspace Setup
```bash
source ~/franka_ros2_ws/install/setup.bash
```
