# TurtleBot3 Autonomy

A TurtleBot3 clone robot project focused on autonomous navigation using SLAM (Simultaneous Localization and Mapping).

## Prerequisites

Install ROS 2 Jazzy and dependencies on Ubuntu 24.04:

```bash
# Install ROS 2 Jazzy and essential packages
sudo apt update
sudo apt install -y ros-jazzy-desktop-full ros-jazzy-colcon-common-extensions

# Install TurtleBot3 and SLAM dependencies
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-joy \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-ros \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip

# Initialize rosdep (if not already done)
sudo rosdep init
rosdep update
```

## Quick Start

1. **Setup Environment**
   ```bash
   source /opt/ros/jazzy/setup.bash
   export TURTLEBOT3_MODEL=burger
   export LDS_MODEL=LDS-02
   cd ~/tb3_autonomy
   source install/setup.bash
   ```

2. **Build**
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **Launch SLAM**
   ```bash
   ros2 launch turtlebot3_bringup slam.launch.py
   ```

4. **Control Robot**
   ```bash
   # Xbox controller (TwistStamped)
   ros2 launch xbox_teleop_stamped xbox_teleop_stamped.launch.py
   
   # Xbox controller (legacy Twist)

## Hardware Requirements

- TurtleBot3 compatible robot hardware

- Xbox controller (optional)
- Ubuntu 24.04 with ROS 2 Jazzy

## ROS 2 Jazzy Compatible

Built and tested on ROS 2 Jazzy distribution on Ubuntu 24.04. 



