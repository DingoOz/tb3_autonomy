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
   # Standard SLAM
   ros2 launch turtlebot3_bringup slam.launch.py
   
   # PC-optimized SLAM (higher resolution, better performance)
   ros2 launch turtlebot3_bringup pc_slam.launch.py
   
   # Confined space SLAM (optimized for tight environments)
   ros2 launch turtlebot3_bringup pc_slam.launch.py slam_params_file:=src/turtlebot3/turtlebot3_bringup/param/mapper_params_confined.yaml
   ```

4. **Control Robot**
   ```bash
   # Xbox controller (TwistStamped)
   ros2 launch xbox_teleop_stamped xbox_teleop_stamped.launch.py
   
   # Xbox controller (legacy Twist)
   ros2 launch xbox_teleop xbox_teleop.launch.py
   ```

5. **Navigation**
   ```bash
   # Standard navigation
   ros2 launch turtlebot3_navigation2 navigation2.launch.py
   
   # Confined space navigation (optimized for tight areas)
   ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:=/home/dingo/Programming/tb3_autonomy/src/turtlebot3/turtlebot3_bringup/param/burger_confined_nav.yaml

   #tested
   ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:=/home/dingo/Programming/tb3_autonomy/src/turtlebot3/turtlebot3_bringup/param/burger_nav_basic.yaml
   ```

## Hardware Requirements

- TurtleBot3 compatible robot hardware

- Xbox controller (optional)
- Ubuntu 24.04 with ROS 2 Jazzy

## SLAM Configuration Options

### Standard SLAM
- **File**: `mapper_params_online_async.yaml`
- **Use case**: General purpose mapping
- **Resolution**: 5cm
- **Command**: `ros2 launch turtlebot3_bringup pc_slam.launch.py`

### Confined Space SLAM
- **File**: `mapper_params_confined.yaml`
- **Use case**: Tight, close-in environments (corridors, small rooms, cluttered spaces)
- **Resolution**: 2.5cm (higher detail)
- **Key features**:
  - More frequent map updates (2.0s vs 5.0s)
  - Higher sensitivity to small movements
  - Enhanced loop closure for tight areas
  - Aggressive scan matching for precision
  - Optimized for narrow passages and close obstacles
- **Command**: `ros2 launch turtlebot3_bringup pc_slam.launch.py slam_params_file:=src/turtlebot3/turtlebot3_bringup/param/mapper_params_confined.yaml`

### Performance vs Precision Trade-off
- **Standard**: Balanced performance and accuracy
- **Confined**: Higher precision, more CPU intensive, ideal for detailed indoor mapping

## ROS 2 Jazzy Compatible

Built and tested on ROS 2 Jazzy distribution on Ubuntu 24.04.
