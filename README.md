# TurtleBot3 Autonomy

A TurtleBot3 clone robot project focused on autonomous navigation using SLAM (Simultaneous Localization and Mapping).

## Quick Start

1. **Setup Environment**
   ```bash
   export TURTLEBOT3_MODEL=burger
   export LDS_MODEL=LDS-02
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
   # Xbox controller
   ros2 launch xbox_teleop.launch.py
   
   # Keyboard
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

## Features

- **SLAM Mapping**: Real-time mapping using slam_toolbox
- **Multiple Teleop Options**: Xbox controller and keyboard control
- **TwistStamped Support**: Enhanced velocity commands with timestamps
- **RViz Visualization**: Real-time robot and map visualization

## Hardware Requirements

- TurtleBot3 compatible robot hardware
- LDS-02 lidar sensor
- Xbox controller (optional)

## ROS 2 Jazzy Compatible

Built and tested on ROS 2 Jazzy distribution. 
