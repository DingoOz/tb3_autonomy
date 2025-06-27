# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a TurtleBot3 clone robot autonomy project focused on SLAM (Simultaneous Localization and Mapping) and navigation capabilities. The project is built on ROS 2 Jazzy and uses the official TurtleBot3 packages as a foundation with custom modifications for autonomy features.

## Build System

This is a ROS 2 workspace using the Colcon build system. The source code is located in `src/` with the main TurtleBot3 packages in `src/turtlebot3/`.

### Common Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with symlink install (for Python packages)
colcon build --symlink-install

# Clean build
rm -rf build/ install/ log/
colcon build
```

### Environment Setup

```bash
# Source the workspace (must be done in each terminal)
source install/setup.bash

# Required environment variables
export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
export LDS_MODEL=LDS-02         # for LD08 lidar
```

## Key Launch Files

### Robot Hardware
- `ros2 launch turtlebot3_bringup robot.launch.py` - Basic robot bringup (hardware + lidar)
- `ros2 launch turtlebot3_bringup dingo.robot.launch.py` - Custom robot variant

### SLAM and Mapping
- `ros2 launch turtlebot3_bringup slam.launch.py` - SLAM with slam_toolbox (primary autonomy feature)

### Teleoperation
- `ros2 launch xbox_teleop.launch.py` - Xbox controller teleop (Twist messages)
- `ros2 launch xbox_teleop_stamped.launch.py` - Xbox controller teleop (TwistStamped messages)
- `ros2 run turtlebot3_teleop teleop_keyboard` - Keyboard teleop

### Visualization  
- `ros2 launch turtlebot3_bringup rviz2.launch.py` - RViz2 visualization

## Architecture Notes

### Message Type Migration
The codebase has been migrated from `geometry_msgs/Twist` to `geometry_msgs/TwistStamped` for `cmd_vel` topics. This is documented in `stamped_changes.md`. All velocity commands now include timestamps and frame references for better safety and synchronization.

### SLAM Implementation
The SLAM system uses slam_toolbox with custom configuration:
- Asynchronous mapping mode with online processing
- Custom parameter file: `param/mapper_params_online_async.yaml`
- Automatic lifecycle management (configure + activate) with timed delays
- Frame transforms from `base_scan` to `laser` for sensor compatibility

### Hardware Integration
- Supports LDS-01 and LDS-02 lidar sensors (configurable via LDS_MODEL)
- Robot hardware communication via `/dev/ttyACM0` (OpenCR board)
- Lidar communication via `/dev/ttyUSB0`
- Xbox controller support via `/dev/input/js0`

### Custom Modifications
- Modified TurtleBot3 parameter files to enable TwistStamped by default
- Custom launch files for different robot variants ("dingo" configuration)
- Enhanced teleop with both standard and stamped message support

## Development Workflow

1. Make code changes in `src/` directory
2. Build with `colcon build --symlink-install` (for Python) or `colcon build` (for C++)
3. Source the workspace: `source install/setup.bash`
4. Test launch files and functionality
5. The project uses git on the `feat/slam-nav2` branch

## Testing and Debugging

- Check `log/` directory for build and runtime logs
- Use `ros2 topic list` and `ros2 topic echo` for debugging message flow
- RViz2 for visual debugging of SLAM and navigation
- Frame visualization available as PDF files in root directory

## Dependencies

The project relies on standard TurtleBot3 ROS 2 packages plus:
- slam_toolbox (for SLAM functionality)
- ld08_driver (for LDS-02 lidar support)
- teleop_twist_joy (for controller input)
- tf2_ros (for coordinate transforms)