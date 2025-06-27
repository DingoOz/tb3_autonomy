# TwistStamped Migration Changes

This document describes the changes made to migrate from `geometry_msgs/Twist` to `geometry_msgs/TwistStamped` for the `cmd_vel` topic throughout the TurtleBot3 codebase.

## Overview

The migration ensures that all velocity commands published to the `cmd_vel` topic now use `TwistStamped` messages instead of `Twist` messages. This provides better timestamp information and frame reference for velocity commands, improving safety and synchronization.

## Files Modified

### C++ Code
- **`src/turtlebot3/turtlebot3_node/include/turtlebot3_node/twist_subscriber.hpp`**
  - Changed default parameter `enable_stamped_cmd_vel` from `false` to `true`
  - The TwistSubscriber class already supported both message types, now defaults to TwistStamped

### Python Teleop
- **`src/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py`**
  - Removed conditional logic based on ROS_DISTRO
  - Now always uses `TwistStamped` for cmd_vel publishing
  - Added proper header.stamp and header.frame_id to messages

### Python Examples
- **`src/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_obstacle_detection/turtlebot3_obstacle_detection.py`**
  - Updated publisher to use `TwistStamped`
  - Added timestamp and frame_id ('base_link') to published messages
  
- **`src/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_patrol/turtlebot3_patrol_server.py`**
  - Updated publisher to use `TwistStamped`
  - Modified all velocity publishing methods to use TwistStamped with proper headers
  
- **`src/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_absolute_move/turtlebot3_absolute_move.py`**
  - Removed conditional import based on ROS_DISTRO
  - Updated all velocity commands to use TwistStamped.twist fields
  - Added proper header information to all published messages
  
- **`src/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_relative_move/turtlebot3_relative_move.py`**
  - Removed conditional import and publishing logic
  - Updated path generation methods to use TwistStamped
  - Added proper header information to all messages
  
- **`src/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_interactive_marker/turtlebot3_interactive_marker.py`**
  - Updated to use TwistStamped instead of Twist
  - Modified velocity publishing to include proper headers

### Parameter Files
Updated all parameter files to enable TwistStamped by default:
- `src/turtlebot3/turtlebot3_bringup/param/humble/burger.yaml`
- `src/turtlebot3/turtlebot3_bringup/param/humble/waffle.yaml`
- `src/turtlebot3/turtlebot3_bringup/param/humble/waffle_pi.yaml`
- `src/turtlebot3/turtlebot3_bringup/param/jazzy/burger.yaml`

## Key Changes Summary

1. **Message Type**: All `cmd_vel` publishers now use `geometry_msgs/TwistStamped` instead of `geometry_msgs/Twist`

2. **Headers**: All TwistStamped messages include:
   - `header.stamp`: Current timestamp using `get_clock().now().to_msg()`
   - `header.frame_id`: Set to 'base_link' for robot-relative velocity commands

3. **Backwards Compatibility**: The C++ TwistSubscriber maintains compatibility by supporting both message types, but now defaults to TwistStamped

4. **Parameter Default**: The `enable_stamped_cmd_vel` parameter now defaults to `true` instead of `false`

## Benefits

- **Timestamp Safety**: Commands now include timestamps for better stale data detection
- **Frame Reference**: Explicit frame_id eliminates ambiguity about coordinate frames
- **Future Compatibility**: Aligns with ROS 2 best practices and newer distributions
- **Network Reliability**: Better handling of command latency over network connections

## Testing

After implementing these changes, the system should be tested to ensure:
- All nodes start successfully
- Velocity commands are properly received and executed
- No performance degradation
- Compatibility across different ROS 2 distributions

The TurtleBot3 hardware node already supported TwistStamped messages, so no firmware changes were required.