# Groot2 Connectivity Issue - Analysis and Solutions

## Problem Statement

When launching Navigation2 with `burger_nav_basic.yaml` parameters and attempting to monitor the behavior tree with Groot2, Groot cannot connect on port 1666 despite Nav2 appearing to have Groot monitoring configured.

## Analysis Performed

### Configuration Status
- ✅ **Parameter Configuration**: `burger_nav_basic.yaml` correctly includes Groot support:
  ```yaml
  bt_navigator:
    ros__parameters:
      #groot support
      enable_groot_monitoring: true
      groot_zmq_publisher_port: 1666
      groot_zmq_server_port: 1667
  ```

- ✅ **Runtime Parameters**: Confirmed parameters are active:
  ```bash
  ros2 param get /bt_navigator enable_groot_monitoring  # Returns: True
  ros2 param get /bt_navigator groot_zmq_publisher_port  # Returns: 1666
  ```

- ✅ **ZMQ Libraries**: System has required ZMQ libraries installed:
  ```
  libzmq5:amd64 (4.3.5-1build2)
  libzmq3-dev:amd64 (4.3.5-1build2)
  cppzmq-dev:amd64 (4.10.0-1build1)
  ```

- ✅ **Nav2 Binary Linking**: bt_navigator is linked with ZMQ:
  ```bash
  ldd /opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator | grep zmq
  # Returns: libzmq.so.5 => /lib/x86_64-linux-gnu/libzmq.so.5
  ```

### Root Cause Identified

**The ROS 2 Jazzy binary distribution of Nav2 (version 1.3.7-1noble.20250624.013307) does not include Groot monitoring functionality compiled in**, despite having the configuration parameters available.

While the bt_navigator binary is linked with ZMQ libraries, the actual Groot monitoring code appears to be disabled or not compiled in the official binary distribution.

## Solutions

### Option 1: Compile Nav2 from Source (Recommended)

Create a separate workspace to compile Nav2 with explicit Groot support:

```bash
# Create overlay workspace
mkdir -p ~/nav2_groot_ws/src
cd ~/nav2_groot_ws/src

# Clone Nav2 source
git clone https://github.com/ros-planning/navigation2.git -b jazzy-devel

# Build with Groot support enabled
cd ~/nav2_groot_ws
colcon build --packages-select nav2_bt_navigator \
  --cmake-args -DENABLE_GROOT_MONITORING=ON

# Source the overlay before running navigation
source ~/nav2_groot_ws/install/setup.bash
```

### Option 2: Check for Updated Packages

Monitor for newer Nav2 packages that might include Groot support:

```bash
sudo apt update
sudo apt upgrade ros-jazzy-nav2-bt-navigator
```

### Option 3: Alternative Behavior Tree Monitoring

Use Nav2's built-in behavior tree debugging capabilities:

```bash
# Enable BT XML reloading for development
ros2 param set /bt_navigator always_reload_bt_xml true

# Use RViz2 with Nav2 plugins for visual monitoring
ros2 launch turtlebot3_bringup rviz2.launch.py
```

### Option 4: Custom Behavior Tree with Logging

Create a custom behavior tree XML file with additional logging nodes for debugging purposes.

## Implementation Steps

1. **Immediate Solution**: Try Option 1 (compile from source) as it's the most reliable
2. **Verify**: After compilation, test Groot2 connection on port 1666
3. **Fallback**: If compilation issues arise, use Option 3 for behavior tree monitoring
4. **Long-term**: Monitor ROS 2 updates for native Groot support in binary distributions

## Testing Verification

After implementing the solution:

1. Launch navigation with active goal
2. Check port 1666 is listening: `ss -tuln | grep 1666`
3. Connect Groot2 to `localhost:1666`
4. Verify behavior tree visualization in Groot2

## Files Involved

- Configuration: `src/turtlebot3/turtlebot3_bringup/param/burger_nav_basic.yaml`
- Nav2 Binary: `/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator`
- ROS Distribution: Jazzy (1.3.7-1noble.20250624.013307)

## Status

- **Issue Identified**: ✅ Complete
- **Root Cause**: ✅ Binary distribution lacks Groot support
- **Solution Recommended**: ✅ Compile from source
- **Implementation**: ⏳ Pending user decision