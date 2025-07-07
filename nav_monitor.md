# Navigation Monitoring and Trajectory Analysis

This guide provides commands to launch Navigation2 with the `burger_nav_basic.yaml` parameters and capture detailed navigation logs for trajectory analysis and optimization.

## Launch Navigation2 with Logging

**Terminal 1 - Main Navigation Launch:**
```bash
ros2 launch nav2_bringup bringup_launch.py \
    params_file:=/home/dingo/Programming/tb3_autonomy/src/turtlebot3/turtlebot3_bringup/param/burger_nav_basic.yaml \
    map:=/path/to/your/map.yaml \
    use_sim_time:=false 2>&1 | tee nav2_trajectory_analysis.log
```

## Comprehensive Data Collection

**Terminal 2 - Record Navigation Topics:**
```bash
ros2 bag record -o nav2_session /cmd_vel /local_costmap/costmap /global_costmap/costmap /plan /tf /tf_static /behavior_tree_log /dwb_planner/trajectory_cloud
```

**Terminal 3 - Monitor Trajectory Costs:**
```bash
ros2 topic echo /dwb_planner/cost_cloud | tee trajectory_costs.log
```

**Terminal 4 - Log Behavior Tree Decisions:**
```bash
ros2 topic echo /behavior_tree_log | tee bt_decisions.log
```

## What the Logs Reveal

The captured data will help identify trajectory optimization opportunities:

- **DWB trajectory costs** - Shows why specific paths are chosen or rejected, helps tune critic weights
- **Behavior tree decisions** - Reveals navigation state transitions and recovery behaviors
- **Costmap updates** - Shows obstacle detection patterns and inflation effects  
- **Path planning frequency** - Identifies planning bottlenecks and replanning triggers
- **Velocity commands** - Shows acceleration/deceleration patterns and smoothness

## Real-time Visualization

Your `burger_nav_basic.yaml` has Groot monitoring enabled (lines 81-83), so you can visualize the behavior tree in real-time:

1. Install Groot2: `sudo apt install ros-jazzy-groot`
2. Launch Groot: `ros2 run groot Groot`
3. Connect to ZMQ ports 1666/1667 as configured in the parameters

## Key Parameters for Trajectory Tuning

Based on the `burger_nav_basic.yaml` configuration, focus on these parameters for smoother trajectories:

### Controller Server (DWB Local Planner)
- `sim_time: 1.5` - Forward simulation time
- `vx_samples: 20` / `vtheta_samples: 40` - Trajectory sampling resolution
- `linear_granularity: 0.03` / `angular_granularity: 0.025` - Path discretization
- Critic scales: `PathAlign.scale: 160.0`, `GoalAlign.scale: 12.0`, etc.

### Velocity Limits
- `max_vel_x: 0.25` - Maximum forward velocity
- `max_vel_theta: 1.0` - Maximum angular velocity  
- `acc_lim_x: 3.0` / `acc_lim_theta: 3.2` - Acceleration limits

### Goal Tolerances
- `xy_goal_tolerance: 0.08` - Position tolerance
- `yaw_goal_tolerance: 0.15` - Orientation tolerance

## Analysis Workflow

1. Launch navigation with logging
2. Send navigation goals and observe behavior
3. Review logs for:
   - Frequent replanning events
   - High trajectory costs
   - Recovery behavior triggers
   - Velocity oscillations
4. Adjust parameters based on findings
5. Test and compare performance