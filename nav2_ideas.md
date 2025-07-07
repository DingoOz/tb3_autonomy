# Nav2 Parameter Optimization Ideas

Based on analysis of `burger_nav_basic.yaml` parameters to address navigation issues: delay before motion, haphazard/non-smooth motion, and difficulty completing final approach to goal.

## Key Parameter Changes Needed:

### 1. **Local Costmap Resolution** (line 179):
- Current: `resolution: 0.1` (10cm cells)
- **Recommend: `resolution: 0.05`** (5cm cells) for higher resolution

### 2. **Robot Size for Collision Checking**:
- **Local costmap** (line 180): Current `robot_radius: 0.05` is too small
- **Recommend: `robot_radius: 0.12`** (TurtleBot3 Burger is ~0.105m radius)
- **Global costmap** (line 228): Current `robot_radius: 0.1` 
- **Recommend: `robot_radius: 0.12`** for consistency

### 3. **Path Planning Simplification**:
- **Planner tolerance** (line 294): Current `tolerance: 0.5` is too loose
- **Recommend: `tolerance: 0.2`** for more precise paths
- **Enable A*** (line 295): Current `use_astar: false`
- **Recommend: `use_astar: true`** for better paths

### 4. **Goal Tolerance Issues** (fixing final approach problems):
- **Controller goal tolerance** (line 122): `xy_goal_tolerance: 0.03` is very tight
- **Recommend: `xy_goal_tolerance: 0.08`**
- **DWB goal tolerance** (line 150): `xy_goal_tolerance: 0.05`
- **Recommend: `xy_goal_tolerance: 0.08`** (match controller)

### 5. **Motion Smoothness** (addressing haphazard movement):
- **Velocity samples** (line 143): `vx_samples: 30` is high
- **Recommend: `vx_samples: 20`**
- **Angular samples** (line 145): `vtheta_samples: 60` is very high
- **Recommend: `vtheta_samples: 40`**
- **Simulation time** (line 146): `sim_time: 1.0` might be too short
- **Recommend: `sim_time: 1.5`**

### 6. **Reduce Initial Delay**:
- **Update frequencies**: Increase local costmap update rate (line 172)
- **Current: `update_frequency: 1.0`**
- **Recommend: `update_frequency: 5.0`**

### 7. **Inflation Parameters** (making robot appear larger):
- **Local inflation** (line 184): `inflation_radius: 0.5` is good
- **Global inflation** (line 273): `inflation_radius: 0.1` is too small
- **Recommend: `inflation_radius: 0.3`**

## Summary
These changes will provide:
- Higher resolution local planning
- Safer collision avoidance with larger apparent robot size
- Simplified path planning with A*
- Reduced computational overhead
- Improved goal approach behavior
- Smoother motion with better sampling parameters