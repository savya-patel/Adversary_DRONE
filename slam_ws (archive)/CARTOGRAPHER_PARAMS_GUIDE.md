# Cartographer Parameter Selection Guide

## Quick Start Commands

### Clean Restart Everything
```bash
cd ~/Adversary_DRONE/slam_ws
./clean_restart.sh
```
This will:
- Stop all Cartographer/LiDAR processes
- Optionally clear old maps
- Optionally archive logs
- Show launch options

### Launch Options

#### 1. **Standard Launch** (Recommended)
```bash
./run_cartographer.sh
```
- Output to terminal
- No logging
- Clean visualization

#### 2. **With Logging** (For Debugging)
```bash
./run_cartographer_log.sh
```
- Saves full log to `logs/cartographer_YYYYMMDD_HHMMSS.log`
- Analyze with: `./analyze_cartographer_log.sh`

#### 3. **Web Browser Visualization**
Terminal 1:
```bash
./run_cartographer.sh
```

Terminal 2:
```bash
./run_rosboard_sudo.sh
```

Browser: `http://10.250.240.81:8888`

---

## Common Parameter Adjustments

### Scenario 1: Robot Moving Slowly (< 0.5 m/s)

**File:** `src/slam_config/config/cartographer_2d_lidar.lua`

**Adjust:**
```lua
-- Make motion filter more sensitive
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.005  -- 5mm (was 0.01)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 0.1° (was 0.2)
```

### Scenario 2: Robot Moving Fast (> 1 m/s)

**Adjust:**
```lua
-- Less sensitive motion filter
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- 5cm
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- 1.0°

-- Faster scan matching
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30  -- (was 50)
```

### Scenario 3: Need Higher Resolution Map

**Adjust:**
```lua
-- Finer grid
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025  -- 2.5cm (was 0.05)

-- More scans per submap
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 180  -- 9 seconds (was 90)
```

### Scenario 4: Need Faster Processing (Lower CPU)

**Adjust:**
```lua
-- Coarser grid
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.10  -- 10cm

-- Fewer iterations
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- Less frequent optimization
POSE_GRAPH.optimize_every_n_nodes = 90  -- (was 45)

-- Reduce threads
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2  -- (was 4)
```

### Scenario 5: Poor Loop Closure

**Adjust:**
```lua
-- More aggressive loop closure
POSE_GRAPH.constraint_builder.min_score = 0.45  -- Accept lower scores (was 0.50)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.7  -- Check more nodes (was 0.5)
POSE_GRAPH.constraint_builder.max_constraint_distance = 30.0  -- Larger search (was 20.0)
```

### Scenario 6: Lots of Drift

**Adjust:**
```lua
-- Optimize more frequently
POSE_GRAPH.optimize_every_n_nodes = 30  -- Every 1.5 sec (was 45)

-- Stronger constraints
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e6  -- (was 1e5)
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e6  -- (was 1e5)
```

---

## After Changing Parameters

**ALWAYS rebuild:**
```bash
cd ~/Adversary_DRONE/slam_ws
colcon build --packages-select slam_config --symlink-install
```

**Then test:**
```bash
./run_cartographer_log.sh
# Rotate/move the robot
# Ctrl+C to stop
./analyze_cartographer_log.sh
```

---

## Key Parameters Reference

### Motion Filter (What gets processed)
```lua
max_time_seconds = 0.5          # Process scan if 0.5s elapsed
max_distance_meters = 0.01      # Or moved 1cm
max_angle_radians = rad(0.2)    # Or rotated 0.2°
```

### Scan Matcher (Real-time matching)
```lua
occupied_space_weight = 10.0    # Feature matching importance
translation_weight = 1.0        # Translation penalty
rotation_weight = 100.0         # Rotation penalty (higher = prioritize)
max_num_iterations = 50         # Optimization iterations
```

### Submaps (Map building blocks)
```lua
num_range_data = 90             # Scans per submap (90 = 4.5s at 20Hz)
resolution = 0.05               # Grid size (5cm)
```

### Loop Closure (Drift correction)
```lua
optimize_every_n_nodes = 45     # Optimize every 2.25 seconds
min_score = 0.50                # Minimum match quality (0-1)
sampling_ratio = 0.5            # Check 50% of previous nodes
max_constraint_distance = 20.0  # Search radius (meters)
```

---

## Current "Best" Configuration

**Use Case:** Rotating in place, stationary testing

**Settings:**
- Motion filter: Sensitive (0.01m, 0.2°)
- Rotation priority: HIGH (weight=100)
- Translation priority: LOW (weight=1)
- Optimize: Every 2.25 seconds (45 nodes)
- Resolution: 5cm
- Match threshold: 50%

**Location:** `slam_ws/src/slam_config/config/cartographer_2d_lidar.lua`

---

## Troubleshooting

### High drift?
→ Increase `local_slam_pose_*_weight`
→ Decrease `optimize_every_n_nodes` (optimize more often)

### Low match scores?
→ Decrease `min_score` threshold
→ Increase `sampling_ratio`

### Too slow?
→ Increase `resolution` (coarser grid)
→ Decrease `max_num_iterations`
→ Decrease `num_threads`

### Map not building on rotation?
→ Increase `rotation_weight`
→ Decrease `max_angle_radians` (motion filter)

### Creating too many submaps?
→ Increase `num_range_data`
→ Adjust motion filter to be less sensitive

---

## Save Configuration

After finding good parameters, document them:

```bash
cp src/slam_config/config/cartographer_2d_lidar.lua \
   src/slam_config/config/cartographer_2d_lidar_BACKUP_$(date +%Y%m%d).lua
```

Or commit to git:
```bash
cd ~/Adversary_DRONE
git add slam_ws/src/slam_config/config/cartographer_2d_lidar.lua
git commit -m "Updated Cartographer params for [use case]"
git push
```
