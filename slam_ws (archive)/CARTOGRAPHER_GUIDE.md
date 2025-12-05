# Google Cartographer SLAM Setup

**Google Cartographer** is Google's open-source SLAM solution that **works perfectly without odometry** using only laser scan data. Unlike slam_toolbox which has TF message filter issues on ROS 2 Humble, Cartographer runs smoothly on the Jetson with the SICK TiM561 LiDAR.

## Why Cartographer?

✅ **No Odometry Required** - Pure lidar scan matching  
✅ **No TF Message Filter Bugs** - Handles high-frequency (20 Hz) scans perfectly  
✅ **Google-Grade Quality** - Used in Google's mapping products  
✅ **Real-Time Loop Closure** - Automatically corrects drift  
✅ **Optimized for Embedded** - Runs efficiently on Jetson Orin Nano  

---

## Quick Launch

```bash
cd ~/Adversary_DRONE/slam_ws
./run_cartographer.sh
```

---

## What Cartographer Does

1. **Subscribes to:** `/scan` (sensor_msgs/LaserScan from USB LiDAR)
2. **Publishes:**
   - `/map` (nav_msgs/OccupancyGrid) - the 2D map
   - `/odom` (nav_msgs/Odometry) - odometry from scan matching
   - TF: `map → odom` and `odom → base_link`
3. **Scan Matching:** Uses Ceres Solver for high-accuracy pose estimation
4. **Loop Closure:** Automatically detects when you revisit an area and corrects drift

---

## Configuration File

**Location:** `slam_ws/src/slam_config/config/cartographer_2d_lidar.lua`

### Key Parameters (already tuned for TIM561):

```lua
-- LiDAR range limits
TRAJECTORY_BUILDER_2D.min_range = 0.1  -- 10 cm minimum
TRAJECTORY_BUILDER_2D.max_range = 10.0 -- 10 m maximum

-- No odometry, no IMU
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Map resolution
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5 cm per pixel

-- Scan frequency handling (20 Hz lidar)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- ~4.5 seconds of data per submap

-- Motion filter (only process scans when robot moves)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02  -- 2 cm movement
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)  -- 0.5° rotation
```

### When to Adjust:

**If robot moves slowly:**
- Decrease `motion_filter.max_distance_meters` to 0.01
- Decrease `max_angle_radians` to 0.25°

**If you want higher resolution maps:**
- Decrease `grid_options_2d.resolution` to 0.025 (2.5 cm)
- **Warning:** Uses more memory and CPU

**If experiencing drift in large areas:**
- Increase `constraint_builder.max_constraint_distance` from 15.0 to 25.0
- Increase `global_constraint_search_after_n_seconds` from 10 to 20

---

## Frame Setup

Cartographer expects this TF tree:

```
map (published by cartographer)
 └─ odom (published by cartographer from scan matching)
     └─ base_link (robot center)
         └─ laser_frame (lidar sensor)
```

The launch file handles this automatically:
1. **Static TF:** `base_link → laser_frame` (0.1m above base)
2. **Cartographer publishes:** `map → odom` and `odom → base_link`

---

## Visualization in RViz

1. Launch Cartographer: `./run_cartographer.sh`
2. In another terminal:
   ```bash
   rviz2
   ```
3. Add displays:
   - **Fixed Frame:** `map`
   - **LaserScan:** topic `/scan`
   - **Map:** topic `/map`
   - **TF:** Show all frames

---

## Topics Published

| Topic | Type | Hz | Description |
|-------|------|-----|-------------|
| `/scan` | sensor_msgs/LaserScan | 20 | Raw lidar data (INPUT) |
| `/map` | nav_msgs/OccupancyGrid | ~1 | 2D occupancy grid map |
| `/odom` | nav_msgs/Odometry | 200 | Pose from scan matching |
| `/submap_list` | cartographer_ros_msgs/SubmapList | 1 | Active submaps |
| `/tracked_pose` | geometry_msgs/PoseStamped | 200 | Current robot pose |
| `/trajectory_node_list` | visualization_msgs/MarkerArray | 1 | SLAM graph nodes |

---

## Saving Maps

Once you've built a map, save it:

```bash
# Save map to file
ros2 run nav2_map_server map_saver_cli -f my_map

# This creates:
# - my_map.pgm (image)
# - my_map.yaml (metadata)
```

Or use Cartographer's state serialization:

```bash
# Finish trajectory and save full state
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/eagle/map.pbstream'}"
```

---

## Troubleshooting

### "Odom frame does not exist"
- **Cause:** Cartographer hasn't received enough scans yet (needs ~2 seconds)
- **Fix:** Wait a few seconds for initialization

### Map looks distorted
- **Cause:** Robot moved too fast for scan matching
- **Fix:** Move robot slower, especially during turns

### High CPU usage
- **Tune:** Reduce `optimize_every_n_nodes` from 90 to 180
- **Reduce:** `num_threads` in `ceres_solver_options`

### No loop closures detected
- **Increase:** `constraint_builder.sampling_ratio` from 0.3 to 0.5
- **Decrease:** `constraint_builder.min_score` from 0.55 to 0.50

---

## Comparison: Cartographer vs SLAM Toolbox

| Feature | Cartographer | SLAM Toolbox (Humble) |
|---------|--------------|----------------------|
| Odometry Required | ❌ No | ⚠️ Optional but buggy |
| TF Message Filter | ✅ No issues | ❌ Drops all scans |
| Loop Closure | ✅ Excellent | ✅ Good |
| CPU Usage | Medium | Low |
| Memory Usage | Medium | Low |
| ROS 2 Humble | ✅ Works great | ❌ Message filter bug |
| Maturity | ✅ Industry proven | ✅ Good |
| **Recommendation** | **✅ USE THIS** | ⚠️ Wait for fix |

---

## Advanced: Running with rf2o Odometry

If you want to add wheel odometry or laser-based odometry later:

1. Set `use_odometry = true` in the `.lua` file
2. Launch rf2o alongside Cartographer
3. Cartographer will fuse scan matching with odometry for better accuracy

**But for now, pure lidar SLAM works perfectly!**

---

## Files Overview

```
slam_ws/
├── run_cartographer.sh              # Quick launch script
├── src/slam_config/
│   ├── launch/
│   │   └── cartographer_usb.launch.py   # Main launch file
│   └── config/
│       └── cartographer_2d_lidar.lua    # Cartographer config
└── src/sick_tim_usb_ros2/           # LiDAR driver (already working)
```

---

## References

- [Cartographer Documentation](https://google-cartographer-ros.readthedocs.io/)
- [Cartographer ROS Integration](https://github.com/cartographer-project/cartographer_ros)
- [Configuration Reference](https://google-cartographer.readthedocs.io/en/latest/configuration.html)

---

**Ready to map!** 🗺️ Just run `./run_cartographer.sh` and start moving your robot.
