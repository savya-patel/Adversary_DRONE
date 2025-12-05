# SLAM Customization Guide

Quick reference for customizable files in the `slam/` workspace.

## Core Launch & Configuration

| File | Purpose | Customizable |
|------|---------|--------------|
| `run_cartographer.sh` | Launches Cartographer + LiDAR + TF | Port, launch args |
| `run_rosboard.sh` | Starts web UI | Port (8888) |
| `src/slam_config/launch/cartographer_usb.launch.py` | Cartographer launch file | LiDAR params, TF pose, launch args |
| `src/slam_config/config/cartographer_2d_lidar.lua` | Cartographer 2D SLAM tuning | All parameters marked `[TUNE]` |

## Key Parameters by File

### `launch/cartographer_usb.launch.py`

**LiDAR node** (sick_tim_usb_ros2):
- `frame_id`: Laser reference frame (default: `laser_frame`)
- `publish_points`: Points across 270° (default: `270` ≈ 1°)
- `range_min` / `range_max`: Valid range (default: `0.1` / `10.0` m)
- `log_throttle_sec`: Log frequency (default: `2.0` s)

**Static transform** (base_link → laser_frame):
- Edit `--x --y --z` for position offset
- Edit `--qx --qy --qz --qw` for rotation (quaternion)

**Launch arguments**:
- `use_sim_time`: Simulation mode (default: `false`)
- `resolution`: Map grid size (default: `0.05` m)

### `config/cartographer_2d_lidar.lua`

All `[TUNE]` parameters affect mapping quality and speed. Common adjustments:

| Parameter | Effect | Range |
|-----------|--------|-------|
| `submap_publish_period_sec` | Submap update rate | Lower = faster |
| `motion_filter.max_distance_meters` | Scan insertion threshold | Lower = denser map |
| `motion_filter.max_angle_radians` | Rotation threshold (rad) | Lower = more scans |
| `submaps.num_range_data` | Scans per submap | Higher = larger submaps |
| `submaps.grid_options_2d.resolution` | Grid cell size (m) | Lower = finer detail |
| `ceres_scan_matcher.translation_weight` | XY motion weight | Higher = prefer translation |
| `ceres_scan_matcher.rotation_weight` | Rotation weight | Higher = prefer rotation |
| `POSE_GRAPH.optimize_every_n_nodes` | Loop closure cadence | Lower = more optimization |
| `POSE_GRAPH.constraint_builder.min_score` | Match quality threshold | 0.0–1.0, higher = stricter |
| `POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations` | Final refinement | Higher = better fit, slower |

## LiDAR Driver

| File | Purpose | Customizable |
|------|---------|--------------|
| `src/sick_tim_usb_ros2/sick_tim_usb_ros2/usb_lidar_node.py` | ROS 2 node publishing `/scan` | Parameters (see launch file) |
| `src/sick_tim_usb_ros2/sick_tim_usb_ros2/L1_lidar_usb.py` | USB/serial SOPAS driver | Connection port, baud rate |

**L1_lidar_usb.py**:
- `connect()`: Set USB vendor/product IDs or serial port
- `run()`: Parse SOPAS binary frames; adjust if protocol changes

## Quick Tuning Tips

**Faster mapping** (lower CPU cost):
- ↑ `motion_filter.max_distance_meters` (skip scans)
- ↓ `rangefinder_sampling_ratio` (0.5 = use 50% of scans)
- ↑ `optimize_every_n_nodes` (delay loop closure)

**Denser map** (slower, more accurate):
- ↓ `motion_filter.max_distance_meters` (insert more scans)
- ↑ `submaps.grid_options_2d.resolution` detail
- ↓ `submaps.num_range_data` (smaller submaps)

**Better loop closure**:
- ↓ `constraint_builder.min_score` (accept lower quality matches)
- ↓ `constraint_builder.max_constraint_distance` (search closer for loops)
- ↑ `POSE_GRAPH.loop_closure_rotation_weight` (favor rotation corrections)

## File Structure

```
slam/
├── run_cartographer.sh          ← Launch Cartographer
├── run_rosboard.sh              ← Launch web UI
├── SETUP.md                     ← Full setup guide
├── CUSTOMIZE.md                 ← This file
└── src/
    ├── slam_config/
    │   ├── launch/
    │   │   └── cartographer_usb.launch.py  ← [CUSTOMIZE] Launch params
    │   └── config/
    │       └── cartographer_2d_lidar.lua   ← [CUSTOMIZE] SLAM tuning
    └── sick_tim_usb_ros2/
        ├── sick_tim_usb_ros2/
        │   ├── usb_lidar_node.py           ← ROS 2 entry point
        │   └── L1_lidar_usb.py             ← USB/serial driver
        └── package.xml
```

---

**For detailed Cartographer reference**: See [official docs](https://google-cartographer-ros.readthedocs.io/).
