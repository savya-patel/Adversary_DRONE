# slam_config

Configuration and launch files for 2D SLAM using Cartographer with a SICK TiM series LiDAR on ROS 2 Humble.

## Usage

Build and run from the `slam/` workspace:

```bash
cd /home/eagle/Adversary_DRONE/slam
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_config sick_tim_usb_ros2 --symlink-install
source install/setup.bash
./run_cartographer.sh
```

This launches:
- `sick_tim_usb_ros2/usb_lidar_node` publishing `/scan`.
- A static transform `base_link` -> `laser_frame`.
- `cartographer_ros` (2D) and the occupancy grid node publishing `/map`.

## Adjustables (recommended)

- Launch arguments (at runtime):
  - `use_sim_time` (bool): Use simulation clock. Default: `false`.
  - `resolution` (float): Occupancy grid resolution (m/cell). Default: `0.05`.

- LiDAR publisher parameters (`sick_tim_usb_ros2/usb_lidar_node`):
  - `frame_id` (string): Laser frame. Default: `laser_frame`.
  - `publish_points` (int): Points published across 270°. 270 ≈ 1° increment. Default: `270`.
  - `range_min` (float): Minimum valid range (m). Default: `0.1`.
  - `range_max` (float): Maximum valid range (m). Default: `10.0`.
  - `log_throttle_sec` (float): Throttled info logging period. Default: `2.0`.

- Static transform (`base_link` -> `laser_frame`):
  - Adjust `--x --y --z --qx --qy --qz --qw` in `launch/cartographer_usb.launch.py` to match your mounting.

- Cartographer configuration (`config/cartographer_2d_lidar.lua`): Key parameters to tune for your environment and platform:
  - `TRAJECTORY_BUILDER_2D.min_range`, `max_range`, `missing_data_ray_length`
  - `TRAJECTORY_BUILDER_2D.motion_filter.*`
  - `TRAJECTORY_BUILDER_2D.submaps.*` (e.g., `resolution`, `num_range_data`)
  - `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.*` weights
  - `POSE_GRAPH.*` (constraint builder thresholds, optimization cadence)

## Notes

- Downsampling: The TiM series often streams at ~0.333° native resolution. The node publishes ~1° by downsampling to `publish_points` across 270°.
- Odometry is not required. Cartographer performs scan matching; `provide_odom_frame=true` publishes `odom -> base_link`.

## File Inventory

- `launch/cartographer_usb.launch.py`: Launches LiDAR node, TF, and Cartographer.
- `config/cartographer_2d_lidar.lua`: Cartographer 2D configuration tuned for the TiM.
- `rviz/slam_tim561.rviz`: Optional RViz view for `/scan` and `/map`.

Legacy (not used by Cartographer):
- `config/mapper_params_online_async*.yaml`: SLAM Toolbox configs. Safe to remove if not using SLAM Toolbox.
