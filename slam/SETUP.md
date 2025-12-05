# Steps to Make a New ROS 2 Workspace with LiDAR and SLAM

**NOTE:** Password for sudo is `Capstone25`

---

## Setup

### Create minimal workspace

```bash
cd ~/Adversary_DRONE
mkdir -p slam/src
cd slam
```

### Copy LiDAR driver and SLAM configuration packages

```bash
# Copy the LiDAR driver package
cp -r ../slam_ws/src/sick_tim_usb_ros2 src/

# Copy the SLAM configuration package (Cartographer + launch files)
cp -r ../slam_ws/src/slam_config src/
```

### Source ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### Install system and Python dependencies

```bash
# Update package lists
sudo apt update

# Install build tools
sudo apt install -y python3-colcon-common-extensions python3-pip

# Install Python packages
python3 -m pip install --user pyserial pyusb numpy
```

### Install ROS package dependencies

```bash
# Initialize rosdep (safe to run even if already initialized)
sudo rosdep init 2>/dev/null || true

# Update rosdep database
rosdep update

# Install all dependencies from src/
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build and Test

### Build the driver and SLAM config packages

```bash
colcon build --symlink-install \
  --packages-select sick_tim_usb_ros2 slam_config
```

### Source the built overlay

```bash
source install/setup.bash
```

### Test the LiDAR driver (requires hardware connected)

```bash
# Run the USB LiDAR node directly
ros2 run sick_tim_usb_ros2 usb_lidar_node

---

## Launch Cartographer SLAM

### Verify launch file discovery

```bash
ros2 launch slam_config cartographer_usb.launch.py --show-args
```

Expected output:
```
Arguments (pass arguments as '<name>:=<value>'):

    'use_sim_time':
        Use simulation clock if true
        (default: 'false')

    'resolution':
        Resolution of the map (meters per pixel)
        (default: '0.05')
```

### Start Cartographer SLAM

From the `slam/` directory:

```bash
./run_cartographer.sh
```

This launches:
- `sick_tim_usb_ros2/usb_lidar_node` publishing `/scan`
- Static transform `base_link` → `laser_frame`
- `cartographer_ros/cartographer_node` running 2D SLAM
- `cartographer_ros/cartographer_occupancy_grid_node` publishing `/map`
---

## Launch rosboard Web UI

### Start rosboard on port 8888

```bash
./run_rosboard.sh
```

### Access the web dashboard

Open a browser on your machine and navigate to:

```
http://jetty:8888
```

You should see:
- `/scan` topic displaying LiDAR data
- `/map` topic showing the occupancy grid
- TF tree: `map` → `odom` → `base_link` → `laser_frame`

---















## Customization

### Adjustable Parameters

**Launch arguments** (pass at runtime):
```bash
./run_cartographer.sh use_sim_time:=true resolution:=0.1
```

**LiDAR node parameters** (in `launch/cartographer_usb.launch.py`):
- `frame_id`: Laser reference frame (default: `laser_frame`)
- `publish_points`: Number of points published across 270° (default: `270` ≈ 1° increment)
- `range_min` / `range_max`: Valid range limits in meters (default: `0.1` / `10.0`)
- `log_throttle_sec`: Throttled logging period (default: `2.0`)

**Static transform** (in `launch/cartographer_usb.launch.py`):
- Edit `--x --y --z --qx --qy --qz --qw` arguments to match your LiDAR mounting pose.

**Cartographer tuning** (in `config/cartographer_2d_lidar.lua`):
- `TRAJECTORY_BUILDER_2D.min_range`, `max_range`
- `TRAJECTORY_BUILDER_2D.motion_filter.*`
- `TRAJECTORY_BUILDER_2D.submaps.*` (resolution, num_range_data)
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.*` (weights)
- `POSE_GRAPH.*` (constraint thresholds, optimization cadence)

See `src/slam_config/README.md` for detailed guidance.

---

## Troubleshooting

### LiDAR node fails to connect

**Check USB enumeration:**
```bash
lsusb | grep -i sick
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

**Verify hardware:**
- Cable is plugged in (both ends).
- LiDAR is powered on.
- Try a different USB port (prefer direct motherboard ports).
- Power-cycle: unplug USB, wait 1 minute, replug.

### Cartographer not publishing `/map`

**Check TF and `/scan`:**
```bash
ros2 topic list | grep -E "scan|map"
ros2 tf2_echo map base_link
```

**Verify configuration:**
- `cartographer_2d_lidar.lua` is readable at the path printed by the launch.
- `use_sim_time` matches your setup (false for real-time, true for bag playback).

### rosboard not accessible

**Check that rosboard is running:**
```bash
ps aux | grep rosboard
```

**Verify port 8888:**
```bash
netstat -tuln | grep 8888
```

**Check firewall:**
```bash
sudo ufw allow 8888/tcp || echo "ufw not enabled"
```

---

## File Structure

```
slam/
├── src/
│   ├── sick_tim_usb_ros2/
│   │   ├── package.xml
│   │   ├── sick_tim_usb_ros2/
│   │   │   ├── usb_lidar_node.py
│   │   │   └── L1_lidar_usb.py
│   │   └── setup.py
│   └── slam_config/
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── launch/
│       │   └── cartographer_usb.launch.py
│       ├── config/
│       │   └── cartographer_2d_lidar.lua
│       ├── rviz/
│       │   └── slam_tim561.rviz
│       └── README.md
├── build/
├── install/
├── log/
├── run_cartographer.sh
├── run_rosboard.sh
└── SETUP.md
```

---

## Summary

1. **Setup:** Create workspace, copy packages, install dependencies.
2. **Build:** `colcon build` for `sick_tim_usb_ros2` and `slam_config`.
3. **Launch:** `./run_cartographer.sh` for SLAM and `./run_rosboard.sh` for web UI.
4. **Customize:** Adjust LiDAR, TF, and Cartographer params as needed.
5. **Troubleshoot:** Check hardware, topics, and TF if issues arise.

For detailed parameter documentation, see `src/slam_config/README.md`.
