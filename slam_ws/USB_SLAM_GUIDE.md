# USB LiDAR SLAM - Quick Start

SICK TiM561 LiDAR + **Google Cartographer** SLAM over USB (no Ethernet, **no odometry needed**!).

## ⚡ Quick Start (Cartographer - RECOMMENDED)

**Rebuild (after code changes):**
```bash
cd ~/Adversary_DRONE/slam_ws
colcon build --packages-select sick_tim_usb_ros2 slam_config --symlink-install --merge-install
source install/setup.bash
```

**Launch Cartographer SLAM (NO ODOMETRY!):**
```bash
cd ~/Adversary_DRONE/slam_ws
./run_cartographer.sh
```

**Expected:**
- ✅ `USB bulk connected (IN=0x81, OUT=0x02)` or `USB serial at /dev/ttyACM0`
- ✅ `Publishing scan: 811 points, 0.333° increment`
- ✅ `Found 'cartographer_2d_lidar.lua'`
- ✅ `Added trajectory with ID '0'`
- ✅ `Inserted submap (0, 0)` and incrementing

**Why Cartographer?**
- ✅ **Works perfectly without odometry** (pure lidar scan matching)
- ✅ **No TF message filter bugs** (unlike slam_toolbox on Humble)
- ✅ High-quality loop closure detection
- ✅ Optimized for real-time on embedded systems

---

## Alternative: SLAM Toolbox (Has Issues on Humble)

**Note:** slam_toolbox has a known ROS 2 Humble bug where the TF message filter drops all scans when using high-frequency LiDAR data. Use Cartographer instead!

**If you still want to try it:**
```bash
cd ~/Adversary_DRONE/slam_ws
sudo bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_config slam_usb.launch.py"
```

**Known issue:** You'll see `Message Filter dropping message... queue is full` continuously.

---

## Visualization (Pick One)

### 🌐 Web Polar Viewer (Best - No X11)
**Auto-launcher (easiest):**
```bash
cd ~/Adversary_DRONE/slam_ws && ./run_web_viz.sh
```
Open: `http://10.250.240.81:8080`

**Manual (3 terminals):**
```bash
# T1: SLAM (see above)
# T2: Rosbridge WebSocket
sudo bash -c "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
# T3: Web server
python3 lidar_web_viz.py
```

**Features:**
- 🎯 270° polar plot (like `L1_lidar_GUI_tunnel.py`)
- 🔴🟠🟡🟢 Color-coded obstacles: <1m red, <2m orange, <6m yellow, >6m green
- ⚠️ Real-time warnings: DANGER/WARNING/CAUTION/CLEAR
- 📊 Live stats: points, nearest obstacle, scan rate, close objects
- 🌍 Works over SSH (no X11 forwarding)
- 🔌 **Ports:** 8080 (HTTP web page), 9090 (WebSocket ROS2 bridge)

### 📊 Rosboard (Table View)
```bash
./run_rosboard_sudo.sh  # Open: http://10.250.240.81:8888
```
Shows `/scan` (data table), `/map` (SLAM grid)

### 🖥️ LiDAR Scan Viewer (X11 GUI)
```bash
source install/setup.bash && python3 lidar_scan_viewer.py
```

### 🔧 RViz (Full 3D - X11)
```bash
source install/setup.bash && rviz2 -d src/slam_config/rviz/slam_tim561.rviz
```

---

## Verify Working

**Map check:**
```bash
sudo -E bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap '{}'" | grep -A2 "width\|height"
```
✅ Non-zero width/height = mapping

**TF check:**
```bash
sudo -E bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run tf2_ros tf2_echo map laser_frame"
```
✅ Continuous transforms @ 1 Hz

**Save map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

---

## Configuration

### TIM561 Params
- **Connection:** USB Serial (`/dev/ttyACM0`) or USB Bulk (19a2:5001)
- **FOV:** 270° (-135° to +135°)
- **Range:** 0.05m - 10.0m
- **Resolution:** `1.0°` (271pts), `0.5°` (541pts), `0.33°` (811pts)
- **Accuracy:** ±5cm typical

### SLAM Params (`/src/slam_config/config/mapper_params_online_async.yaml`)
- `minimum_time_interval: 0.5` → Updates every 0.5s (**no movement needed!**)
- `minimum_travel_distance: 0.005` → 5mm threshold
- `resolution: 0.05` → 5cm grid cells
- `max_laser_range: 10.0` → Match TIM561 spec

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| USB not found | `lsusb \| grep 19a2`, check cable |
| Permission denied | Use `sudo` for launches |
| No `/map` topic | Wait 15s after launch |
| Web viz no data | Check rosbridge running (port 9090) |
| Range discrepancy | Rebuild: `colcon build --merge-install` |
| rosboard "waiting" | Use `./run_rosboard_sudo.sh` (DDS context) |

**DDS Issue:** Sudo-launched nodes invisible to regular user. Always use:
```bash
sudo -E bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && <cmd>"
```

---

## Architecture

**Nodes:** `usb_lidar_node` → `slam_toolbox` → TF publishers  
**TF Chain:** `map → odom → base_link → laser_frame`  
**QoS:** `BEST_EFFORT` / `VOLATILE` / `KEEP_LAST:10`

**Scripts:**
- `run_web_viz.sh` → Auto-launch polar viewer (ports 8080/9090)
- `run_rosboard_sudo.sh` → Table view (port 8888)
- `lidar_scan_viewer.py` → GUI polar plot
- `lidar_web_viz.py` → Web server for polar plot

**Ports:**
- **8080:** HTTP web page (lidar_web_viz.py)
- **8888:** Rosboard web interface
- **9090:** WebSocket bridge (rosbridge) for ROS2 ↔ JavaScript

---

## Lidar-only vs with laser_scan_matcher

Two valid ways to run without a wheel odometer:

1) Simple (SLAM-only, current setup)
- Params in `src/slam_config/config/mapper_params_online_async.yaml`:
	- `use_odometry: false`
	- `use_tf_scan_transformation: false`
	- `provide_odom_frame: true`
- TFs:
	- `map -> odom` (slam_toolbox)
	- `odom -> base_link` (identity static TF)
	- `base_link -> laser_frame` (static TF)
- Notes:
	- Global pose in `map` still updates because slam_toolbox updates `map->odom`.
	- Good for mapping and visualization; no `/odom` topic for controllers.

2) With `ros2_laser_scan_matcher` (recommended if you need /odom)
- Start `laser_scan_matcher` from `/scan`.
- Update SLAM params:
	- `use_odometry: true`
	- `use_tf_scan_transformation: true`
	- `provide_odom_frame: true`
- TFs:
	- `map -> odom` (slam_toolbox)
	- `odom -> base_link` (laser_scan_matcher)
	- `base_link -> laser_frame` (static TF)
- Notes:
	- Provides smooth local odometry on `/odom` with twist.
	- SLAM still handles global corrections via `map->odom`.

## Using rf2o_laser_odometry (lightweight odom)

If you prefer a lighter odometry node, use `rf2o_laser_odometry` instead of laser_scan_matcher.

Install (Humble):
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-rf2o-laser-odometry
```

Run with SLAM (odom-enabled variant):
```bash
cd ~/Adversary_DRONE/slam_ws
sudo bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_config slam_usb_with_odom.launch.py"
```

What this does:
- rf2o publishes `odom -> base_link` and `/odom` from `/scan`.
- slam_toolbox uses odom and publishes `map -> odom` (params file: `mapper_params_online_async_odom.yaml`).
- Identity `odom -> base_link` static TF is NOT used in this variant.

Verify TF and topics:
```bash
sudo -E bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run tf2_ros tf2_echo odom base_link"
sudo -E bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic echo -n1 /odom"
```
