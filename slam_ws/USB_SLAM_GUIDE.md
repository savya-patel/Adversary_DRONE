# USB LiDAR SLAM - Quick Start

SICK TiM561 LiDAR + SLAM Toolbox over USB (no Ethernet, no odometry).

## ⚡ Quick Start

**Rebuild (after code changes):**
```bash
cd ~/Adversary_DRONE/slam_ws
colcon build --packages-select sick_tim_usb_ros2 slam_config --symlink-install --merge-install
source install/setup.bash
```

**Launch SLAM:**
```bash
cd ~/Adversary_DRONE/slam_ws
sudo bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_config slam_usb.launch.py"
```

**Expected:**
- ✅ `USB bulk connected (IN=0x81, OUT=0x02)` or `USB serial at /dev/ttyACM0`
- ✅ `Publishing scan: 271 points`
- ✅ `Registering sensor: [Custom Described Lidar]`

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
