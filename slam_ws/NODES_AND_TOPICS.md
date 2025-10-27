# SLAM System - Nodes and Topics Explained

## The Three Main Nodes in Your System

### 1️⃣ SICK LiDAR Driver Node
**Node Name:** `/sick_tim_5xx`
**Package:** `sick_scan_xd`
**Executable:** `sick_generic_caller`

**What it does:**
- Connects to TIM561 LiDAR via Ethernet
- Reads raw laser range data
- Converts to ROS2 message format

**PUBLISHES (sends data):**
- `/scan` → `sensor_msgs/LaserScan`
  - Contains: 811 range measurements (270° / 0.33° resolution)
  - Each measurement: distance in meters
  - Update rate: ~15 Hz

- `/diagnostics` → `diagnostic_msgs/DiagnosticArray`
  - LiDAR health status
  - Connection state
  - Error messages

**SUBSCRIBES (receives data):** 
- Nothing (it's a sensor driver, only outputs data)

---

### 2️⃣ SLAM Toolbox Node
**Node Name:** `/slam_toolbox`
**Package:** `slam_toolbox`
**Executable:** `async_slam_toolbox_node`

**What it does:**
- Takes LiDAR scans and builds a 2D map
- Tracks robot position (localization)
- Performs loop closure (corrects drift)
- Uses Karto SLAM algorithm with Ceres optimization

**SUBSCRIBES (receives data):**
- `/scan` ← from LiDAR driver
  - Laser range measurements
  
- `/odom` ← from odometry source (optional, improves accuracy)
  - Robot's estimated position from wheel encoders or visual odometry
  - **Currently not used** - SLAM works with scan-matching only

**PUBLISHES (sends data):**
- `/map` → `nav_msgs/OccupancyGrid`
  - The 2D grid map being built
  - Each cell: 0=free, 100=occupied, -1=unknown
  - Updated every 2 seconds (configurable)

- `/map_metadata` → `nav_msgs/MapMetaData`
  - Map dimensions, resolution, origin

- `/slam_toolbox/graph_visualization` → `visualization_msgs/MarkerArray`
  - SLAM graph for debugging

- `/slam_toolbox/scan_visualization` → `sensor_msgs/LaserScan`
  - Processed scans aligned to map

**PROVIDES SERVICES (on-demand actions):**
- `/slam_toolbox/serialize_map` - Save map to file
- `/slam_toolbox/deserialize_map` - Load map from file
- `/slam_toolbox/pause_new_measurements` - Pause mapping
- `/slam_toolbox/clear_queue` - Clear scan buffer

**PUBLISHES TF TRANSFORMS:**
- `map` → `odom` transform
  - Corrects drift in odometry
  - This is the SLAM output for localization

---

### 3️⃣ Static Transform Publisher Node
**Node Name:** `/base_to_laser_tf`
**Package:** `tf2_ros`
**Executable:** `static_transform_publisher`

**What it does:**
- Publishes fixed coordinate frame relationship
- Tells ROS2 where the LiDAR is mounted on the robot

**PUBLISHES TF TRANSFORMS:**
- `base_link` → `laser` transform
  - Translation: x=0, y=0, z=0.1m (10cm above robot base)
  - Rotation: no rotation (pointing forward)
  - **You should adjust this** to match your actual LiDAR mounting position!

---

## Complete Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                       PHYSICAL HARDWARE                          │
└─────────────────────────────────────────────────────────────────┘
                               │
                    SICK TIM561 LiDAR
                    (Ethernet Connection)
                               │
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│  NODE: /sick_tim_5xx                                             │
│  (sick_scan_xd package)                                          │
│                                                                   │
│  Reads raw laser data from LiDAR hardware                        │
└─────────────────────────────────────────────────────────────────┘
                               │
                               │ PUBLISHES
                               ↓
                    ╔══════════════════╗
                    ║  TOPIC: /scan    ║
                    ║  sensor_msgs/    ║
                    ║  LaserScan       ║
                    ╚══════════════════╝
                               │
                               │ SUBSCRIBES
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│  NODE: /slam_toolbox                                             │
│  (slam_toolbox package)                                          │
│                                                                   │
│  • Matches scans to build map                                    │
│  • Tracks robot position                                         │
│  • Optimizes graph (loop closure)                                │
│  • Publishes map and localization                                │
└─────────────────────────────────────────────────────────────────┘
                               │
                    ┌──────────┴──────────┐
                    ↓                     ↓
         ╔══════════════════╗   ╔═══════════════════╗
         ║  TOPIC: /map     ║   ║  TF: map→odom     ║
         ║  nav_msgs/       ║   ║                   ║
         ║  OccupancyGrid   ║   ║  (localization)   ║
         ╚══════════════════╝   ╚═══════════════════╝
                    │                     │
                    └──────────┬──────────┘
                               ↓
                        ┌─────────────┐
                        │   RViz2     │
                        │ Visualization│
                        └─────────────┘
```

---

## TF (Transform) Tree

SLAM Toolbox creates this coordinate frame hierarchy:

```
map (global, fixed coordinate frame)
 └─ odom (odometry frame, corrected by SLAM)
     └─ base_link (robot center)
         └─ laser (LiDAR sensor position)
```

**Published by:**
- `map → odom` : SLAM Toolbox (dynamic, corrects drift)
- `odom → base_link` : Odometry source OR SLAM scan-matching
- `base_link → laser` : Static transform publisher (fixed)

---

## Topic Details

### `/scan` Topic
**Message Type:** `sensor_msgs/LaserScan`

**Structure:**
```python
header:
  stamp: Time of measurement
  frame_id: "laser"
  
angle_min: -2.35619      # -135° (start of scan)
angle_max: 2.35619       # +135° (end of scan)
angle_increment: 0.00581 # 0.33° between points

range_min: 0.05          # Minimum valid range (5cm)
range_max: 10.0          # Maximum range (10m)

ranges: [2.34, 2.35, 2.36, ...]  # 811 measurements
  # Distance to obstacle in meters
  # inf = no detection
  # < range_min = too close
  # > range_max = too far

intensities: [47, 52, 48, ...]   # Signal strength (optional)
```

### `/map` Topic
**Message Type:** `nav_msgs/OccupancyGrid`

**Structure:**
```python
header:
  stamp: Time
  frame_id: "map"

info:
  resolution: 0.05        # 5cm per cell
  width: 384              # Map width in cells
  height: 384             # Map height in cells
  origin:
    position:
      x: -10.0            # Map origin position
      y: -10.0
      z: 0.0

data: [0, 0, 0, 100, 0, -1, ...]  # Flattened 2D array
  # 0 = Free space
  # 100 = Occupied (obstacle)
  # -1 = Unknown (not yet mapped)
```

---

## Check What's Running

**List active nodes:**
```bash
ros2 node list
```
Expected output:
```
/sick_tim_5xx
/slam_toolbox
/base_to_laser_tf
```

**List active topics:**
```bash
ros2 topic list
```
Expected output:
```
/scan
/map
/map_metadata
/tf
/tf_static
/slam_toolbox/feedback
/slam_toolbox/graph_visualization
...
```

**Monitor /scan topic:**
```bash
ros2 topic echo /scan
```

**Check publishing rate:**
```bash
ros2 topic hz /scan
```
Should show ~15 Hz

**Visualize node graph:**
```bash
rqt_graph
```
Shows boxes (nodes) and arrows (topics)

---

## Publisher vs Subscriber Analogy

Think of it like a **radio station**:

**Publisher (Radio Station):**
- Broadcasts data continuously
- Doesn't care who's listening
- Example: LiDAR publishes `/scan` at 15 Hz

**Subscriber (Radio Listener):**
- Tunes in to receive data
- Gets updates whenever new data arrives
- Example: SLAM Toolbox subscribes to `/scan`

**Topic (Radio Frequency):**
- The channel they communicate on
- Many can listen to same topic
- Example: `/scan` at 15 Hz

---

## Why These Specific Nodes?

1. **sick_scan_xd**: Hardware interface
   - Talks to physical LiDAR
   - Converts to ROS2 standard messages

2. **slam_toolbox**: Brain of the system
   - Takes sensor data
   - Builds map
   - Tracks position
   - No hardware interaction

3. **static_transform_publisher**: Geometry
   - Defines sensor mounting position
   - Critical for accurate mapping
   - No data processing

---

## Who Publishes What - Summary Table

| Node | Publishes | Subscribes |
|------|-----------|------------|
| `/sick_tim_5xx` | `/scan`, `/diagnostics` | - |
| `/slam_toolbox` | `/map`, TF `map→odom` | `/scan`, (optional `/odom`) |
| `/base_to_laser_tf` | TF `base_link→laser` | - |

---

## Test Your Understanding

**Try these commands after launching:**

```bash
# 1. Start SLAM system
ros2 launch slam_config tim561_slam.launch.py

# 2. In another terminal, check nodes
ros2 node list

# 3. See what /slam_toolbox publishes
ros2 node info /slam_toolbox

# 4. Monitor scan data
ros2 topic echo /scan --once

# 5. Check map data
ros2 topic echo /map --once

# 6. View TF tree
ros2 run tf2_tools view_frames

# 7. Monitor system graph
rqt_graph
```

---

Does this clarify the node/topic architecture? The key point: **we installed SLAM Toolbox from apt** (pre-compiled), and it acts as the **subscriber** to `/scan` and **publisher** of `/map`!
