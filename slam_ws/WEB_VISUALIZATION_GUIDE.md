# Web Visualization Guide - View SLAM from Your Laptop Browser

## Quick Start (3 Steps)

### Terminal 1 - Start Cartographer SLAM:
```bash
cd ~/Adversary_DRONE/slam_ws
./run_cartographer.sh
```

### Terminal 2 - Start Rosboard Web Server:
```bash
cd ~/Adversary_DRONE/slam_ws
./run_rosboard_sudo.sh
```

### Laptop Browser - Open the Dashboard:
```
http://<jetson-ip>:8888
```

Replace `<jetson-ip>` with your Jetson's IP address (will be shown when you run the script).

---

## What You'll See in the Browser

### 1. **LaserScan Viewer** (`/scan`)
- Real-time lidar points in 2D radar view
- 811 points updating at 20 Hz
- Color-coded by distance

### 2. **Map Viewer** (`/map`)
- Occupancy grid map from Cartographer
- Black = occupied, white = free space
- Updates as the map builds

### 3. **Transform Tree** (`/tf`)
- Visual representation of: map → odom → base_link → laser_frame
- Shows robot position in real-time

### 4. **Odometry** (`/odom`)
- Robot pose from scan matching
- X, Y position and orientation

---

## Browser Setup Instructions

1. **Find Jetson IP**: When you run `run_rosboard_sudo.sh`, it will display:
   ```
   Remote: http://192.168.X.X:8888
   ```

2. **Open in Laptop Browser**:
   - Copy the URL
   - Open Chrome/Firefox/Edge
   - Paste the URL
   - You should see the Rosboard dashboard

3. **Add Visualizations**:
   - Click **"+"** button
   - Select topic type:
     - **LaserScan** → Choose `/scan`
     - **OccupancyGrid** → Choose `/map`  
     - **TF** → Shows transform tree
   - Resize panels as needed

4. **Customize View**:
   - Drag panels to rearrange
   - Resize with corner handles
   - Change visualization settings (colors, sizes, etc.)

---

## Example: Full SLAM Dashboard Layout

```
+------------------------+------------------------+
|                        |                        |
|    LaserScan (/scan)   |    Map (/map)         |
|    [Radar view]        |    [Occupancy Grid]   |
|                        |                        |
+------------------------+------------------------+
|                        |                        |
|    TF Tree             |    Odometry (/odom)   |
|    [Transforms]        |    [X,Y,θ display]    |
|                        |                        |
+------------------------+------------------------+
```

---

## Troubleshooting

### Can't Connect from Laptop

1. **Check Jetson IP**:
   ```bash
   hostname -I
   ```

2. **Check Same Network**: Laptop and Jetson must be on same WiFi/LAN

3. **Ping Test from Laptop**:
   ```bash
   ping <jetson-ip>
   ```

4. **Firewall**: If using firewall on Jetson:
   ```bash
   sudo ufw allow 8888/tcp
   ```

### Rosboard Shows "No Topics"

1. **Make sure Cartographer is running** in Terminal 1
2. **Check topics**:
   ```bash
   ros2 topic list
   ```
   Should show: `/scan`, `/map`, `/odom`, `/tf`

3. **Refresh browser** (F5)

### Slow Performance

1. **Reduce LaserScan size**: In rosboard settings, decrease point size
2. **Limit update rate**: Set topic refresh rate to 5 Hz instead of 20 Hz
3. **Use Chrome**: Better WebGL performance than other browsers

---

## Advanced: Custom Port

If port 8888 is busy, edit `run_rosboard_sudo.sh`:

```bash
# Change the port number
sudo -E env "PATH=..." rosboard_node --port 9999
```

Then access at: `http://<jetson-ip>:9999`

---

## Features You Can Use

### In LaserScan View:
- **Zoom**: Mouse scroll
- **Pan**: Click and drag
- **Rotate**: Right-click drag
- **Settings**: Click gear icon
  - Point size
  - Color scheme
  - Range limits

### In Map View:
- **Zoom**: Mouse scroll
- **Pan**: Click and drag
- **Color**: Click gear → change occupied/free colors
- **Grid**: Toggle grid overlay

### Recording Video:
- Some browsers allow screen recording
- Or use OBS Studio to record the browser window
- Great for documentation!

---

## Summary Commands

```bash
# Terminal 1 - SLAM
cd ~/Adversary_DRONE/slam_ws
./run_cartographer.sh

# Terminal 2 - Web Viz
cd ~/Adversary_DRONE/slam_ws  
./run_rosboard_sudo.sh

# Laptop Browser
http://<jetson-ip>:8888
```

**That's it!** You can now view and share your SLAM mapping in any web browser. 🌐🗺️
