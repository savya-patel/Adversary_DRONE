# Finding Where TIM561 Publishes Scan Data - Code Location Guide

## 🎯 THE KEY FILE: Where Scans Are Published

**Main File:** `/home/eagle/slam_ws/src/sick_scan_xd/driver/src/sick_scan_common.cpp`

This is **299,089 bytes** and **6,573 lines** of C++ code - the heart of the LiDAR driver!

---

## 📍 Key Locations in sick_scan_common.cpp

### **1. Publisher Setup (Line ~676)**
```cpp
pub_ = rosAdvertise<ros_sensor_msgs::LaserScan>(nh, laserscan_topic, 1000);
```
**What it does:** Creates the `/scan` topic publisher with queue size 1000

---

### **2. Main Publishing Code (Lines 5030-5040)**
```cpp
if (sendMsg && (outputChannelFlagId != 0))
{
#if defined USE_DIAGNOSTIC_UPDATER
  if(diagnosticPub_)
    diagnosticPub_->publish(msg);  // <-- PUBLISHES LaserScan HERE
  else
    rosPublish(pub_, msg);          // <-- OR HERE (normal mode)
#else
  rosPublish(pub_, msg);            // <-- PUBLISHES /scan topic
#endif
}
```
**What it does:** This is WHERE the actual `/scan` topic gets published!

**The `msg` variable is of type:** `ros_sensor_msgs::LaserScan`

---

### **3. Building the LaserScan Message (Lines 4800-5000)**

**Key fields being set:**

```cpp
// Line ~4651: Message declaration
ros_sensor_msgs::LaserScan msg;

// Line ~4700-4800: Header setup
msg.header.stamp = timestamp;
msg.header.frame_id = config_.frame_id;  // Usually "laser"

// Line ~4850: Angle setup
msg.angle_min = angleMinRad;          // -135° for TIM561
msg.angle_max = angleMaxRad;          // +135° for TIM561  
msg.angle_increment = angleIncrement; // 0.33° for TIM561

// Line ~4900: Range setup
msg.range_min = minRange;              // 0.05m
msg.range_max = maxRange;              // 10.0m

// Line ~4950-4990: Fill in the actual measurements
for (int j = 0; j < numChannels; j++)
{
  msg.ranges.push_back(distVal);      // Distance in meters
  msg.intensities.push_back(rssiVal); // Signal strength
}
```

---

## 🗺️ How to Find Coordinates/Points

### **The LaserScan Message Structure:**

```cpp
sensor_msgs/LaserScan {
  Header header
  float32 angle_min        # Start angle (-2.356 rad = -135°)
  float32 angle_max        # End angle (2.356 rad = +135°)
  float32 angle_increment  # Step between rays (0.0058 rad = 0.33°)
  float32 time_increment   # Time between measurements
  float32 scan_time        # Time for full scan
  float32 range_min        # Min valid range (0.05m)
  float32 range_max        # Max valid range (10.0m)
  float32[] ranges         # ARRAY OF 811 DISTANCES [m]
  float32[] intensities    # ARRAY OF 811 SIGNAL STRENGTHS
}
```

### **Converting to XY Coordinates:**

The `ranges` array gives you **POLAR coordinates** (distance + angle).
To get **Cartesian coordinates** (x, y):

```cpp
// For each point i in ranges array:
float angle = msg.angle_min + (i * msg.angle_increment);
float distance = msg.ranges[i];

// Convert to Cartesian (assuming laser at origin, pointing forward)
float x = distance * cos(angle);
float y = distance * sin(angle);
```

**Example for point 400 (center of 811-point scan):**
```cpp
int i = 400;  // Middle point (points straight ahead)
float angle = -2.356 + (400 * 0.0058) = 0.0 radians (0°)
float distance = msg.ranges[400];  // e.g., 3.5 meters

float x = 3.5 * cos(0) = 3.5m  (forward)
float y = 3.5 * sin(0) = 0.0m  (no side offset)
```

---

## 🎨 Where Colors/Intensities Come From

### **In sick_scan_common.cpp (~Line 4990):**

```cpp
// Extract RSSI (Received Signal Strength Indicator)
uint16_t rssi = dataPtr[idx + i * 2 + 1];  // Raw intensity value
float rssiVal = (float)rssi;
msg.intensities.push_back(rssiVal);
```

**What is intensity?**
- NOT color (LiDAR doesn't see color)
- Signal reflection strength (0-65535 range typically)
- Higher values = stronger reflection (e.g., reflective materials)
- Lower values = weaker reflection (e.g., dark/absorptive surfaces)

**To visualize as color in RViz:**
- RViz maps intensity → color gradient
- Usually: low intensity = dark blue, high = bright red
- You can change this in RViz display settings

---

## 📂 Other Important Files

### **1. sick_generic_caller.cpp** (7,291 bytes)
**Location:** `~/slam_ws/src/sick_scan_xd/driver/src/sick_generic_caller.cpp`

**What it does:** 
- Main entry point (has `main()` function)
- Initializes ROS2 node
- Creates `SickScanCommon` object
- Starts the scanning loop

**Key line (~100):**
```cpp
int main(int argc, char **argv)
{
  rosInit(argc, argv, nodeName);
  SickScanCommon *commonPtr = new SickScanCommon(nh);
  commonPtr->init();
  commonPtr->run();  // <-- Enters main scanning loop
}
```

---

### **2. sick_generic_parser.cpp** (63,128 bytes)
**Location:** `~/slam_ws/src/sick_scan_xd/driver/src/sick_generic_parser.cpp`

**What it does:**
- Parses raw binary data from TIM561
- Extracts distance measurements
- Extracts RSSI (intensity) values
- Handles different SICK protocols

---

### **3. sick_scan_common_tcp.cpp** (27,805 bytes)
**Location:** `~/slam_ws/src/sick_scan_xd/driver/src/sick_scan_common_tcp.cpp`

**What it does:**
- TCP/IP network communication
- Connects to LiDAR via Ethernet
- Sends commands to LiDAR
- Receives raw scan data packets

**Key function (~Line 150):**
```cpp
int SickScanCommonTcp::readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size)
{
  // Read data from TCP socket
  // Returns raw binary scan data from TIM561
}
```

---

## 🔍 How to Add Your Own Processing

### **Option 1: Modify sick_scan_common.cpp**

**Before publishing (Line ~5030), add processing:**

```cpp
// Right before: rosPublish(pub_, msg);

// Example: Log statistics
float min_dist = *std::min_element(msg.ranges.begin(), msg.ranges.end());
float max_dist = *std::max_element(msg.ranges.begin(), msg.ranges.end());
ROS_INFO("Scan: min=%.2fm, max=%.2fm", min_dist, max_dist);

// Example: Filter out invalid measurements
for(size_t i = 0; i < msg.ranges.size(); i++) {
  if(msg.ranges[i] < msg.range_min || msg.ranges[i] > msg.range_max) {
    msg.ranges[i] = std::numeric_limits<float>::infinity();
  }
}

// Example: Convert to XY coordinates
std::vector<std::pair<float,float>> xy_points;
for(size_t i = 0; i < msg.ranges.size(); i++) {
  float angle = msg.angle_min + (i * msg.angle_increment);
  float dist = msg.ranges[i];
  if(std::isfinite(dist)) {  // Valid measurement
    float x = dist * cos(angle);
    float y = dist * sin(angle);
    xy_points.push_back({x, y});
  }
}

// Now publish original message
rosPublish(pub_, msg);
```

---

### **Option 2: Create Subscriber Node (RECOMMENDED)**

**Better approach:** Don't modify driver, create separate processing node

**File:** `~/slam_ws/src/slam_config/scripts/scan_processor.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
    def scan_callback(self, msg):
        # Get all measurements
        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities)
        
        # Calculate angles for each point
        num_points = len(ranges)
        angles = msg.angle_min + np.arange(num_points) * msg.angle_increment
        
        # Convert to XY coordinates
        valid = np.isfinite(ranges)  # Filter out inf/nan
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        
        # Find closest obstacle
        if len(ranges[valid]) > 0:
            min_dist = np.min(ranges[valid])
            min_idx = np.argmin(ranges[valid])
            min_angle = angles[valid][min_idx]
            
            self.get_logger().info(
                f'Closest: {min_dist:.2f}m at {np.degrees(min_angle):.1f}°'
            )
        
        # Find points with high intensity (reflective surfaces)
        high_intensity = intensities > 1000
        self.get_logger().info(
            f'Reflective points: {np.sum(high_intensity)}'
        )

def main():
    rclpy.init()
    node = ScanProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Make executable and run:**
```bash
chmod +x ~/slam_ws/src/slam_config/scripts/scan_processor.py
ros2 run slam_config scan_processor.py
```

---

## 📊 Topic Data Flow Summary

```
TIM561 Hardware
    ↓ (Ethernet)
sick_scan_common_tcp.cpp → readWithTimeout()
    ↓ (raw binary data)
sick_generic_parser.cpp → parse_datagram()
    ↓ (extracted distances + intensities)
sick_scan_common.cpp → build LaserScan msg
    ↓ (Line 5037)
rosPublish(pub_, msg)
    ↓ (publishes to /scan topic)
ROS2 Network
    ↓ (subscribes)
slam_toolbox → process scan
YOUR_NODE → process scan (optional)
rviz2 → visualize scan
```

---

## 🛠️ Quick Commands to Inspect Data

### **See message structure:**
```bash
ros2 interface show sensor_msgs/msg/LaserScan
```

### **Live view of scan data:**
```bash
# See one scan
ros2 topic echo /scan --once

# See field names only
ros2 topic echo /scan --once --no-arr

# Monitor rate
ros2 topic hz /scan
```

### **Dump scan to file for analysis:**
```bash
ros2 topic echo /scan > scan_data.txt
```

### **Extract just ranges:**
```bash
ros2 topic echo /scan --once | grep "ranges:"
```

---

## 📝 Summary

**To find coordinates/points:**
1. **Publisher code:** Line 5037 in `sick_scan_common.cpp`
2. **Message type:** `sensor_msgs/LaserScan`
3. **Data location:** `msg.ranges[]` array (811 distances)
4. **Intensities:** `msg.intensities[]` array (signal strength, NOT color)
5. **To get XY:** Calculate from polar coords (see examples above)

**Best practice:** Create separate subscriber node instead of modifying driver!

---

Need help implementing custom scan processing? Want to extract specific obstacles or add color visualization? Ask away! 🚀
