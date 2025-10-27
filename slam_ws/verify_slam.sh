#!/bin/bash
# Comprehensive SLAM verification script - works over SSH without RViz

WS_PATH="/home/eagle/Adversary_DRONE/slam_ws"

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         SLAM Toolbox Verification (No RViz Needed)        ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Source ROS
source /opt/ros/humble/setup.bash
source $WS_PATH/install/setup.bash

echo "=== 1. ROS2 Nodes Running ==="
ros2 node list
echo ""

echo "=== 2. Critical Topics ==="
ros2 topic list | grep -E "(/scan|/map|/tf|/slam_toolbox)" || echo "No SLAM topics found!"
echo ""

echo "=== 3. USB LiDAR Publishing Status ==="
echo -n "Publisher: "
ros2 node info /usb_tim_publisher 2>/dev/null | grep "Publisher" | head -1 || echo "❌ USB LiDAR node not found"
echo -n "Scan rate: "
timeout 5 ros2 topic hz /scan 2>&1 | grep "average rate" || echo "⏳ Waiting for scans..."
echo ""

echo "=== 4. SLAM Toolbox Subscription Status ==="
SCAN_INFO=$(ros2 topic info /scan -v 2>&1)
echo "$SCAN_INFO" | grep -A10 "Subscription count" | head -12
echo ""

QOS_MATCH=$(echo "$SCAN_INFO" | grep -c "BEST_EFFORT")
if [ $QOS_MATCH -eq 2 ]; then
    echo "✅ QoS Match: Publisher and Subscriber both use BEST_EFFORT"
else
    echo "❌ QoS Mismatch: Check publisher/subscriber QoS settings"
fi
echo ""

echo "=== 5. SLAM Processing Status ==="
echo -n "Map updates: "
timeout 3 ros2 topic hz /map 2>&1 | grep "average rate" || echo "⏳ No map published yet (needs movement)"

echo -n "TF transforms: "
timeout 3 ros2 topic hz /tf 2>&1 | grep "average rate" || echo "⏳ No TF published yet (needs movement)"

echo -n "Pose updates: "
timeout 3 ros2 topic hz /pose 2>&1 | grep "average rate" || echo "⏳ No pose published yet (needs movement)"
echo ""

echo "=== 6. SLAM Services Available ==="
ros2 service list | grep slam_toolbox | wc -l | xargs echo "SLAM services:"
echo ""

echo "=== 7. TF Tree Status ==="
TF_FRAMES=$(ros2 run tf2_tools view_frames 2>&1 | grep "frame_yaml")
if echo "$TF_FRAMES" | grep -q "map"; then
    echo "✅ Complete TF chain: map → odom → base_link → laser_frame"
else
    echo "⚠️  Partial TF chain (map frame missing - SLAM not initialized)"
    echo "   Current frames: odom → base_link → laser_frame"
fi
echo ""

echo "=== 8. One Scan Sample (First 10 ranges) ==="
timeout 3 ros2 topic echo /scan --once 2>&1 | grep -A10 "ranges:" | head -11 || echo "No scan data"
echo ""

echo "╔════════════════════════════════════════════════════════════╗"
echo "║                    Diagnostic Summary                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check if SLAM is fully operational
if timeout 3 ros2 topic hz /tf 2>&1 | grep -q "average rate"; then
    echo "✅ SLAM IS WORKING: Map is being built!"
    echo ""
    echo "Next steps:"
    echo "  • Save map: cd ~/Adversary_DRONE/slam_ws && ./save_map.sh my_map"
    echo "  • View in browser: cd ~/Adversary_DRONE/slam_ws && ./run_rosboard.sh"
else
    echo "⏳ SLAM IS READY BUT WAITING: Everything configured correctly"
    echo ""
    echo "To start mapping:"
    echo "  1. Move the LiDAR slowly (or the robot/platform it's on)"
    echo "  2. Minimum movement needed:"
    echo "     - 1cm translation OR"
    echo "     - 0.01 radians rotation (~0.5°)"
    echo "  3. Watch terminal 1 for 'New node added' messages"
    echo "  4. Re-run this script to see map updates"
    echo ""
    echo "Alternative - Test with stationary scans:"
    echo "  • Slightly rotate the LiDAR by hand (even 1-2 degrees)"
    echo "  • Or use rosboard for live view: ./run_rosboard.sh"
fi
echo ""
