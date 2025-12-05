#!/bin/bash
# Complete SLAM cleanup script - kills ALL SLAM-related processes

echo "========================================="
echo "  Killing ALL SLAM Processes"
echo "========================================="

# Kill slam_toolbox and rf2o (old setup)
echo "[1/5] Killing slam_toolbox and rf2o..."
pkill -9 -f slam_toolbox
pkill -9 -f rf2o_laser_odometry
sudo pkill -9 -f slam_toolbox 2>/dev/null
sudo pkill -9 -f rf2o_laser_odometry 2>/dev/null

# Kill Cartographer
echo "[2/5] Killing Cartographer..."
pkill -9 -f cartographer_node
pkill -9 -f cartographer_occupancy_grid_node

# Kill USB LiDAR node
echo "[3/5] Killing USB LiDAR node..."
pkill -9 -f usb_lidar_node
pkill -9 -f usb_tim_publisher

# Kill all ROS 2 launch processes
echo "[4/5] Killing all ROS 2 launch files..."
pkill -9 -f "ros2 launch slam_config"
sudo pkill -9 -f "ros2 launch slam_config" 2>/dev/null

# Kill static TF publishers
echo "[5/5] Killing static TF publishers..."
pkill -9 -f static_transform_publisher
sudo pkill -9 -f static_transform_publisher 2>/dev/null

# Kill Rosboard
pkill -9 -f rosboard_node

sleep 2

echo ""
echo "Cleanup complete! Checking remaining nodes..."
echo ""
ros2 node list 2>&1

echo ""
echo "Remaining SLAM processes:"
ps aux | grep -E "slam_toolbox|cartographer|rf2o|usb_lidar|usb_tim" | grep -v grep || echo "  None found ✓"

echo ""
echo "========================================="
echo "  All SLAM processes stopped!"
echo "========================================="
