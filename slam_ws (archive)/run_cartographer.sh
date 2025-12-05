#!/bin/bash
# Launch Cartographer SLAM with USB LiDAR (no odometry needed!)

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "========================================="
echo "  Cartographer SLAM with USB LiDAR"
echo "========================================="
echo "Hardware: SICK TiM561 via USB"
echo "SLAM: Google Cartographer (no odom!)"
echo "Topics:"
echo "  - /scan (input from LiDAR)"
echo "  - /map (output occupancy grid)"
echo "  - /odom (from scan matching)"
echo "========================================="
echo ""

ros2 launch slam_config cartographer_usb.launch.py
