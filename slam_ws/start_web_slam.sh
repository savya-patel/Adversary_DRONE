#!/bin/bash

# Quick Launch Script for Web-Based SLAM Visualization
# Since you're SSH'd into 10.250.240.81, access from laptop at:
# http://10.250.240.81:8888

echo "========================================="
echo "  Web SLAM Visualization Quick Start"
echo "========================================="
echo ""
echo "Jetson IP: 10.250.240.81"
echo ""
echo "🌐 OPEN IN YOUR LAPTOP BROWSER:"
echo "   http://10.250.240.81:8888"
echo ""
echo "========================================="
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Start Cartographer in background
echo "[1/2] Starting Cartographer SLAM..."
cd ~/Adversary_DRONE/slam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

gnome-terminal -- bash -c "cd ~/Adversary_DRONE/slam_ws && ./run_cartographer.sh; exec bash" &
sleep 5

# Start Rosboard
echo "[2/2] Starting Rosboard Web Server..."
echo ""
echo "========================================="
echo "✅ Ready! Open your laptop browser to:"
echo ""
echo "   http://10.250.240.81:8888"
echo ""
echo "Then click '+' and add:"
echo "  - LaserScan → /scan"
echo "  - OccupancyGrid → /map"
echo "========================================="
echo ""

./run_rosboard_sudo.sh
