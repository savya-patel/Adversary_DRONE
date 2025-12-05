#!/bin/bash

# RViz2 Launcher for Cartographer SLAM Visualization
# Run this in a separate terminal AFTER starting run_cartographer.sh

echo "========================================="
echo "  Starting RViz2 for SLAM Visualization"
echo "========================================="
echo ""
echo "Configure RViz2:"
echo "  1. Fixed Frame: 'map'"
echo "  2. Add LaserScan → Topic: /scan"
echo "  3. Add Map → Topic: /map"
echo "  4. Add TF (optional)"
echo ""
echo "========================================="
echo ""

# Set display
export DISPLAY=:1

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/Adversary_DRONE/slam_ws/install/setup.bash

# Launch RViz2
rviz2
