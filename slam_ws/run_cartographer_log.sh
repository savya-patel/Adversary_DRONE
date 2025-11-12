#!/bin/bash

# Cartographer SLAM Logger
# Logs all output to timestamped file for analysis

# Create logs directory if it doesn't exist
LOGS_DIR=~/Adversary_DRONE/slam_ws/logs
mkdir -p $LOGS_DIR

# Generate timestamp for log file
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$LOGS_DIR/cartographer_$TIMESTAMP.log"

echo "========================================="
echo "  Cartographer SLAM with Logging"
echo "========================================="
echo "Hardware: SICK TiM561 via USB"
echo "SLAM: Google Cartographer (no odom!)"
echo ""
echo "Log file: $LOG_FILE"
echo ""
echo "Topics:"
echo "  - /scan (input from LiDAR)"
echo "  - /map (output occupancy grid)"
echo "  - /odom (from scan matching)"
echo "========================================="
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/Adversary_DRONE/slam_ws/install/setup.bash

# Log start time
echo "======================================== CARTOGRAPHER LOG ========================================" > $LOG_FILE
echo "Start time: $(date)" >> $LOG_FILE
echo "Jetson: $(hostname)" >> $LOG_FILE
echo "ROS 2 Distro: $ROS_DISTRO" >> $LOG_FILE
echo "==================================================================================================" >> $LOG_FILE
echo "" >> $LOG_FILE

# Launch Cartographer and tee output to both terminal and log file
ros2 launch slam_config cartographer_usb.launch.py 2>&1 | tee -a $LOG_FILE

# Log end time
echo "" >> $LOG_FILE
echo "==================================================================================================" >> $LOG_FILE
echo "End time: $(date)" >> $LOG_FILE
echo "==================================================================================================" >> $LOG_FILE

echo ""
echo "========================================="
echo "Log saved to: $LOG_FILE"
echo "========================================="
