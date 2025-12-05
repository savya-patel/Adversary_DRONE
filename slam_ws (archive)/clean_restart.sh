#!/bin/bash

# Clean restart of Cartographer SLAM
# This script stops all SLAM processes and clears old data

echo "========================================="
echo "  Cartographer Clean Restart"
echo "========================================="
echo ""

# 1. Kill any running Cartographer processes
echo "[1/4] Stopping all Cartographer processes..."
pkill -f cartographer
pkill -f usb_lidar_node
pkill -f rosboard
sleep 2

# 2. Check if anything is still running
RUNNING=$(ps aux | grep -E "cartographer|usb_lidar" | grep -v grep)
if [ ! -z "$RUNNING" ]; then
    echo "⚠️  Warning: Some processes still running:"
    echo "$RUNNING"
    echo ""
    read -p "Force kill with sudo? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo pkill -9 -f cartographer
        sudo pkill -9 -f usb_lidar_node
        sleep 1
    fi
fi

# 3. Optional: Clear old maps (ask first)
echo ""
echo "[2/4] Map data cleanup..."
read -p "Clear old map data from /tmp? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    rm -f /tmp/cartographer_*.pbstream
    rm -f /tmp/*.pgm
    rm -f /tmp/*.yaml
    echo "✓ Cleared old map files"
else
    echo "✓ Keeping existing maps"
fi

# 4. Optional: Archive old logs
echo ""
echo "[3/4] Log management..."
read -p "Archive old logs? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd ~/Adversary_DRONE/slam_ws/logs
    if [ $(ls -1 cartographer_*.log 2>/dev/null | wc -l) -gt 0 ]; then
        ARCHIVE_DIR="archive_$(date +%Y%m%d_%H%M%S)"
        mkdir -p "$ARCHIVE_DIR"
        mv cartographer_*.log "$ARCHIVE_DIR/"
        echo "✓ Archived logs to logs/$ARCHIVE_DIR/"
    else
        echo "✓ No logs to archive"
    fi
    cd ~/Adversary_DRONE/slam_ws
else
    echo "✓ Keeping logs as-is"
fi

# 5. Show available launch options
echo ""
echo "========================================="
echo "[4/4] Launch Options Available:"
echo "========================================="
echo ""
echo "1. Standard Cartographer (recommended)"
echo "   ./run_cartographer.sh"
echo ""
echo "2. Cartographer with Logging"
echo "   ./run_cartographer_log.sh"
echo ""
echo "3. Web Visualization (Rosboard)"
echo "   Terminal 1: ./run_cartographer.sh"
echo "   Terminal 2: ./run_rosboard_sudo.sh"
echo ""
echo "4. slam_toolbox (has TF issues, not recommended)"
echo "   ./run_slam.sh"
echo ""
echo "========================================="
echo "System ready for clean launch!"
echo "========================================="
