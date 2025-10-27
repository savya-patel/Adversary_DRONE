#!/bin/bash
# Launch rosboard with sudo to access topics published by sudo-launched nodes
# Access at http://<jetson-ip>:8888 or http://localhost:8888

cd ~/Adversary_DRONE/slam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "Starting Rosboard Web Interface (with sudo)"
echo "=========================================="
echo ""
echo "Access the dashboard at:"
echo "  - Local: http://localhost:8888"
echo "  - Remote: http://$(hostname -I | awk '{print $1}'):8888"
echo ""
echo "Visualizations available:"
echo "  - /scan → LaserScan (radar view)"
echo "  - /map → OccupancyGrid (SLAM map)"
echo "  - /tf → Transform tree"
echo "  - /pose → Robot pose"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Run rosboard with sudo but use user's installation
sudo -E env "PATH=/home/eagle/.local/bin:$PATH" bash -c "source /opt/ros/humble/setup.bash && source $PWD/install/setup.bash && /home/eagle/.local/bin/rosboard_node"
