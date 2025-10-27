#!/bin/bash
# Save the current SLAM map to ~/maps/<name>.pgm/.yaml
set -eo pipefail
NAME=${1:-map_$(date +%Y%m%d_%H%M%S)}
OUTDIR=${2:-$HOME/maps}
mkdir -p "$OUTDIR"
WS=/home/eagle/Adversary_DRONE/slam_ws
cd "$WS"
# Use sudo to match slam running as root and ensure access to the same ROS graph
sudo -E bash -c "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f $OUTDIR/$NAME"
echo "Saved map to $OUTDIR/$NAME.[pgm|yaml]"