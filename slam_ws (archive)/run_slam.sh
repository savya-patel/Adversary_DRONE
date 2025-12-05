#!/usr/bin/env bash
# Simple launcher for USB LiDAR + slam_toolbox
# Usage:
#   sudo ./run_slam.sh         # default params
#   sudo ./run_slam.sh 10hz    # lower CPU profile
#   sudo SLAM_PARAMS_FILE=/full/path.yaml ./run_slam.sh  # override params

set -eo pipefail

WS=/home/eagle/Adversary_DRONE/slam_ws
PARAMS_DEFAULT="$WS/src/slam_config/config/mapper_params_online_async.yaml"
PARAMS_10HZ="$WS/src/slam_config/config/mapper_params_online_async_10hz.yaml"
PROFILE=${1:-default}

# Re-exec under sudo preserving env if not already root
if [[ $EUID -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

cd "$WS"
source /opt/ros/humble/setup.bash
# Source workspace if present
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
fi
# Ensure discovery not localhost-only for web tools
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Select params
SLAM_PARAMS="$PARAMS_DEFAULT"
if [[ "$PROFILE" == "10hz" ]]; then
  SLAM_PARAMS="$PARAMS_10HZ"
fi
# Env override wins
if [[ -n "$SLAM_PARAMS_FILE" ]]; then
  SLAM_PARAMS="$SLAM_PARAMS_FILE"
fi

if [[ ! -f "$SLAM_PARAMS" ]]; then
  echo "ERROR: Params file not found: $SLAM_PARAMS" >&2
  exit 1
fi

echo "Launching SLAM with params: $SLAM_PARAMS"
exec ros2 launch slam_config slam_usb.launch.py slam_params_file:="$SLAM_PARAMS"