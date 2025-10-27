#!/usr/bin/env bash
# Rebuild ignored artifacts (build/install/log) and install dependencies.
# Usage:
#   WS_DIR=~/slam_ws ROS_DISTRO=humble CMAKE_BUILD_TYPE=Release ./scripts/bootstrap_build.sh
# Defaults: WS_DIR=$HOME/slam_ws, ROS_DISTRO=humble, CMAKE_BUILD_TYPE=Release, MERGE_INSTALL=1

set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-humble}
# Default WS_DIR to parent of this script (repo-local slam_ws) if not set
if [[ -z "${WS_DIR:-}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  CANDIDATE_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
  if [[ -d "${CANDIDATE_WS}/src" ]]; then
    WS_DIR="${CANDIDATE_WS}"
  else
    WS_DIR="$HOME/slam_ws"
  fi
fi
CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Release}
MERGE_INSTALL=${MERGE_INSTALL:-1}
EXTRA_CMAKE_ARGS=${EXTRA_CMAKE_ARGS:-}

log() { echo -e "[bootstrap] $*"; }
err() { echo -e "[bootstrap][ERROR] $*" >&2; }

if [[ ! -d "$WS_DIR/src" ]]; then
  err "Expected workspace at $WS_DIR with a src/ directory. Set WS_DIR or create it."
  exit 1
fi

# Detect Ubuntu version (optional)
. /etc/os-release || true
UBU_CODENAME=${VERSION_CODENAME:-}
log "Ubuntu codename: ${UBU_CODENAME:-unknown}"

# Ensure base tools
log "Installing base tools and ROS helpers via apt..."
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-pip python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool

# Ensure ROS is installed
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  err "/opt/ros/${ROS_DISTRO}/setup.bash not found. Install ROS ${ROS_DISTRO} first."
  err "Docs: https://docs.ros.org/en/${ROS_DISTRO}/Installation/Ubuntu-Install-Debs.html"
  exit 2
fi

# rosdep init/update
log "Initializing rosdep (may be already initialized)..."
sudo rosdep init 2>/dev/null || true
rosdep update

# Source ROS env
log "Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
# Work around set -u (nounset) conflicting with ROS setup files using
# unbound variables like AMENT_TRACE_SETUP_FILES.
# Temporarily disable nounset while sourcing, then restore.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# Install package dependencies
log "Installing package dependencies with rosdep..."
rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y

# Build
log "Building workspace at $WS_DIR (CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})..."
cd "$WS_DIR"
if [[ "$MERGE_INSTALL" == "1" ]]; then
  colcon build --merge-install --cmake-args \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
    -DBUILD_WITH_LDMRS_SUPPORT=OFF \
    -DENABLE_EMULATOR=OFF \
    ${EXTRA_CMAKE_ARGS}
else
  colcon build --cmake-args \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
    -DBUILD_WITH_LDMRS_SUPPORT=OFF \
    -DENABLE_EMULATOR=OFF \
    ${EXTRA_CMAKE_ARGS}
fi

log "Build complete. To use this overlay, run:"
log "  source $WS_DIR/install/setup.bash"
log "Tip: add the line above to your ~/.bashrc for convenience."
