# Rebuild ignored artifacts (build/install/log) on a new machine

This workspace only pushes source code. The generated directories `build/`, `install/`, and `log/` are git-ignored. Use this guide to install dependencies and rebuild everything from source on a fresh computer.

Tested on Ubuntu 22.04 (Jetson) with ROS 2 Humble. Adapt ROS distro names if you use a different version.

---

## TL;DR one-liner

Prefer the bootstrap script to do everything automatically:

```bash
cd ~/Adversary_DRONE/slam_ws
WS_DIR=~/Adversary_DRONE/slam_ws ROS_DISTRO=humble CMAKE_BUILD_TYPE=Release ./scripts/bootstrap_build.sh
```

Then source:

```bash
source ~/Adversary_DRONE/slam_ws/install/setup.bash
```

---

## 1) Prereqs: ROS 2 + dev tools

```bash
# If ROS 2 Humble is not installed yet (Ubuntu 22.04):
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# Ensure you can source /opt/ros/humble/setup.bash

# Base build tools and ROS helpers
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-pip python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool

# Initialize rosdep once per machine
sudo rosdep init || true
rosdep update
```

---

## 2) Get the workspace

```bash
# This workspace lives at:
cd ~/Adversary_DRONE/slam_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

---

## 3) Install package dependencies

Let rosdep install all declared dependencies from `package.xml` (e.g., slam_toolbox, rviz2, tf2_ros, etc.).

```bash
cd ~/Adversary_DRONE/slam_ws
rosdep install --from-paths src --ignore-src -r -y
```

If rosdep reports missing keys, make sure your ROS distro’s apt repository is set up correctly and try again.

---

## 4) Build

```bash
cd ~/Adversary_DRONE/slam_ws
# Recommended: release build for faster binaries
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or minimal defaults
# colcon build
```

Notes:
- `build/`, `install/` and `log/` will be created automatically.
- `sick_scan_xd` optional features (LDMRS, emulator) are OFF by default; if you need them, pass cmake flags, e.g.:

```bash
colcon build --merge-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_LDMRS_SUPPORT=OFF \
  -DENABLE_EMULATOR=OFF
```

---

## 5) Use the workspace

```bash
# Source the overlay (do this in each new shell or add to ~/.bashrc)
source ~/Adversary_DRONE/slam_ws/install/setup.bash

# Example: verify packages are visible
ros2 pkg list | grep -E 'sick_scan_xd|slam_toolbox|slam_config'
```

---

## 6) Helpful apt packages (common runtime tools)

```bash
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-tf2-ros
```

These may already be satisfied via rosdep; listing them here for clarity.

---

## 7) Troubleshooting

- Missing dependencies: rerun `rosdep install --from-paths src --ignore-src -r -y` after sourcing `/opt/ros/<distro>/setup.bash`.
- Wrong ROS distro: substitute `humble` with your version (e.g., `iron`, `jazzy`).
- Slow/low-memory builds (Jetson): try `-DCMAKE_BUILD_TYPE=Release` and close heavy apps; consider adding swap.
- Clean rebuild: remove previous artifacts then rebuild
  ```bash
  rm -rf build/ install/ log/
  colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

---

## 8) Why these files aren’t in Git

To keep the repo small and under GitHub’s per-file limit (100 MB), we exclude large generated artifacts via `.gitignore`:
- `build/` (compiled objects and libraries)
- `install/` (install tree)
- `log/` (colcon/ROS logs)

Rebuilding restores everything deterministically from the sources in `src/`.
