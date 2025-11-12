# Quick SLAM Commands

## ✅ Start Cartographer (Recommended)
```bash
cd ~/Adversary_DRONE/slam_ws

# Standard launch
./run_cartographer.sh

# With logging (for diagnostics)
./run_cartographer_log.sh

# Web visualization (in separate terminal)
./run_rosboard_sudo.sh
# Then open: http://10.250.240.81:8888
```

## 🛑 Stop Everything
```bash
cd ~/Adversary_DRONE/slam_ws

# Kill all SLAM processes (Cartographer, slam_toolbox, rf2o, everything)
./kill_all_slam.sh

# Interactive clean restart (clears maps, archives logs)
./clean_restart.sh
```

## 📊 Check Status
```bash
# See running nodes
ros2 node list

# See topics being published
ros2 topic list

# Check scan rate (should be ~20 Hz)
ros2 topic hz /scan

# Analyze Cartographer performance
./analyze_cartographer_log.sh
```

## ⚠️ DO NOT USE
- `slam_usb_with_odom.launch.py` - slam_toolbox is broken on Humble
- Any manual `ros2 launch slam_config slam_usb_with_odom` commands

## 📚 Documentation
- `CARTOGRAPHER_GUIDE.md` - Full Cartographer setup and usage
- `CONFIG_DRONE_VS_STATIONARY.md` - Configuration details
- `WEB_VISUALIZATION_GUIDE.md` - Rosboard setup
- `CARTOGRAPHER_PARAMS_GUIDE.md` - Parameter tuning

## 🚁 Current Configuration
- **Mode**: Flying drone (balanced translation + rotation)
- **LiDAR**: SICK TiM561 USB @ 20 Hz
- **SLAM**: Google Cartographer 2.0.9004
- **Config**: `cartographer_2d_lidar.lua` (balanced weights 10:40)
- **Visualization**: Rosboard on port 8888
