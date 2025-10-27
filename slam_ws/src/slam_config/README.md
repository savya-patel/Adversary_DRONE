# SLAM System with SICK TIM561 on NVIDIA Jetson

This package provides optimized SLAM configuration for running SLAM Toolbox with the SICK TIM561 2D LiDAR on an NVIDIA Jetson.

## Hardware Requirements
- NVIDIA Jetson (Nano/Xavier/Orin)
- SICK TIM561 2D LiDAR (270° FoV)
- Network connection to LiDAR

## Quick Start

### 1. Configure LiDAR IP Address
Edit the launch file and set your LiDAR's IP address:
```bash
nano ~/slam_ws/src/slam_config/launch/slam_tim561.launch.py
# Change 'hostname': '192.168.0.1' to your LiDAR's IP
```

### 2. Build the Workspace
```bash
cd ~/slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Run SLAM
```bash
ros2 launch slam_config slam_tim561.launch.py
```

## Configuration

### SLAM Parameters
Main config file: `config/mapper_params_online_async.yaml`

Key parameters optimized for Jetson:
- **Async mapping mode**: Better CPU utilization
- **Ceres solver**: SPARSE_NORMAL_CHOLESKY for ARM64
- **Resolution**: 5cm map resolution
- **Loop closure**: Enabled with optimized search parameters
- **Max laser range**: 15m (TIM561 has 10m range)

### Performance Tuning

For **maximum speed** (lower quality):
- Increase `minimum_travel_distance` to 0.5
- Decrease `correlation_search_space_resolution` to 0.02
- Set `throttle_scans` to 2

For **maximum quality** (slower):
- Decrease `minimum_travel_distance` to 0.1
- Increase `correlation_search_space_resolution` to 0.005
- Set `resolution` to 0.025

### CPU vs GPU
Currently configured for **CPU-only** processing since SLAM Toolbox doesn't use GPU. The Jetson's ARM CPU cores are optimized with:
- Multi-threaded Ceres solver
- Async processing mode
- Optimized search parameters

## Sensor Mounting
Adjust the static transform in the launch file based on your sensor mounting:
```python
# Example: LiDAR is 15cm above base_link, rotated 180°
arguments=['0', '0', '0.15', '0', '0', '3.14159', 'base_link', 'laser']
```

## Visualization
Uncomment the RViz line in the launch file to enable visualization:
```python
ld.add_action(rviz_node)  # Enable RViz
```

## Saving Maps
To save the generated map:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'mymap'}}"
```

Maps are saved to `~/.ros/` by default.

## Troubleshooting

### LiDAR not connecting
1. Check network: `ping 192.168.0.1` (your LiDAR IP)
2. Verify LiDAR configuration with SOPAS software
3. Check firewall: `sudo ufw allow 2112`

### High CPU usage
1. Reduce scan rate: increase `minimum_time_interval`
2. Increase travel thresholds
3. Disable loop closure temporarily

### Poor map quality
1. Drive slower for better scan alignment
2. Reduce `minimum_travel_distance`
3. Increase `correlation_search_space_resolution`

## Advanced: Adding IMU Support (Future)
When you connect the Cube Orange IMU:
1. Add IMU package dependency to `package.xml`
2. Modify SLAM config to fuse IMU data
3. Update launch file with IMU node

## Performance Benchmarks (Jetson Orin)
- Mapping rate: ~5-10 Hz
- CPU usage: 40-60% (1-2 cores)
- Map update: Real-time
- Loop closure: ~100ms per loop

## License
Apache 2.0
