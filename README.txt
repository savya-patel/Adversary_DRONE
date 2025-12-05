ADVERSARY TRACKING DRONE

Project: Capstone under ETID and GCRI (Cyber Research Institute) at Texas A&M.

System: Runs on NVIDIA Jetson companion computer to send attitude commands to flight computer using:
- Drone tracking: YOLOv5 custom vision model for real-time detection
- Obstacle detection: SICK TiM561 2D LiDAR and ToF sensors
- SLAM: Google Cartographer for real-time 2D mapping

================================================================================
FOLDER INVENTORY
================================================================================

slam/ - Main ROS 2 SLAM Workspace (Active)
--------
Minimal, production-ready workspace for running Cartographer SLAM with SICK TiM561 LiDAR.

Contents:
  src/sick_tim_usb_ros2/     USB/SOPAS driver for SICK TiM LiDAR (publishes /scan)
  src/slam_config/           Cartographer launch files and 2D configuration
  run_cartographer.sh        Start Cartographer + LiDAR + TF
  run_rosboard.sh            Start web UI (port 8888) for map/scan visualization
  SETUP.md                   Complete setup guide from scratch
  CUSTOMIZE.md               Customization reference for all tunable parameters

Use for:
  - Running Cartographer 2D SLAM
  - Publishing /map and /scan topics
  - Testing LiDAR driver
  - Web-based visualization via rosboard

Status: Clean, formalized, ready for deployment


slam_ws/ - Original ROS 2 SLAM Workspace (Archive/Reference)
--------
Extended development workspace from earlier iterations. Contains extra files and 
experiments with SLAM Toolbox (before switching to Cartographer).

Contents:
  src/sick_tim_usb_ros2/     Original USB driver (same as in slam/)
  src/slam_config/           Extended config with SLAM Toolbox YAMLs and rviz configs
  build/, install/, log/     Build artifacts

When to use:
  - Fallback: If slam/ has issues, try copying files from slam_ws/src/slam_config/rviz/
  - Reference: Historical configuration files and parameter tuning examples
  - Legacy: Use config/mapper_params_online_async*.yaml if switching to SLAM Toolbox

Status: Archive; do not modify unless troubleshooting


Detect_Drone/ - YOLOv5 Drone Tracking & Attitude Control
-----------
Real-time vision-based drone tracking with attitude command generation.

Contents:
  ssh_yolo_feed.py           Server-side detection/tracking via SSH tunnel (Jetson)
  ssh_yolo_feed_laptop.py    Client-side viewer (runs on laptop/ground station)
  best.pt                    Pre-trained YOLOv5 weights (custom dataset)
  utils/                     YOLOv5 utilities, post-processing, and helpers
  models/                    YOLOv5 architecture definitions
  data/                      Dataset configs (drone.yaml, etc.)

  Note: the ssh yolo feeds were made by Eagle Eye, and the best.pt pretrained weight comes from 
  the Github Repo tusharsarkar3/Detect_Drone. The utils/, models/, and data/ files also came from 
  the tusharsarkar3/Detect_Drone repo.

Key Features:
  - Attitude control: Reads /dev/ttyACM0 (Cube Orange autopilot) and sends attitude targets
  - SSH tunnel: Streams video to ground station for real-time monitoring
  - Custom dataset: Trained on adversary drone images for high-accuracy detection

Usage:
  sudo /home/eagle/venv/bin/python3 ssh_yolo_feed.py \
    --server --host 0.0.0.0 --port 5000 \
    --weights best.pt --attitude-control --serial /dev/ttyACM0

Status: Clean, formalized, ready for deployment

Lidar_Lab/ - LiDAR Development & Experiments (Archive)
----------
Experimental scripts and prototypes for SICK TiM and has MXET 300 files.

Contents:
  L1_lidar.py                USB direct access via PyUSB (prototype)
  L1_lidar_usb.py            Cleaned version; SOPAS binary frame parsing and USB bulk
  L1_lidar_web_viz.py        Web-based LiDAR visualization (early prototype)
  L1_lidar_GUI*.py           GUI-based visualization (various iterations)

History:
  - MXet 300: Original LiDAR; required Ethernet connection
  - SICK TiM561: Switched to USB for simpler integration; added custom USB/SOPAS driver
  - Outcome: Refined code moved to slam/src/sick_tim_usb_ros2/L1_lidar_usb.py

When to use:
  - Reference: How USB bulk SOPAS communication works
  - Prototyping: Alternative visualization or direct USB testing
  - Troubleshooting: Low-level LiDAR connection diagnostics

Status: Archive; core functionality ported to ROS 2 node


yolov4tiny/ - YOLOv4-Tiny Weights & Config 
----------
Was parallel workstream to YOLOv5 not function on Jetson yet made to replace YoloV5 with less computaionally hungry model.

Contents:
  yolov4-tiny-custom_*.weights  Pre-trained weights (various epochs)
  yolov4-tiny-custom.cfg        Model architecture
  obj.names                      Class labels
  test*.py                       Standalone inference scripts

Status: Non functional, needs work


runs/ - Experiment & Training Outputs when running Yolo
----
Logs, weights, and results from YOLOv5 training runs these are not important.

Contents:
  detect/                    Detection outputs (labeled images, inference logs)
  Training artifacts         Weights, loss plots, etc.

Status: Accumulates during experimentation; safe to clean if storage is needed.


