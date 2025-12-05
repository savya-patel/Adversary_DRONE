# Adversary DRONE - Detection Scripts Documentation

## Overview

This document provides comprehensive documentation for all detection and streaming scripts in the Adversary DRONE system.

---

## Detection Scripts

### Core Detection Files

#### `detect_test_jetty.py`
**Purpose:** Base YOLO detection module that runs inference on video streams or webcams.

**What it does:**
- Runs YOLOv5 inference on camera input or video files
- Detects drones in the frame using the trained model
- Optionally draws bounding boxes and labels on frames
- Provides a frame callback mechanism for processing each frame
- Supports both display and headless modes
- Handles multiple detections per frame with confidence filtering

**Key Features:**
- GPU acceleration via CUDA (Jetson-compatible)
- Configurable confidence thresholds
- Frame callback support for real-time processing
- FPS calculation and display
- Center-of-frame marker for targeting reference

**Typical Use:**
- Baseline detection without drone control
- Testing YOLO model accuracy
- Debugging detection pipeline

---

#### `detect_test_attitude_jetty.py`
**Purpose:** YOLO detection with attitude control for the Cube Orange autopilot (TESTING VERSION with limited throttle).

**What it does:**
- Runs YOLO detection on the Jetson
- Connects directly to Cube Orange via serial (MAVLink)
- Converts bounding box errors into attitude commands (roll, pitch, yaw_rate, thrust)
- Sends attitude target commands to keep the target drone centered in the frame
- **Respects RC flight mode switches** - only performs autonomous control in GUIDED_NOGPS mode
- Maintains altitude when switching between flight modes (STABILIZE/ALT_HOLD/GUIDED_NOGPS)

**Thrust Range:**
- **MIN: 10% throttle**
- **MAX: 20% throttle**
- **Hover Point: 15% throttle**
- **Use Case:** Testing and bench testing (props OFF or blocked)

**Control Logic:**
- Horizontal error (X-axis) → Yaw rate commands (left/right rotation)
- Vertical error (Y-axis) → Thrust adjustments (up/down)
- Depth error (Z-axis, from bounding box size) → Pitch commands (forward/backward tilt)
- Roll always kept at 0 for stability

**Flight Modes:**
- **STABILIZE:** Manual control via RC, no autonomous commands
- **ALT_HOLD:** Manual control with altitude hold, no autonomous commands
- **GUIDED_NOGPS:** Autonomous control via YOLO detection active

**Running:**
```bash
python detect_test_attitude_jetty.py --weights best.pt --source 0 --max-det 1 --imgsz 320 --nosave --device 0
```

---

#### `detect_test_attitude_jetty_flight.py`
**Purpose:** YOLO detection with attitude control for the Cube Orange autopilot (FULL FLIGHT VERSION with unrestricted throttle).

**What it does:**
- Identical to `detect_test_attitude_jetty.py` but with full throttle range
- All same flight mode switching and altitude maintenance behavior
- Proper hover point at 50% throttle

**Thrust Range:**
- **MIN: 30% throttle**
- **MAX: 70% throttle**
- **Hover Point: 50% throttle**
- **Use Case:** Actual flight with props on

**Running:**
```bash
python detect_test_attitude_jetty_flight.py --weights best.pt --source 0 --max-det 1 --imgsz 320 --nosave --device 0
```

**⚠️ WARNING:** Only use this version with props on the drone and during actual flight tests. Always test with `detect_test_attitude_jetty.py` first.

---

#### `detect_yolov4tiny.py`
**Purpose:** Alternative YOLOv4-Tiny based detection module.

**What it does:**
- Runs YOLOv4-Tiny inference instead of YOLOv5
- Lighter weight model, faster inference on edge devices
- Similar frame callback structure to `detect_test_jetty.py`

**Use Case:**
- Embedded systems with limited resources
- When YOLOv4-Tiny accuracy is sufficient
- Jetson Nano or Raspberry Pi deployments

---

## SSH YOLO Feed Scripts

These scripts enable remote YOLO detection with server-client architecture for streaming video and optional drone control.

### `ssh_yolo_feed.py`
**Purpose:** Server-client streaming for YOLO detection with optional Cube Orange attitude control.

**What it does:**
- **Server Mode:** Runs on Jetson with camera/video source
  - Executes YOLO detection
  - Optionally connects to Cube Orange for autonomous control
  - Streams processed frames to client over network
  - Monitors RC flight mode changes
  - Only performs autonomous control when in GUIDED_NOGPS mode
  
- **Client Mode:** Runs on remote machine (laptop)
  - Receives and displays YOLO-processed video stream
  - Shows real-time detections with bounding boxes

**Network Architecture:**
```
Jetson (Server) --TCP Stream--> Laptop (Client)
   ↓
Cube Orange (Optional Attitude Control via Serial/MAVLink)
```

**Flight Mode Behavior:**
- Server connects to vehicle but does NOT force any mode
- Respects RC control entirely
- When RC switch is in STABILIZE or ALT_HOLD: autonomous control disabled
- When RC switch is in GUIDED_NOGPS: autonomous YOLO-based control active
- Maintains altitude during mode transitions

**Server Command (with attitude control):**
```bash
sudo /home/eagle/venv/bin/python3 ssh_yolo_feed.py \
  --server \
  --host 0.0.0.0 \
  --port 5000 \
  --device 0 \
  --quality 60 \
  --weights best.pt \
  --attitude-control \
  --serial /dev/ttyACM0 \
  --baud 57600 \
  --conf-thres 0.25
```

**Server Command (video only, no drone control):**
```bash
python3 ssh_yolo_feed.py \
  --server \
  --host 0.0.0.0 \
  --port 5000 \
  --device 0 \
  --quality 60 \
  --weights best.pt
```

**Client Command (from laptop, with SSH tunnel):**
```bash
# Terminal 1: Create SSH tunnel
ssh -L 5000:localhost:5000 eagle@<JETSON_IP>

# Terminal 2: Run client
python3 ssh_yolo_feed.py --client --host 127.0.0.1 --port 5000
```

**Arguments:**
- `--server`: Run as server (Jetson)
- `--client`: Run as client (laptop)
- `--host`: Server listening address
- `--port`: Network port (default: 5000)
- `--device`: Camera device ID (0 for default camera)
- `--quality`: JPEG compression quality (1-100, lower = smaller bandwidth)
- `--weights`: Path to YOLO model weights
- `--attitude-control`: Enable Cube Orange connection (server only)
- `--serial`: Serial port for Cube Orange (default: /dev/ttyACM0)
- `--baud`: Serial baud rate (default: 57600)
- `--conf-thres`: Confidence threshold for detections (default: 0.25)

---

### `ssh_yolo_feed_laptop.py`
**Purpose:** Lightweight laptop-based client for receiving YOLO video stream.

**What it does:**
- Connects to ssh_yolo_feed.py server
- Receives and displays YOLO-processed frames over network
- Minimal dependencies (no Cube Orange connection)
- Used for remote monitoring and testing

**Running:**
```bash
python3 ssh_yolo_feed_laptop.py --host <JETSON_IP> --port 5000
```

---

## Quick Start Guide

### 1. **Local Testing (Bench Test, Props OFF)**
```bash
cd Detect_Drone
python detect_test_attitude_jetty.py --weights best.pt --source 0 --max-det 1 --imgsz 320 --nosave --device 0
```
- Keep props removed or blocked
- Use RC to switch between STABILIZE/ALT_HOLD/GUIDED_NOGPS
- Observe console output for mode changes and control commands

### 2. **Remote Video Streaming (Jetson to Laptop)**
Terminal 1 (Jetson - Server):
```bash
python3 ssh_yolo_feed.py --server --host 0.0.0.0 --port 5000 --device 0 --quality 60 --weights best.pt
```

Terminal 2 (Laptop - SSH Tunnel):
```bash
ssh -L 5000:localhost:5000 eagle@<JETSON_IP>
```

Terminal 3 (Laptop - Client):
```bash
python3 ssh_yolo_feed_laptop.py --host 127.0.0.1 --port 5000
```

### 3. **Remote Autonomous Control (Jetson Connected to Cube Orange)**
Terminal 1 (Jetson - Server with Attitude Control):
```bash
sudo /home/eagle/venv/bin/python3 ssh_yolo_feed.py \
  --server --host 0.0.0.0 --port 5000 --device 0 --quality 60 --weights best.pt \
  --attitude-control --serial /dev/ttyACM0 --baud 57600
```

Terminal 2 & 3: Same as video streaming above

Then:
1. Take off manually in STABILIZE mode using RC
2. Switch RC to GUIDED_NOGPS when ready for autonomous control
3. Switch back to STABILIZE/ALT_HOLD for manual override
4. Land manually in STABILIZE mode

---

## File Comparison Quick Reference

| File | Mode | Throttle Range | Autopilot | Use Case |
|------|------|---|---|---|
| `detect_test_jetty.py` | Detection only | N/A | No connection | Baseline YOLO testing |
| `detect_test_attitude_jetty.py` | With control | 10-20% | Direct connect | Bench testing (props OFF) |
| `detect_test_attitude_jetty_flight.py` | With control | 30-70% | Direct connect | Actual flight (props ON) |
| `ssh_yolo_feed.py` (video only) | Detection only | N/A | Optional | Remote video streaming |
| `ssh_yolo_feed.py` (with attitude) | With control | 10-20% | Connects | Remote control (bench test) |
| `ssh_yolo_feed_laptop.py` | Viewer | N/A | No connection | Remote video display |

---

## Flight Mode Control Overview

### How Flight Mode Switching Works

All attitude control scripts (`detect_test_attitude_jetty.py`, `detect_test_attitude_jetty_flight.py`, `ssh_yolo_feed.py` with `--attitude-control`) follow the same flight mode logic:

1. **At Startup:**
   - Script connects to Cube Orange
   - Does NOT force any mode change
   - Respects whatever mode the RC is currently in

2. **During Operation:**
   - Continuously monitors the current flight mode
   - When mode is **GUIDED_NOGPS**: autonomous YOLO control is ACTIVE
   - When mode is **STABILIZE** or **ALT_HOLD**: autonomous control is INACTIVE (manual only)
   - Altitude is automatically maintained when switching modes

3. **Mode Transitions:**
   - RC switch toggles between modes
   - Script detects mode change and adjusts behavior
   - Last known thrust value is preserved to maintain altitude

### Testing Flight Modes

**Prerequisites:**
- RC controller configured with mode switch
- Autopilot parameters allow RC to control flight modes
- Ground station (QGroundControl/Mission Planner) connected for verification

**Test Procedure:**
1. Start the script with `--attitude-control`
2. Observe console: "[server] Ready - Waiting for RC control mode changes..."
3. Toggle RC mode switch to GUIDED_NOGPS
4. Observe console: "[server] Flight mode changed to: GUIDED_NOGPS"
5. YOLO autonomous control is now active
6. Toggle RC mode switch to STABILIZE or ALT_HOLD
7. Observe console: "[server] Flight mode changed to: STABILIZE"
8. Autonomous control stops; only manual control via RC
9. In ground station, verify mode indicator matches current flight mode

---

## Notes

- Always start with the 10-20% throttle version for testing before flying
- The RC flight mode switch has full authority - script respects it
- Altitude is maintained during mode transitions
- Serial connection to Cube Orange requires appropriate permissions (use `sudo` on Linux)
- Network streaming bandwidth is reduced with `--quality` parameter (lower = smaller)
- Confidence threshold affects both detection quantity and visibility of boxes
- When using `ssh_yolo_feed.py`, ensure SSH tunnel is active before starting client
- For remote operations, use screen or tmux to manage multiple terminal sessions on Jetson
