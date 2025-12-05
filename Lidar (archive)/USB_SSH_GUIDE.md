## LiDAR USB + SSH Tunnel: Quickstart

This is the shortest path to run the LiDAR locally over USB or view it remotely over SSH.

---

## 1) Local USB GUI (Jetson)

```bash
cd ~/Adversary_DRONE/Lidar_Lab
sudo python3 L1_lidar_GUI.py --usb --resolution 1.0
```

Tip: resolution 1.0 = 270 pts @ 50 Hz (fastest).

---

## 2) Remote GUI via SSH Tunnel

### On Jetson (server)
```bash
cd ~/Adversary_DRONE/Lidar_Lab
sudo python3 L1_lidar_GUI_tunnel.py --server --port 5100
```

### On your laptop (client)
Copy the client once:

```bash

scp eagle@<jetson-ip>:/home/eagle/Adversary_DRONE/Lidar_Lab/L1_lidar_GUI_client.py $env:USERPROFILE\Downloads\
```

Open a tunnel and run the client:

```bash
# Windows PowerShell (two terminals)
ssh -L 5100:127.0.0.1:5100 eagle@10.250.240.81
python $env:USERPROFILE\Downloads\L1_lidar_GUI_client.py --host 127.0.0.1 --port 5100
```

Optional (one-time): set up key-based SSH to skip passwords.

```bash
# Linux/Mac
ssh-keygen -t ed25519 && ssh-copy-id eagle@<jetson-ip>

# Windows PowerShell
ssh-keygen -t ed25519; type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh eagle@<jetson-ip> "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```

---

## Troubleshooting (quick)

- LiDAR not found: `lsusb | grep -i sick` → should show 19a2:5001; run with `sudo` for USB.
- Client can’t connect: verify tunnel is up (port 5100) and server is running on Jetson.
- Frozen GUI: restart tunnel and client; ensure Jetson terminal shows "client connected".

---
# In Terminal 1, stop current SLAM (Ctrl+C)

# Restart with fixed QoS:
cd ~/Adversary_DRONE/slam_ws
sudo bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_config slam_usb.launch.py"# In Terminal 1, stop current SLAM (Ctrl+C)

# Restart with fixed QoS:
cd ~/Adversary_DRONE/slam_ws
sudo bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_config slam_usb.launch.py"cd ~/Adversary_DRONE/slam_ws
./run_rosboard.sh
That’s it. For Ethernet mode, run `L1_lidar_GUI.py` without `--usb` and provide your IP/port.
