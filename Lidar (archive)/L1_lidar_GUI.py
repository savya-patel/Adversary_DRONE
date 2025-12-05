"""
L1_lidar_GUI.py — Real-time GUI visualizer for SICK TiM LiDAR (270°)
Author: Boo Man & GPT-5

Usage:
------
# Choose local connection type via CLI (no tunnel required)
python3 L1_lidar_GUI.py --ethernet   # use Ethernet (default)
python3 L1_lidar_GUI.py --usb        # use USB (via L1_lidar_usb)

Ethernet quick setup:
---------------------
sudo ip addr add 168.254.15.100/16 dev enP8p1s0
sudo ip link set enP8p1s0 up
ping 168.254.15.1

USB quick notes:
----------------
- No need to specify /dev/tty*: USB handled by L1_lidar_usb (serial or bulk)
- If permissions block access, add a udev rule or run with sudo as a fallback
"""

import socket
import numpy as np
import time
import threading
import collections
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from L1_lidar_usb import Lidar as UsbLidar
from L1_lidar import Lidar as EthernetLidar

# ========== CONFIGURATION ==========
# Ethernet settings
SENSOR_IP = "168.254.15.1"
SENSOR_PORT = 2112

# Angular resolution settings
# TiM561 supports: 0.33° (810 points), 0.5° (540 points), or 1.0° (270 points)
# Options: '0.33', '0.5', or '1.0'
ANGULAR_RESOLUTION = '1.0'  # Default: 0.33° for maximum resolution
# ================================================== #


# --- GUI Class --- #
class LidarGUI:
    def __init__(self, lidar):
        self.lidar = lidar
        self.root = tk.Tk()
        self.root.title("SICK TiM561 LiDAR Visualizer (270°)")

        # Matplotlib figure
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111, polar=True)
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(1)  # Counterclockwise: left=-135°, right=+135°
        self.ax.set_thetalim(-np.pi * 3 / 4, np.pi * 3 / 4)
        self.ax.set_ylim(0, 4)  # 4 meter range
        self.ax.set_title("Live LiDAR Scan", va='bottom')

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.scatter = None
        self.update_plot()

    def update_plot(self):
        """Update polar plot with latest LiDAR scan."""
        if self.lidar.ds is not None:
            num_points = len(self.lidar.ds)
            # Angles from -135° (left) to +135° (right)
            angles = np.linspace(-135, 135, num_points) * np.pi / 180
            distances = self.lidar.ds

            self.ax.clear()
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(1)  # Counterclockwise: left=-135°, right=+135°
            self.ax.set_thetalim(-np.pi * 3 / 4, np.pi * 3 / 4)
            self.ax.set_ylim(0, 4)
            self.ax.scatter(angles, distances, s=3, c='cyan', alpha=0.8)
            self.ax.set_title(f"SICK TiM561 Live 270° Scan ({num_points} points)", va='bottom')
            self.ax.grid(True, alpha=0.3)
            self.canvas.draw()

        self.root.after(50, self.update_plot)  # refresh every 50ms for smoother updates

    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()


# --- Main Execution --- #
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Local LiDAR GUI over Ethernet or USB")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--ethernet", action="store_true", help="use Ethernet (default)")
    group.add_argument("--usb", action="store_true", help="use USB via L1_lidar_usb")
    parser.add_argument("--ip", default=SENSOR_IP, help="sensor IP for Ethernet mode")
    parser.add_argument("--port", type=int, default=SENSOR_PORT, help="sensor TCP port for Ethernet mode")
    args = parser.parse_args()

    mode = 'usb' if args.usb else 'ethernet'
    
    if mode == 'ethernet':
        # Use the proven L1_lidar.py implementation for Ethernet
        lidar = EthernetLidar(IP=args.ip)
        lidar.connect()
        processor = lidar.run()
        time.sleep(1)  # Allow connection to stabilize
        print(f"[+] Using Ethernet LiDAR at {args.ip}")
    else:
        # Use USB implementation
        lidar = UsbLidar()
        lidar.connect()
        processor = lidar.run()
        time.sleep(1)
        print("[+] Using USB LiDAR")

    gui = LidarGUI(lidar)
    try:
        gui.run()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        lidar.kill(processor)
