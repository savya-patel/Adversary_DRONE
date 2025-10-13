"""
L1_lidar_gui.py — Real-time GUI visualizer for SICK TiM561 LiDAR (270°)
Author: Boo Man & GPT-5

Before running:
----------------
# Ensure Ethernet IP and connection
sudo ip addr add 168.254.15.100/16 dev eth0
sudo ip link set eth0 up
ping 168.254.15.1

# Run this script
python3 L1_lidar_gui.py
"""

import socket
import numpy as np
import time
import threading
import collections
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# LiDAR configuration
SENSOR_IP = "168.254.15.1"
SENSOR_PORT = 2112
STX, ETX = b'\x02', b'\x03'


# --- LiDAR Helper Functions --- #
def bytes_from_socket(sock):
    """Yield bytes from the socket stream."""
    while True:
        data = sock.recv(256)
        for b in data:
            yield bytes([b])


def datagrams_from_socket(sock):
    """Generate datagrams starting with STX and ending with ETX."""
    byte_gen = bytes_from_socket(sock)
    while True:
        datagram = b''
        # Find STX
        for b in byte_gen:
            if b == STX:
                break
        # Read until ETX
        for b in byte_gen:
            if b == ETX:
                break
            datagram += b
        yield datagram


def parse_number(n):
    """Parse decimal or hex numbers from bytes."""
    try:
        return int(n, 16)
    except ValueError:
        return int(n)


def decode_datagram(datagram):
    """Extract scan data from the LiDAR datagram."""
    try:
        items = datagram.split(b' ')
        if items[0] != b'sSN' or items[1] != b'LMDscandata':
            return None

        num_data = parse_number(items[25])
        data = [parse_number(x) / 1000 for x in items[26:26 + num_data]]
        return np.array(data)
    except Exception:
        return None


# --- LiDAR Class --- #
class Lidar:
    def __init__(self, ip=SENSOR_IP, port=SENSOR_PORT):
        self.ip = ip
        self.port = port
        self.stop_flag = False
        self.ds = None
        self.socket = None
        self.datagrams_generator = None

    def connect(self):
        """Connect to LiDAR TCP socket."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.ip, self.port))
            print("[+] Connected to LiDAR")

            # Activate data streaming
            self.socket.send(b'\x02sEN LMDscandata 1\x03\0')
            self.datagrams_generator = datagrams_from_socket(self.socket)
        except Exception as e:
            print(f"[!] Connection error: {e}")

    def run(self):
        """Read continuous data from LiDAR."""
        print("[*] Starting LiDAR stream...")
        while not self.stop_flag:
            try:
                datagram = next(self.datagrams_generator)
                decoded = decode_datagram(datagram)
                if decoded is not None:
                    self.ds = decoded
            except Exception:
                pass
        print("[*] LiDAR stream stopped")

    def stop(self):
        """Stop LiDAR."""
        self.stop_flag = True
        if self.socket:
            self.socket.close()


# --- GUI Class --- #
class LidarGUI:
    def __init__(self, lidar: Lidar):
        self.lidar = lidar
        self.root = tk.Tk()
        self.root.title("SICK TiM561 LiDAR Visualizer (270°)")

        # Matplotlib figure
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, polar=True)
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_thetalim(-np.pi * 3 / 4, np.pi * 3 / 4)
        self.ax.set_title("Live LiDAR Scan", va='bottom')

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.scatter = None
        self.update_plot()

    def update_plot(self):
        """Update polar plot with latest LiDAR scan."""
        if self.lidar.ds is not None:
            angles = np.linspace(-135, 135, len(self.lidar.ds)) * np.pi / 180
            distances = self.lidar.ds

            self.ax.clear()
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            self.ax.set_thetalim(-np.pi * 3 / 4, np.pi * 3 / 4)
            self.ax.scatter(angles, distances, s=5, c='cyan')
            self.ax.set_title("SICK TiM561 Live 270° Scan", va='bottom')
            self.canvas.draw()

        self.root.after(100, self.update_plot)  # refresh every 100ms

    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()


# --- Main Execution --- #
if __name__ == "__main__":
    lidar = Lidar()
    lidar.connect()

    # Start LiDAR reading in a separate thread
    t = threading.Thread(target=lidar.run, daemon=True)
    t.start()

    # Start GUI
    gui = LidarGUI(lidar)
    try:
        gui.run()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        lidar.stop()
