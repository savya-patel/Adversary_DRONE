#!/usr/bin/env python3
"""
SICK TIM561 LiDAR Scan Visualizer
Shows scan orientation, point positions, and configurable parameters
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import tkinter as tk
from tkinter import ttk
import threading
import math

class LidarScanGUI(Node):
    def __init__(self):
        super().__init__('lidar_scan_gui')
        
        # Subscribe to scan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Current scan data
        self.current_scan = None
        self.lock = threading.Lock()
        
        # Create GUI
        self.root = tk.Tk()
        self.root.title("SICK TIM561 LiDAR Scan Viewer")
        self.root.geometry("1000x700")
        self.setup_gui()
        
    def setup_gui(self):
        """Create the GUI layout"""
        # Title
        title = tk.Label(self.root, text="SICK TIM561 LiDAR - Real-Time Scan", 
                        font=("Arial", 16, "bold"), bg="#1a1a2e", fg="white")
        title.pack(fill=tk.X, pady=5)
        
        # Main container
        container = tk.Frame(self.root)
        container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel - Scan visualization
        left_frame = tk.Frame(container)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # Canvas for scan drawing
        self.canvas = tk.Canvas(left_frame, width=600, height=600, bg="black")
        self.canvas.pack()
        
        # Right panel - Parameters and info
        right_frame = tk.Frame(container, width=350)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=5)
        
        # TIM561 Parameters section
        params_frame = ttk.LabelFrame(right_frame, text="TIM561 Parameters", padding=10)
        params_frame.pack(fill=tk.X, pady=5)
        
        params_info = [
            ("Model:", "SICK TIM561-2050101"),
            ("Field of View:", "270° (-135° to +135°)"),
            ("Angular Resolution:", "Configurable"),
            ("  • High (0.33°):", "811 points"),
            ("  • Medium (0.5°):", "541 points"),
            ("  • Low (1.0°):", "271 points"),
            ("Range:", "0.05m to 10.0m"),
            ("Scan Rate:", "15 Hz"),
            ("Interface:", "USB (Serial/Bulk)"),
        ]
        
        for label, value in params_info:
            row = tk.Frame(params_frame)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, font=("Arial", 9, "bold"), width=20, anchor="w").pack(side=tk.LEFT)
            tk.Label(row, text=value, font=("Arial", 9), anchor="w").pack(side=tk.LEFT)
        
        # Current scan info
        info_frame = ttk.LabelFrame(right_frame, text="Current Scan Data", padding=10)
        info_frame.pack(fill=tk.X, pady=5)
        
        self.info_labels = {}
        info_items = [
            ("Points Count:", "points"),
            ("Angular Step:", "step"),
            ("Angle Min:", "angle_min"),
            ("Angle Max:", "angle_max"),
            ("Range Min:", "range_min"),
            ("Range Max:", "range_max"),
            ("Closest Point:", "closest"),
            ("Farthest Point:", "farthest"),
            ("Scan Time:", "scan_time"),
        ]
        
        for label, key in info_items:
            row = tk.Frame(info_frame)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, font=("Arial", 9, "bold"), width=15, anchor="w").pack(side=tk.LEFT)
            self.info_labels[key] = tk.Label(row, text="--", font=("Arial", 9), fg="blue")
            self.info_labels[key].pack(side=tk.LEFT)
        
        # Orientation diagram
        orient_frame = ttk.LabelFrame(right_frame, text="Orientation", padding=10)
        orient_frame.pack(fill=tk.X, pady=5)
        
        orient_canvas = tk.Canvas(orient_frame, width=300, height=150, bg="white")
        orient_canvas.pack()
        
        # Draw orientation diagram
        cx, cy = 150, 100
        # Draw arc
        orient_canvas.create_arc(50, 20, 250, 180, start=45, extent=90, 
                                 outline="blue", width=2, style=tk.ARC)
        # Center point
        orient_canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="red")
        orient_canvas.create_text(cx, cy+15, text="LiDAR", font=("Arial", 8))
        # Angles
        orient_canvas.create_line(cx, cy, cx-70, cy-70, arrow=tk.LAST, fill="green", width=2)
        orient_canvas.create_text(cx-85, cy-75, text="-135°\n(Left)", font=("Arial", 8))
        orient_canvas.create_line(cx, cy, cx, cy-80, arrow=tk.LAST, fill="orange", width=2)
        orient_canvas.create_text(cx, cy-95, text="0°\n(Front)", font=("Arial", 8))
        orient_canvas.create_line(cx, cy, cx+70, cy-70, arrow=tk.LAST, fill="green", width=2)
        orient_canvas.create_text(cx+85, cy-75, text="+135°\n(Right)", font=("Arial", 8))
        orient_canvas.create_text(150, 140, text="270° Field of View", font=("Arial", 10, "bold"))
        
        # Control info
        control_frame = ttk.LabelFrame(right_frame, text="Configuration", padding=10)
        control_frame.pack(fill=tk.X, pady=5)
        
        config_text = """
To change angular resolution, edit:
/src/sick_tim_usb_ros2/config/tim_params.yaml

angular_resolution: '1.0'  # or '0.5', '0.33'

Then rebuild with:
colcon build --packages-select sick_tim_usb_ros2
        """
        tk.Label(control_frame, text=config_text, font=("Courier", 8), 
                justify=tk.LEFT, anchor="w").pack()
        
        # Start update loop
        self.update_display()
        
    def scan_callback(self, msg):
        """Process incoming scan data"""
        with self.lock:
            self.current_scan = msg
        
    def update_display(self):
        """Update the display with latest scan data"""
        with self.lock:
            scan = self.current_scan
        
        if scan is not None:
            # Update info labels
            num_points = len(scan.ranges)
            angle_step = math.degrees(scan.angle_increment)
            angle_min = math.degrees(scan.angle_min)
            angle_max = math.degrees(scan.angle_max)
            
            valid_ranges = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
            closest = min(valid_ranges) if valid_ranges else 0.0
            farthest = max(valid_ranges) if valid_ranges else 0.0
            
            self.info_labels['points'].config(text=f"{num_points}")
            self.info_labels['step'].config(text=f"{angle_step:.2f}°")
            self.info_labels['angle_min'].config(text=f"{angle_min:.1f}°")
            self.info_labels['angle_max'].config(text=f"{angle_max:.1f}°")
            self.info_labels['range_min'].config(text=f"{scan.range_min:.2f} m")
            self.info_labels['range_max'].config(text=f"{scan.range_max:.2f} m")
            self.info_labels['closest'].config(text=f"{closest:.2f} m")
            self.info_labels['farthest'].config(text=f"{farthest:.2f} m")
            self.info_labels['scan_time'].config(text=f"{scan.scan_time:.3f} s")
            
            # Draw scan
            self.draw_scan(scan)
        
        # Schedule next update
        self.root.after(50, self.update_display)  # 20 Hz update
        
    def draw_scan(self, scan):
        """Draw the LiDAR scan on canvas"""
        self.canvas.delete("all")
        
        # Canvas parameters
        width = 600
        height = 600
        cx, cy = width // 2, height // 2
        scale = 50  # pixels per meter
        
        # Draw grid
        for i in range(1, 11):
            r = i * scale
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline="#333333")
            if i % 2 == 0:
                self.canvas.create_text(cx+r, cy, text=f"{i}m", fill="gray", anchor="w")
        
        # Draw angle lines
        for angle_deg in [-135, -90, -45, 0, 45, 90, 135]:
            angle_rad = math.radians(angle_deg)
            x = cx + 250 * math.sin(angle_rad)
            y = cy - 250 * math.cos(angle_rad)
            color = "yellow" if angle_deg == 0 else "gray"
            self.canvas.create_line(cx, cy, x, y, fill=color, dash=(2, 2))
            # Label
            label_x = cx + 270 * math.sin(angle_rad)
            label_y = cy - 270 * math.cos(angle_rad)
            self.canvas.create_text(label_x, label_y, text=f"{angle_deg}°", fill="white", font=("Arial", 8))
        
        # Draw center
        self.canvas.create_oval(cx-8, cy-8, cx+8, cy+8, fill="red", outline="white")
        self.canvas.create_text(cx, cy+20, text="LiDAR", fill="white", font=("Arial", 10, "bold"))
        
        # Draw scan points
        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if scan.range_min <= r <= scan.range_max:
                # Convert to canvas coordinates
                x = cx + r * scale * math.sin(angle)
                y = cy - r * scale * math.cos(angle)
                
                # Color based on distance
                if r < 1.0:
                    color = "red"
                elif r < 3.0:
                    color = "orange"
                elif r < 6.0:
                    color = "yellow"
                else:
                    color = "green"
                
                # Draw point
                size = 3
                self.canvas.create_oval(x-size, y-size, x+size, y+size, fill=color, outline=color)
            
            angle += scan.angle_increment
        
        # Draw info overlay
        points_displayed = len([r for r in scan.ranges if scan.range_min <= r <= scan.range_max])
        self.canvas.create_text(10, 10, text=f"Points: {points_displayed}/{len(scan.ranges)}", 
                               fill="white", anchor="nw", font=("Arial", 10, "bold"))
        
    def run(self):
        """Run the GUI and ROS2 spin in separate threads"""
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()
        
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gui = LidarScanGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
