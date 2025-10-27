#!/usr/bin/env python3
"""
Rosboard Custom Visualization for SICK TIM561 LiDAR
Publishes scan data visualization to rosboard web interface
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import numpy as np
import math

class RosboardLidarPublisher(Node):
    def __init__(self):
        super().__init__('rosboard_lidar_viz')
        
        # Subscribe to scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publish visualization data (for custom rosboard widget)
        self.viz_pub = self.create_publisher(
            String,
            '/rosboard/lidar_viz',
            10
        )
        
        self.get_logger().info('Rosboard LiDAR visualization publisher started')
        self.get_logger().info('Open rosboard and add /rosboard/lidar_viz topic')
        
    def scan_callback(self, msg: LaserScan):
        """Process scan and publish visualization data"""
        
        # Convert LaserScan to visualization format
        viz_data = {
            'type': 'polar_scan',
            'timestamp': self.get_clock().now().to_msg().sec,
            'ranges': [],
            'angles': [],
            'colors': [],
            'metadata': {
                'points': 0,
                'min_distance': float('inf'),
                'close_count': 0,
                'fov_deg': 0,
                'resolution_deg': 0,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            }
        }
        
        valid_points = 0
        min_dist = float('inf')
        close_count = 0
        
        for i, range_val in enumerate(msg.ranges):
            # Skip invalid ranges
            if not (msg.range_min <= range_val <= msg.range_max) or not math.isfinite(range_val):
                continue
            
            valid_points += 1
            if range_val < min_dist:
                min_dist = range_val
            if range_val < 2.0:
                close_count += 1
            
            # Calculate angle
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)
            
            # Color based on distance
            if range_val < 1.0:
                color = '#d32f2f'  # Red
            elif range_val < 2.0:
                color = '#f57c00'  # Orange
            elif range_val < 6.0:
                color = '#fbc02d'  # Yellow
            else:
                color = '#16c79a'  # Green
            
            viz_data['ranges'].append(round(range_val, 3))
            viz_data['angles'].append(round(angle_deg, 2))
            viz_data['colors'].append(color)
        
        # Update metadata
        viz_data['metadata']['points'] = valid_points
        viz_data['metadata']['min_distance'] = round(min_dist, 3) if math.isfinite(min_dist) else None
        viz_data['metadata']['close_count'] = close_count
        viz_data['metadata']['fov_deg'] = round(math.degrees(msg.angle_max - msg.angle_min), 1)
        viz_data['metadata']['resolution_deg'] = round(math.degrees(msg.angle_increment), 3)
        
        # Publish as JSON string
        viz_msg = String()
        viz_msg.data = json.dumps(viz_data)
        self.viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RosboardLidarPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
