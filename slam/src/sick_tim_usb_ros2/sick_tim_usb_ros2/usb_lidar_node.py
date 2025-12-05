#!/usr/bin/env python3
"""
ROS2 node publishing LaserScan from SICK TiM via USB.

Features:
- BEST_EFFORT QoS for SLAM consumers.
- Optional downsampling to ~1.0° increment.
- Clean startup/shutdown and throttled logging.
"""

import math
import time
import threading
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from .L1_lidar_usb import Lidar


class SimpleLidarPublisher(Node):
    """Minimal LaserScan publisher wrapping `L1_lidar_usb.Lidar`.

    Reads distances array and publishes a `sensor_msgs/LaserScan` with
    configurable downsampling to reduce bandwidth/compute.
    """

    def __init__(self) -> None:
        super().__init__('simple_lidar_publisher')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('log_throttle_sec', 2.0)
        self.declare_parameter('publish_points', 270)  # target points (270 ~ 1° over 270°)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.log_throttle_sec = float(self.get_parameter('log_throttle_sec').value)
        self.publish_points = int(self.get_parameter('publish_points').value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(LaserScan, self.topic, qos_profile)

        self.lidar = Lidar()
        self.lidar.connect()
        self.proc = self.lidar.run()
        time.sleep(1)

        self.latest_scan: Optional[np.ndarray] = None
        self.running: bool = True

        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()
        self.timer = self.create_timer(0.05, self._publish)  # 20 Hz

    def _read_loop(self) -> None:
        while self.running:
            if self.lidar.ds is not None:
                self.latest_scan = np.array(self.lidar.ds)
            time.sleep(0.01)

    def _publish(self) -> None:
        if self.latest_scan is None:
            self.get_logger().warn('No scan data available yet', throttle_duration_sec=5.0)
            return

        ranges = self.latest_scan
        # Optional downsampling for ~1.0° output (270 points across 270°)
        target_points = max(16, self.publish_points)
        if len(ranges) > target_points + 10:
            idx = np.linspace(0, len(ranges) - 1, target_points)
            ranges = np.interp(idx, np.arange(len(ranges)), ranges)
        num_points = int(len(ranges))

        # Mirror left/right if needed (keep existing behavior)
        ranges = ranges[::-1]

        angle_min = -135.0 * math.pi / 180.0
        angle_max = 135.0 * math.pi / 180.0
        angle_increment = (angle_max - angle_min) / (num_points - 1) if num_points > 1 else 0.0

        subs = self.publisher.get_subscription_count()
        self.get_logger().info(
            f'Publishing scan: {num_points} points, {math.degrees(angle_increment):.3f}° increment | subs={subs}',
            throttle_duration_sec=self.log_throttle_sec,
        )

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.scan_time = 0.05  # 20 Hz
        msg.time_increment = 0.0
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = [float(r) if self.range_min <= r <= self.range_max else float('inf') for r in ranges]
        msg.intensities = []
        self.publisher.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.lidar.kill(self.proc)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLidarPublisher()
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