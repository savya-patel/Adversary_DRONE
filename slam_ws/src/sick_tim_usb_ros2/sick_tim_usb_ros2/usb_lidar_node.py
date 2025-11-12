#!/usr/bin/env python3
import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from .L1_lidar_usb import Lidar  # reuse your USB LiDAR class

class SimpleLidarPublisher(Node):
    def __init__(self):
        super().__init__('simple_lidar_publisher')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('log_throttle_sec', 2.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.log_throttle_sec = float(self.get_parameter('log_throttle_sec').value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher = self.create_publisher(LaserScan, self.topic, qos_profile)

        self.lidar = Lidar()
        self.lidar.connect()
        self.proc = self.lidar.run()
        time.sleep(1)

        self.latest_scan = None
        self.running = True

        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()
        self.timer = self.create_timer(0.05, self._publish)  # 20 Hz

    def _read_loop(self):
        while self.running:
            if self.lidar.ds is not None:
                self.latest_scan = np.array(self.lidar.ds)
            time.sleep(0.01)

    def _publish(self):
        if self.latest_scan is None:
            self.get_logger().warn('No scan data available yet', throttle_duration_sec=5.0)
            return

        ranges = self.latest_scan
        num_points = len(ranges)

        # Mirror left/right to match GUI/web if needed
        ranges = ranges[::-1]

        angle_min = -135.0 * math.pi / 180.0
        angle_max = 135.0 * math.pi / 180.0
        angle_increment = (angle_max - angle_min) / (num_points - 1) if num_points > 1 else 0.0
        
        # Log every publish for debug
        subs = self.publisher.get_subscription_count()
        # Provide a message (first positional argument) and keep throttle_duration_sec as a kwarg
        self.get_logger().info(
            f'Publishing scan: {num_points} points, {math.degrees(angle_increment):.3f}° increment | subs={subs}',
            throttle_duration_sec=self.log_throttle_sec
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
        # clamp values
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

"""#!/usr/bin/env python3
import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

# Minimal embedded Lidar implementation; adapted from L1_lidar_usb.py
# Supports USB serial (/dev/ttyACM*) or PyUSB bulk fallback.

import glob
import serial

try:
    import usb.core
    import usb.util
    HAVE_USB = True
except Exception:
    HAVE_USB = False

STX, ETX = b'\x02', b'\x03'


def parse_number(nbr_str: bytes) -> int:
    try:
        return int(nbr_str, 16)
    except ValueError:
        return int(nbr_str)


def decode_datagram(datagram: bytes):
    try:
        items = datagram.split(b' ')
        if items[0] != b'sSN' or items[1] != b'LMDscandata':
            return None
        step_width = parse_number(items[24])  # in 1/10000 degree
        count = parse_number(items[25])
        data = [parse_number(x) / 1000.0 for x in items[26:26 + count]]  # meters
        return {
            'AngularStepWidth': step_width,
            'NumberOfData': count,
            'Data': data,
        }
    except Exception:
        return None


def sopas_command(cmd_ascii: bytes) -> bytes:
    return STX + cmd_ascii + ETX


def find_serial_port() -> str | None:
    candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    return candidates[0] if candidates else None


def bytes_from_serial(ser: serial.Serial):
    while True:
        data = ser.read(256)
        if not data:
            yield from []
        else:
            for b in data:
                yield bytes([b])


def datagrams_from_serial(ser: serial.Serial):
    byte_gen = bytes_from_serial(ser)
    while True:
        # find STX
        for b in byte_gen:
            if b == STX:
                break
        datagram = b''
        for b in byte_gen:
            if b == ETX:
                break
            datagram += b
        if datagram:
            yield datagram


class UsbTimPublisher(Node):
    def __init__(self):
        super().__init__('usb_tim_publisher')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('angular_resolution', '1.0')  # '0.33','0.5','1.0'
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 3.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.angular_resolution = self.get_parameter('angular_resolution').get_parameter_value().string_value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        # QoS profile matching SLAM Toolbox expectations (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher = self.create_publisher(LaserScan, self.topic, qos_profile)

        self._usb = None
        self._ser = None
        self._buf = b''
        self._scan_data = None  # Store full decoded scan info (Data + AngularStepWidth)

        self.get_logger().info('Connecting to SICK TiM over USB...')
        self._connect()

        # Publish timer
        self._timer = self.create_timer(0.02, self._publish)  # 50 Hz target

        # Start reader thread
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._stop = True
        try:
            if self._ser:
                self._ser.write(sopas_command(b'sEN LMDscandata 0'))
                self._ser.flush()
                self._ser.close()
        except Exception:
            pass
        return super().destroy_node()

    def _connect(self):
        port = find_serial_port()
        if port:
            self._ser = serial.Serial(port=port, baudrate=115200, timeout=1.0,
                                      parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                      bytesize=serial.EIGHTBITS, write_timeout=1.0)
            time.sleep(0.2)
            self._ser.reset_input_buffer(); self._ser.reset_output_buffer()
            self.get_logger().info(f"USB serial connected at {port}")
            # SOPAS init
            self._send_and_wait_ack_serial(b'sMN SetAccessMode 03 F4724744', b'sAN SetAccessMode')
            self._send_and_wait_ack_serial(b'sMN LMCstartmeas', b'sAN LMCstartmeas')
            self._send_and_wait_ack_serial(b'sMN Run', b'sAN Run')
            res_code = {'0.33': '1', '0.5': '2', '1.0': '3'}.get(self.angular_resolution, '3')
            cfg_cmd = f'sMN mLMPsetscancfg +{res_code} +1 -450000 +2250000'.encode()
            self._send_and_wait_ack_serial(cfg_cmd, b'sAN mLMPsetscancfg')
            self._send_and_wait_ack_serial(b'sEN LMDscandata 1', b'sEA LMDscandata')
            self._dg_serial = datagrams_from_serial(self._ser)
            return
        # Fallback PyUSB bulk
        if not HAVE_USB:
            raise RuntimeError('No serial port and no PyUSB available. Install pyusb or connect CDC-ACM device.')
        dev = usb.core.find(idVendor=0x19A2, idProduct=0x5001)
        if dev is None:
            raise RuntimeError('SICK TIM (19a2:5001) not found over USB.')
        try:
            for i in range(dev.get_active_configuration().bNumInterfaces):
                if dev.is_kernel_driver_active(i):
                    dev.detach_kernel_driver(i)
        except Exception:
            pass
        dev.set_configuration(); cfg = dev.get_active_configuration()
        ep_in = ep_out = None
        for intf in cfg:
            for ep in intf:
                direction = usb.util.endpoint_direction(ep.bEndpointAddress)
                if direction == usb.util.ENDPOINT_IN and ep_in is None:
                    ep_in = ep
                if direction == usb.util.ENDPOINT_OUT and ep_out is None:
                    ep_out = ep
            if ep_in is not None and ep_out is not None:
                break
        if ep_in is None or ep_out is None:
            raise RuntimeError('USB endpoints not found for TIM device')
        self._usb = {'dev': dev, 'ep_in': ep_in, 'ep_out': ep_out}
        self.get_logger().info(f"USB bulk connected (IN=0x{ep_in.bEndpointAddress:02x}, OUT=0x{ep_out.bEndpointAddress:02x})")
        # Init over USB bulk
        self._usb_write(b'sMN SetAccessMode 03 F4724744'); self._usb_expect(b'sAN SetAccessMode')
        self._usb_write(b'sMN LMCstartmeas'); self._usb_expect(b'sAN LMCstartmeas')
        self._usb_write(b'sMN Run'); self._usb_expect(b'sAN Run')
        res_code = {'0.33': '1', '0.5': '2', '1.0': '3'}.get(self.angular_resolution, '3')
        self._usb_write(f'sMN mLMPsetscancfg +{res_code} +1 -450000 +2250000'.encode())
        self._usb_expect(b'sAN mLMPsetscancfg')  # continue if empty
        self._usb_write(b'sEN LMDscandata 1'); self._usb_expect(b'sEA LMDscandata')

    def _send_and_wait_ack_serial(self, cmd_ascii: bytes, expect_prefix: bytes | None = None, tries: int = 10):
        self._ser.write(sopas_command(cmd_ascii)); self._ser.flush()
        gen = datagrams_from_serial(self._ser)
        for _ in range(tries):
            try:
                dg = next(gen)
            except StopIteration:
                break
            if expect_prefix is None or dg.startswith(expect_prefix):
                return dg
        return b''

    def _usb_write(self, payload_ascii: bytes):
        frame = sopas_command(payload_ascii)
        self._usb['ep_out'].write(frame)
        time.sleep(0.01)

    def _usb_expect(self, prefix: bytes | None, timeout_s: float = 2.0):
        start = time.time()
        ep_in = self._usb['ep_in']
        while time.time() - start < timeout_s:
            try:
                chunk = bytes(ep_in.read(512, timeout=500))
            except Exception:
                time.sleep(0.01)
                continue
            if chunk:
                self._buf += chunk
                while STX in self._buf and ETX in self._buf:
                    s = self._buf.find(STX); e = self._buf.find(ETX, s)
                    if e > s:
                        dg = self._buf[s+1:e]; self._buf = self._buf[e+1:]
                        if prefix is None or dg.startswith(prefix):
                            return dg
                    else:
                        break
        return b''

    def _read_loop(self):
        if self._ser is not None:
            while not self._stop:
                try:
                    dg = next(self._dg_serial)
                    dec = decode_datagram(dg)
                    if dec is not None:
                        self._scan_data = dec  # Store full info including AngularStepWidth
                except Exception:
                    time.sleep(0.01)
        else:
            ep_in = self._usb['ep_in']
            while not self._stop:
                try:
                    chunk = bytes(ep_in.read(8192, timeout=1000))
                    if not chunk:
                        continue
                    self._buf += chunk
                    while STX in self._buf and ETX in self._buf:
                        s = self._buf.find(STX); e = self._buf.find(ETX, s)
                        if e > s:
                            dg = self._buf[s+1:e]; self._buf = self._buf[e+1:]
                            dec = decode_datagram(dg)
                            if dec is not None:
                                self._scan_data = dec  # Store full info including AngularStepWidth
                        else:
                            break
                except Exception:
                    time.sleep(0.01)

    def _publish(self):
        if self._scan_data is None:
            self.get_logger().warn('No scan data available yet', throttle_duration_sec=5.0)
            return
        
        ranges_full = np.array(self._scan_data['Data'], dtype=float)
        
        # Downsample to exactly 270 points
        target_points = 270
        if len(ranges_full) > target_points:
            # Downsample using linear interpolation
            indices = np.linspace(0, len(ranges_full) - 1, target_points)
            ranges = np.interp(indices, np.arange(len(ranges_full)), ranges_full)
        else:
            ranges = ranges_full
        
        num_points = len(ranges)
        
        # Simple approach matching L1_lidar_GUI.py:
        # TiM561 always scans -135° to +135° (270°), distribute points evenly
        angle_min_rad = -135.0 * math.pi / 180.0
        angle_max_rad = 135.0 * math.pi / 180.0
        angular_increment_rad = (angle_max_rad - angle_min_rad) / (num_points - 1) if num_points > 1 else 0.0
        
        subs = self.publisher.get_subscription_count()
        self.get_logger().info(
            f'Publishing scan: {num_points} points, 270.0° span, '
            f'{math.degrees(angular_increment_rad):.3f}° increment | subs={subs}',
            throttle_duration_sec=2.0
        )
        
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(angle_min_rad)
        msg.angle_max = float(angle_max_rad)
        msg.angle_increment = float(angular_increment_rad)
        msg.scan_time = 0.02  # 50 Hz default target
        msg.time_increment = 0.0
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        # Clamp ranges to [range_min, range_max] and fill inf for invalids
        rngs = np.where((ranges >= self.range_min) & (ranges <= self.range_max), ranges, np.inf)
        msg.ranges = [float(x) for x in rngs]
        msg.intensities = []
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UsbTimPublisher()
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
"""