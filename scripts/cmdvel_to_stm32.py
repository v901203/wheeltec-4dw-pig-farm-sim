#!/usr/bin/env python3
"""ROS2 node: subscribe /cmd_vel and send STM32 serial packet per WHEELTEC manual.

Packet format (11 bytes):
  [0] 0x7B (header)
  [1] reserved (0x00)
  [2] reserved (0x00)
  [3] X MSB
  [4] X LSB
  [5] Y MSB
  [6] Y LSB
  [7] Z MSB
  [8] Z LSB
  [9] checksum = XOR of bytes 0..8
  [10] 0x7D (tail)

X,Y: mm/s (signed short, big-endian)
Z: angular z * 1000 (rad/s * 1000, signed short, big-endian)

Usage:
  - install dependencies: pip install rclpy pyserial
  - run: ros2 run or python3 scripts/cmdvel_to_stm32.py

"""
import sys
import struct
import threading
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist

try:
    import serial
except Exception:
    serial = None


def clamp_short(v: int) -> int:
    if v < -32768:
        return -32768
    if v > 32767:
        return 32767
    return v


class CmdVelToSTM32(Node):
    def __init__(self):
        super().__init__('cmdvel_to_stm32')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 10)
        self.declare_parameter('command_timeout', 0.5)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        rate_hz = self.get_parameter('publish_rate').get_parameter_value().integer_value
        self.command_timeout = self.get_parameter('command_timeout').value
        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0:
            raise ValueError('command_timeout must be finite and positive')
        if rate_hz < 1:
            raise ValueError('publish_rate must be positive')

        if serial is None:
            self.get_logger().error('pyserial not available. pip install pyserial')
            raise RuntimeError('pyserial not available')

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.5, write_timeout=0.1)
            self.get_logger().info(f'Opened serial {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {port}: {e}')
            raise

        self.last_twist = Twist()
        self.last_command_time = None
        self.timed_out = False
        self.lock = threading.Lock()

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb_cmdvel, 1)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_cb,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.send_twist(Twist())

    def cb_cmdvel(self, msg: Twist):
        with self.lock:
            if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
                self.last_twist = Twist()
                self.last_command_time = None
                self.get_logger().error('Invalid cmd_vel; stopping until a fresh finite command arrives')
            else:
                self.last_twist = msg
                self.last_command_time = time.monotonic()
                self.timed_out = False

    def timer_cb(self):
        with self.lock:
            expired = (self.last_command_time is None or
                       time.monotonic() - self.last_command_time >= self.command_timeout)
            t = Twist() if expired else self.last_twist
            if expired and self.last_command_time is not None and not self.timed_out:
                self.get_logger().warning('cmd_vel timeout; sending zero velocity')
                self.timed_out = True
        self.send_twist(t)

    def send_twist(self, t):
        # Convert: linear x,y in m/s -> mm/s (int)
        # Bound before multiplication, so even a huge finite input cannot
        # overflow and prevent the timer from sending subsequent stop packets.
        def scaled(value):
            return int(round(min(32.767, max(-32.768, value)) * 1000.0))

        x_mm = scaled(t.linear.x)
        y_mm = scaled(t.linear.y)
        z_val = scaled(t.angular.z)

        x_mm = clamp_short(x_mm)
        y_mm = clamp_short(y_mm)
        z_val = clamp_short(z_val)

        # Pack big-endian MSB/LSB
        pkt = bytearray()
        pkt.append(0x7B)
        pkt.append(0x00)
        pkt.append(0x00)
        pkt.extend(struct.pack('>h', x_mm))
        pkt.extend(struct.pack('>h', y_mm))
        pkt.extend(struct.pack('>h', z_val))

        # checksum: XOR of first 9 bytes
        chk = 0
        for b in pkt[0:9]:
            chk ^= b
        pkt.append(chk & 0xFF)
        pkt.append(0x7D)

        try:
            self.ser.write(pkt)
            # optional flush
        except Exception as e:
            self.get_logger().error(f'Error writing serial: {e}')
            # Do not replay the cached motion if the link later recovers.
            with self.lock:
                self.last_twist = Twist()
                self.last_command_time = None

    def stop_and_close(self):
        """Best effort only: SIGKILL, power loss and a broken link need a
        chassis-side watchdog; this process cannot protect its own death.
        """
        try:
            self.send_twist(Twist())
        finally:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelToSTM32()
    except Exception as e:
        print('Failed to start node:', e)
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_and_close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
