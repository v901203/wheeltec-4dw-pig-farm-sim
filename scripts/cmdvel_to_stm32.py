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

import rclpy
from rclpy.node import Node
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

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        rate_hz = self.get_parameter('publish_rate').get_parameter_value().integer_value

        if serial is None:
            self.get_logger().error('pyserial not available. pip install pyserial')
            raise RuntimeError('pyserial not available')

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.5)
            self.get_logger().info(f'Opened serial {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {port}: {e}')
            raise

        self.last_twist = Twist()
        self.lock = threading.Lock()

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb_cmdvel, 10)
        self.timer = self.create_timer(1.0 / max(1, rate_hz), self.timer_cb)

    def cb_cmdvel(self, msg: Twist):
        with self.lock:
            self.last_twist = msg

    def timer_cb(self):
        with self.lock:
            t = self.last_twist

        # Convert: linear x,y in m/s -> mm/s (int)
        x_mm = int(round(t.linear.x * 1000.0))
        y_mm = int(round(t.linear.y * 1000.0))
        z_val = int(round(t.angular.z * 1000.0))

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
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
