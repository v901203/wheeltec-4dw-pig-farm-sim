#!/usr/bin/env python3
"""簡單的 /cmd_vel 發布腳本。

用法範例：
  python3 scripts/control_cmdvel.py --linear 0.2 --duration 3
  python3 scripts/control_cmdvel.py --angular 0.5 --duration 2

此腳本會在指定時間內以固定頻率發送同一個 Twist。
"""
import time
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self, topic: str, qos: int = 10):
        super().__init__('cmdvel_pub')
        self.pub = self.create_publisher(Twist, topic, qos)

    def publish_for(self, linear_x: float, angular_z: float, duration: float, rate: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        end = time.time() + float(duration)
        period = 1.0 / float(rate) if rate > 0 else 0.1

        while time.time() < end:
            self.pub.publish(msg)
            time.sleep(period)

        # publish zero to stop
        stop = Twist()
        self.pub.publish(stop)


def parse_args():
    p = argparse.ArgumentParser(description='Publish a Twist to /cmd_vel for a duration')
    p.add_argument('--topic', '-t', default='/cmd_vel', help='cmd_vel topic name')
    p.add_argument('--linear', '-x', type=float, default=0.0, help='linear.x (m/s)')
    p.add_argument('--angular', '-z', type=float, default=0.0, help='angular.z (rad/s)')
    p.add_argument('--duration', '-d', type=float, default=1.0, help='duration in seconds')
    p.add_argument('--rate', '-r', type=float, default=10.0, help='publish rate in Hz')
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CmdVelPublisher(args.topic)
    try:
        node.publish_for(args.linear, args.angular, args.duration, args.rate)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
