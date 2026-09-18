#!/usr/bin/env python3
"""Record human Twist commands paired with the latest preceding LiDAR scan.

Read-only: this node never publishes cmd_vel and never resets Gazebo.
Hold keyboard movement keys (or enable joystick autorepeat) while driving.
"""

import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from demonstrations import DEMO_DIR, DemoWriter, demonstration_sample
from navigation import CONFIG_FILE, load_config


class DemonstrationRecorder(Node):
    def __init__(self, writer, cfg, scan_topic="/scan", cmd_topic="/cmd_vel", max_scan_age=0.25):
        super().__init__("corridor_demonstration_recorder")
        self.writer, self.cfg = writer, cfg
        self.max_scan_age = max_scan_age
        self.cmd_topic = cmd_topic
        self.latest_scan = None
        self.used_stamp = None
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, cmd_topic, self._command_cb, 10)
        self.create_timer(15.0, self.report)

    def _scan_cb(self, msg):
        self.latest_scan = (msg, time.monotonic())

    def _command_cb(self, msg):
        # Refuse blended expert/RL streams. Remap --cmd-topic for a command mux.
        if self.count_publishers(self.cmd_topic) != 1:
            self.writer.counts["multiple_or_unknown_command_sources"] += 1
            return
        now = time.monotonic()
        if self.latest_scan is None:
            self.writer.counts["no_scan"] += 1
            return
        scan, received = self.latest_scan
        age = now - received
        if not 0 <= age <= self.max_scan_age:
            self.writer.counts["stale_scan"] += 1
            return
        stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if stamp == self.used_stamp:
            self.writer.counts["duplicate_scan"] += 1
            return
        other_axes = (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
        if any(not math.isfinite(v) or abs(v) > 1e-8 for v in other_axes):
            self.writer.counts["unsupported_motion"] += 1
            return
        sample, reason = demonstration_sample(scan, [msg.linear.x, msg.angular.z], self.cfg)
        if sample is None:
            self.writer.counts[reason] += 1
            return
        observation, action = sample
        self.writer.add(observation, action, stamp[0] + stamp[1]*1e-9, now, age)
        self.used_stamp = stamp

    def report(self):
        self.writer.flush()
        self.get_logger().info(f"saved={self.writer.saved}, counts={dict(self.writer.counts)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEMO_DIR)
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--cmd-topic", default="/cmd_vel", help="geometry_msgs/Twist topic from the human driver")
    parser.add_argument("--max-scan-age", type=float, default=0.25, help="Maximum receipt-time gap in wall seconds")
    parser.add_argument("--chunk-size", type=int, default=500)
    parser.add_argument("--duration", type=float, default=0, help="Wall seconds; 0 records until Ctrl-C")
    args = parser.parse_args()
    if not math.isfinite(args.max_scan_age) or args.max_scan_age <= 0 or args.chunk_size < 1:
        parser.error("--max-scan-age and --chunk-size must be positive")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and nonnegative")
    cfg = load_config(args.config)
    rclpy.init()
    writer = DemoWriter(args.output, cfg, args.chunk_size,
                        dict(scan_topic=args.scan_topic, cmd_topic=args.cmd_topic,
                             pairing="latest scan received before command", max_scan_age=args.max_scan_age))
    node = DemonstrationRecorder(writer, cfg, args.scan_topic, args.cmd_topic, args.max_scan_age)
    print(f"Recording manual corridor demonstrations to {writer.directory}", flush=True)
    print("Stop PPO first. Hold movement keys while driving; Ctrl-C here saves the final chunk.", flush=True)
    deadline = time.monotonic() + args.duration if args.duration else math.inf
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        writer.flush()
        print(f"Saved {writer.saved} samples to {writer.directory}; counts={dict(writer.counts)}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
