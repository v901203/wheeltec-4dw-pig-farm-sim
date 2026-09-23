#!/usr/bin/env python3
"""LiDAR patrol: left branch, reverse to right end, return to main, repeat."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from lidar_patrol_controller import TraditionalPatrolController
from navigation import SensorFault, load_config, scan_features


STATE_LABELS = {
    "MAIN": "主幹道前進", "CENTER": "路口置中", "TURN_LEFT": "左轉進入支線",
    "FINAL_MAIN": "支線巡邏完成，沿主幹道前進至終點",
    "ENTRY_LEFT": "進入左支線", "LEFT_OUTBOUND": "左支線前進到底",
    "REVERSE_LEFT": "左支線倒退返回", "CROSS_REVERSE": "倒退穿越路口",
    "RIGHT_OUTBOUND": "右支線倒退到底", "RETURN_RIGHT": "前進返回路口",
    "RETURN_CENTER": "返回路口置中", "TURN_MAIN": "右轉回主幹道",
    "EXIT_MAIN": "離開路口", "DONE": "主幹道終點停止", "FAILED": "異常停止",
}


class LidarPatrolNode(Node):
    def __init__(self):
        super().__init__("pig_pen_lidar_patrol")
        self.cfg = load_config()
        self.controller = TraditionalPatrolController(self.cfg)
        self.last_received = None
        self.reported = None
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(LaserScan, "/scan", self.on_scan,
                                            qos_profile_sensor_data)
        self.watchdog = self.create_timer(.1, self.check_scan,
                                         clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("LiDAR 巡邏啟動：左支線前進、倒退至右端、返回主幹道")

    def publish(self, command):
        msg = Twist()
        msg.linear.x, msg.angular.z = map(float, command)
        self.pub.publish(msg)
        status = (self.controller.state, self.controller.failure)
        if status != self.reported:
            self.get_logger().info(
                f"{STATE_LABELS[status[0]]} ({status[0]})，"
                f"已完成路口：{self.controller.junctions_done}，"
                f"已到達支線端點：{self.controller.endpoints_reached}"
                + (f"，原因：{status[1]}" if status[1] else ""))
            if status[0] == "FAILED":
                details = self.controller.failure_diagnostics
                self.get_logger().error("停止診斷：" + json.dumps(details, ensure_ascii=False))
            self.reported = status

    def on_scan(self, scan):
        self.last_received = time.monotonic()
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        previous_failure = self.controller.failure
        try:
            command = self.controller.command(scan_features(scan, self.cfg), stamp)
        except (SensorFault, ValueError, FloatingPointError) as exc:
            command = self.controller.fail(f"invalid_scan: {exc}")
        if previous_failure is None and self.controller.failure == "obstacle_clearance":
            for hit in self.controller.failure_diagnostics.get("clearance_hits", []):
                raw = float(scan.ranges[hit["ray_index"]])
                hit["raw_range_m"] = raw if math.isfinite(raw) else str(raw)
                hit["raw_valid"] = math.isfinite(raw) and scan.range_min <= raw <= scan.range_max
        self.publish(command)

    def check_scan(self):
        if self.last_received is None:
            self.publish((0.0, 0.0))
        elif time.monotonic() - self.last_received > .5:
            self.publish(self.controller.fail("scan_receive_timeout"))


def main(args=None):
    rclpy.init(args=args)
    node = LidarPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish((0.0, 0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
