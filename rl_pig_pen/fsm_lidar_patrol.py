#!/usr/bin/env python3
"""LiDAR patrol: left branch, reverse to right end, return to main, repeat."""

import json
import math
import time
import signal
import sys
from collections import deque
from copy import deepcopy
from dataclasses import replace
from threading import Event
from types import SimpleNamespace

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown
from rclpy.impl.implementation_singleton import rclpy_implementation
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from lidar_patrol_controller import TraditionalPatrolController, incoming_corridor
from navigation import SensorFault, lidar_ignore_mask, load_config, scan_features


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
    WATCHDOG_PERIOD = .1
    MAX_RECEIVE_PAUSE = .4
    TIMING_SAMPLE_LIMIT = .5

    def __init__(self):
        super().__init__("pig_pen_lidar_patrol")
        self.cfg = load_config()
        self.controller = TraditionalPatrolController(self.cfg)
        self.last_received = None
        self.reported = None
        self.init_scan_health()
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(LaserScan, "/scan", self.on_scan,
                                            qos_profile_sensor_data)
        self.watchdog = self.create_timer(self.WATCHDOG_PERIOD, self.check_scan,
                                         clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("LiDAR 巡邏啟動：左支線前進、倒退至右端、返回主幹道")

    def init_scan_health(self):
        self.scan_hold_since = None
        self.scan_hold_reason = None
        self.recovery_frames = 0
        self.observed_stamp = None
        self.last_command = (0.0, 0.0)
        self.reported_obstacle = None
        self.reported_relocalization = None
        self.last_receive_interval = None
        self.receive_intervals = deque(maxlen=16)
        self.last_valid_receive = None
        self.last_valid_stamp = None
        self.receive_pause = min(self.MAX_RECEIVE_PAUSE, max(.15, 2 * self.cfg.control_dt))
        # The node may wait for a slower stream. Include the capped watchdog
        # reaction horizon in forward AND reverse braking checks, instead of
        # keeping the old 1/12 s assumption when increasing the receive budget.
        self.controller.cfg = replace(self.controller.cfg, control_dt=max(
            self.cfg.control_dt, self.MAX_RECEIVE_PAUSE + self.WATCHDOG_PERIOD))

    def record_scan_timing(self, now, stamp):
        """Learn only from fresh, valid scan pairs; never from timer ticks.

        Sensor stamps still have their own independent continuity checks.
        A long outage cannot teach the receiver to tolerate arbitrary delay.
        """
        if self.last_valid_receive is not None:
            interval = now - self.last_valid_receive
            sensor_interval = stamp - self.last_valid_stamp
            if (0 < interval < self.TIMING_SAMPLE_LIMIT and
                    0 < sensor_interval <= self.cfg.junction_tracking_max_gap):
                self.receive_intervals.append(interval)
        self.last_valid_receive, self.last_valid_stamp = now, stamp
        if len(self.receive_intervals) >= 3:
            self.receive_pause = min(self.MAX_RECEIVE_PAUSE, max(
                .15, 2 * self.cfg.control_dt,
                1.5 * float(np.quantile(self.receive_intervals, .9)) + .02))

    def scan_health_failure(self, reason, now):
        self.controller.diagnostics.update(
            scan_receive_age_s=None if self.last_received is None else now - self.last_received,
            scan_receive_interval_s=self.last_receive_interval,
            scan_pause_threshold_s=self.receive_pause,
            scan_interval_median_s=(float(np.median(self.receive_intervals))
                                    if self.receive_intervals else None),
            braking_reaction_horizon_s=self.controller.cfg.control_dt,
            recovery_elapsed_s=None if self.scan_hold_since is None else now - self.scan_hold_since,
            recovery_frames=self.recovery_frames,
            recovery_required_frames=self.cfg.confirm_frames,
            recovery_timeout_s=self.cfg.sensor_timeout,
            observed_scan_stamp=self.observed_stamp,
            validated_scan_stamp=self.controller.last_stamp)
        return self.controller.fail(reason)

    def scan_hold_failure_reason(self):
        return ("scan_receive_timeout" if self.scan_hold_reason == "scan_receive_gap"
                else f"scan_recovery_timeout: {self.scan_hold_reason}")

    def pause_scan(self, reason, now, *, invalidates_progress=True, started_at=None):
        hold = self.controller.obstacle
        if hold is not None:
            if hold["phase"] == "RETREAT":
                hold["attempted"] = True
            hold.update(phase="WAIT", reason="sensor_pause", clear_frames=0, ready_frames=0,
                        end_frames=0)
        if self.scan_hold_since is None:
            # For a receive gap, count from the instant the adaptive stop
            # threshold was crossed, even if the watchdog itself ran late.
            self.scan_hold_since = now if started_at is None else min(now, started_at)
            self.scan_hold_reason = reason
            self.recovery_frames = 0
            self.controller.confirm = 0
            self.get_logger().warning(f"雷達安全暫停：{reason}；等待連續有效掃描")
        # A timer observing the same receive gap is not another bad scan.
        # Actual invalid/duplicate scans still break consecutive confirmation.
        if invalidates_progress:
            self.recovery_frames = 0
            self.controller.confirm = 0
            self.last_valid_receive = self.last_valid_stamp = None
        if now - self.scan_hold_since >= self.cfg.sensor_timeout:
            return self.scan_health_failure(self.scan_hold_failure_reason(), now)
        return (0.0, 0.0)

    def measured_hazard(self, scan):
        """Use genuine returns for STOP decisions even if other rays are bad.

        Unconfigured unknown rays are excluded only from this positive-obstacle
        check; they NEVER authorize motion. Explicit robot self-mask rays are
        ignored consistently here and by the separate scan feature pipeline.
        """
        raw = np.asarray(scan.ranges, dtype=float)
        if (raw.ndim != 1 or not math.isfinite(scan.angle_min) or
                not math.isfinite(scan.angle_increment) or scan.angle_increment == 0):
            return None
        angles = scan.angle_min + np.arange(raw.size) * scan.angle_increment + self.cfg.lidar_yaw
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        ignored = lidar_ignore_mask(angles, self.cfg)
        valid = np.isfinite(raw) & (raw >= scan.range_min) & (raw <= scan.range_max)
        ranges = np.where(valid, raw, 1e6)
        ranges[ignored] = 1e6
        features = SimpleNamespace(
            ranges=ranges, angles=angles)
        details = self.controller._clearance_details(self.last_command, features)
        if details["clearance_hits"]:
            self.controller.diagnostics.update(details)
            return self.controller.stop_for_obstacle(self.last_command, features)
        return None

    def publish(self, command):
        msg = Twist()
        msg.linear.x, msg.angular.z = map(float, command)
        self.pub.publish(msg)
        # Preserve the last moving command during a sensor pause so genuine
        # obstacles inside its stopping envelope still trigger protection.
        if self.scan_hold_since is None:
            self.last_command = tuple(command)
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
        hold = self.controller.obstacle
        obstacle_status = None if hold is None else (hold["phase"], hold["reason"])
        if obstacle_status != self.reported_obstacle:
            if hold is None:
                self.get_logger().info("障礙保護解除：淨空或支線端點已確認，依目前路線狀態繼續")
            elif self.controller.state != "FAILED":
                label = {"STOP": "停車確認", "RETREAT": "低速反向退避", "WAIT": "原地等待"}
                measurement = ("以障礙淨空判斷，無縱向位移量測" if hold["mode"] == "clearance"
                               else f"量測退避：{hold['progress']:.3f} m")
                self.get_logger().warning(
                    f"障礙保護：{label[hold['phase']]}；原因：{hold['reason']}；"
                    f"保留狀態：{self.controller.state}；{measurement}")
                if self.reported_obstacle is None:
                    self.get_logger().warning("障礙診斷：" + json.dumps(hold["details"], ensure_ascii=False))
            self.reported_obstacle = obstacle_status
        recovery = self.controller.relocalization
        recovery_status = None if recovery is None else (
            recovery["reason"], recovery["failed_state"])
        if recovery_status != self.reported_relocalization:
            if recovery is None:
                if self.reported_relocalization is not None:
                    self.get_logger().info(
                        "重新定位完成：已取得連續一致的幾何，保留原路線狀態繼續巡邏")
            else:
                self.get_logger().warning(
                    f"定位異常：{recovery['reason']}；保留狀態：{recovery['failed_state']}；"
                    "已停車並等待重新定位")
                self.get_logger().warning(
                    "定位診斷：" + json.dumps(recovery["diagnostics"], ensure_ascii=False))
            self.reported_relocalization = recovery_status

    def on_scan(self, scan):
        now = time.monotonic()
        self.last_receive_interval = None if self.last_received is None else now - self.last_received
        # Stop at the adaptive threshold, but allow the same bounded recovery
        # window used for other sensor faults. A late packet cannot erase the
        # elapsed outage because the hold starts at the threshold crossing.
        if (self.last_receive_interval is not None and
                self.last_receive_interval > self.receive_pause and
                self.controller.state not in self.controller.TERMINAL):
            held = self.pause_scan(
                "scan_receive_gap", now, invalidates_progress=False,
                started_at=now - self.last_receive_interval + self.receive_pause)
            self.publish(held)
            if self.controller.state in self.controller.TERMINAL:
                return
        self.last_received = now
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        previous_failure = self.controller.failure
        try:
            if self.controller.state in self.controller.TERMINAL:
                self.publish((0.0, 0.0))
                return
            hazard = self.measured_hazard(scan)
            if hazard is not None and self.controller.state == "FAILED":
                command = hazard
            else:
                if not math.isfinite(stamp) or (self.observed_stamp is not None and stamp < self.observed_stamp):
                    command = self.controller.fail("scan_time_reversed" if math.isfinite(stamp)
                                                   else "invalid_scan_stamp")
                elif stamp == self.observed_stamp:
                    command = self.pause_scan("repeated_scan_stamp", now)
                else:
                    self.observed_stamp = stamp
                    raw = np.asarray(scan.ranges, dtype=float)
                    known = ((np.isfinite(raw) & (raw >= scan.range_min) & (raw <= scan.range_max)) |
                             np.isposinf(raw))
                    if not np.all(known):
                        raise SensorFault("Invalid LiDAR return (including isolated rays)")
                    features = scan_features(scan, self.cfg)
                    self.record_scan_timing(now, stamp)
                    if self.scan_hold_since is None:
                        command = self.controller.command(features, stamp)
                    elif now - self.scan_hold_since >= self.cfg.sensor_timeout:
                        command = self.scan_health_failure(self.scan_hold_failure_reason(), now)
                    else:
                        # Trial processing checks geometry continuity, stamp
                        # gaps and clearance, without advancing the real FSM
                        # or its counters while the output is held at zero.
                        trial = deepcopy(self.controller)
                        command = trial.command(features, stamp, allow_retreat=False)
                        if self.controller.obstacle is not None:
                            # The obstacle layer freezes all route progress;
                            # keep its measured pose and suspended state clock.
                            self.controller = trial
                        if trial.state == "FAILED":
                            self.controller = trial
                        else:
                            localized = (trial.tracker is not None and trial.tracker.stamp == stamp)
                            if trial.state in ("MAIN", "FINAL_MAIN", "LEFT_OUTBOUND", "REVERSE_LEFT",
                                               "RIGHT_OUTBOUND", "RETURN_RIGHT") or (
                                    trial.state in trial.ENTRY_STATES
                                    and trial.entry_phase == "WALL_CONFIRM"):
                                localized = incoming_corridor(features, self.cfg) is not None
                            self.recovery_frames = self.recovery_frames + 1 if localized else 0
                            if self.recovery_frames >= self.cfg.confirm_frames:
                                self.controller = trial
                                self.scan_hold_since = self.scan_hold_reason = None
                                self.recovery_frames = 0
                                self.get_logger().info("雷達有效性、幾何與安全距離確認完成，恢復巡邏")
                            else:
                                if localized:
                                    # Commit only validated perception while
                                    # stopped, NOT hypothetical FSM progress.
                                    # Subsequent scans must satisfy the same
                                    # per-scan time/pose continuity checks.
                                    self.controller.last_stamp = trial.last_stamp
                                    if trial.state == self.controller.state:
                                        self.controller.tracker = trial.tracker
                                        # The wall-reference latch is perception
                                        # state; route confirmation remains frozen.
                                        self.controller.entry_phase = trial.entry_phase
                                command = (0.0, 0.0)
        except (SensorFault, ValueError, FloatingPointError) as exc:
            command = self.pause_scan(f"invalid_scan: {exc}", now)
        if previous_failure is None and self.controller.failure == "obstacle_clearance":
            for hit in self.controller.failure_diagnostics.get("clearance_hits", []):
                raw = float(scan.ranges[hit["ray_index"]])
                hit["raw_range_m"] = raw if math.isfinite(raw) else str(raw)
                hit["raw_valid"] = math.isfinite(raw) and scan.range_min <= raw <= scan.range_max
        if self.controller.obstacle is not None:
            # Witnesses belong to the triggering scan, not later waiting scans.
            for hit in self.controller.obstacle["details"].get("clearance_hits", []):
                if "raw_valid" not in hit:
                    raw = float(scan.ranges[hit["ray_index"]])
                    hit["raw_range_m"] = raw if math.isfinite(raw) else str(raw)
                    hit["raw_valid"] = math.isfinite(raw) and scan.range_min <= raw <= scan.range_max
        self.publish(command)

    def check_scan(self):
        now = time.monotonic()
        if self.last_received is None:
            self.publish((0.0, 0.0))
        elif (self.scan_hold_since is not None and
              now - self.scan_hold_since >= self.cfg.sensor_timeout):
            self.publish(self.scan_health_failure(self.scan_hold_failure_reason(), now))
        elif now - self.last_received > self.receive_pause:
            self.publish(self.pause_scan(
                "scan_receive_gap", now, invalidates_progress=False,
                started_at=self.last_received + self.receive_pause))


def main(args=None):
    # Do not let rclpy's SIGINT handler invalidate the context before the
    # final stop. The handlers only request exit; ROS is closed afterwards.
    stopping = Event()
    previous_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        for sig in previous_handlers:
            signal.signal(sig, lambda *_: stopping.set())
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = LidarPatrolNode()
        while rclpy.ok(context=node.context) and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    if node.context.ok():
                        node.publish((0.0, 0.0))
                    else:
                        print("ROS context 已關閉，無法再送停止指令；需由接收端逾時保護。", file=sys.stderr)
                except rclpy_implementation.RCLError:
                    if node.context.ok():
                        raise
                    print("ROS context 在退出時關閉，停止指令未能送出。", file=sys.stderr)
                finally:
                    node.destroy_node()
        finally:
            try:
                try_shutdown()
            finally:
                for sig, handler in previous_handlers.items():
                    signal.signal(sig, handler)


if __name__ == "__main__":
    main()
