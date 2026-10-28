#!/usr/bin/env python3
"""Optional AUTO/MANUAL/STOP entry. The legacy patrol entry is unchanged."""

import json
import math
import time

import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from fsm_lidar_patrol import LidarPatrolNode, main
from lidar_patrol_controller import TraditionalPatrolController, incoming_corridor
from navigation import SensorFault, scan_features, safe_command, lidar_ignore_mask


class ModeControlNode(LidarPatrolNode):
    """One node owns the final velocity output; never run beside legacy patrol.

    Mode changes are serialized by the default single-threaded executor. Manual
    takeover invalidates the route even if motion cannot be measured. The user
    must explicitly confirm return to the start before a new automatic route.
    """

    def __init__(self):
        self.mode = "STOP"
        self.mode_reason = "STARTUP_STOP"
        self.needs_restart = False
        self.manual_command = (0., 0.)
        self.manual_received = None
        self.manual_since = None
        self.features = None
        self.features_received = None
        self.input_stamp = None
        self.commanded = (0., 0.)
        self.control_pub = None
        self.auto_started = False
        super().__init__()
        self.manual_timeout = self._positive_parameter("manual_timeout", .5)
        self.max_linear = self._positive_parameter("max_linear", .2)
        self.max_angular = self._positive_parameter("max_angular", .6)
        self.manual_linear = min(self.max_linear, self._positive_parameter("manual_max_linear", .1))
        self.manual_angular = min(self.max_angular, self._positive_parameter("manual_max_angular", .3))
        self.control_pub = self.create_publisher(String, "/control/status", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.manual_sub = self.create_subscription(Twist, "/manual/cmd_vel", self.on_manual,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.mode_service = self.create_service(SetParametersAtomically,
            "/control/set_mode", self.set_mode)
        self.reset_service = self.create_service(Trigger,
            "/control/reset_at_start", self.reset_at_start)
        self.controller.pause_external()
        self.publish_state()
        self.get_logger().info("三模式控制已啟動：STOP；等待 ROS 2 模式切換請求")

    def _positive_parameter(self, name, default):
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
        return float(value)

    def publish_state(self):
        super().publish_state()
        if self.control_pub is not None:
            v, w = self.commanded
            motion = ("STOPPED" if v == 0 and w == 0 else
                      "ROTATING" if v == 0 else "FORWARD" if v > 0 else "REVERSE")
            self.control_pub.publish(String(data=json.dumps({
                "mode": self.mode, "route_state": self.controller.state,
                "reason": self.mode_reason, "restart_required": self.needs_restart,
                "commanded_motion": motion, "linear_x": v, "angular_z": w,
                "physical_stop_confirmed": False}, ensure_ascii=False)))

    def _emit(self, command):
        self.commanded = tuple(map(float, command))
        msg = Twist()
        msg.linear.x, msg.angular.z = self.commanded
        self.pub.publish(msg)

    def publish(self, command):
        # Called by legacy AUTO callbacks and shutdown; other modes never allow
        # automatic commands through this path.
        if self.mode != "AUTO":
            self._emit((0., 0.))
            return
        command = np.clip(command, [-self.max_linear, -self.max_angular],
                          [self.max_linear, self.max_angular])
        super().publish(command)
        self.commanded = ((0., 0.) if self.controller.external_paused or
            self.scan_hold_since is not None or self.controller.state in self.controller.TERMINAL
            else tuple(map(float, command)))
        if self.controller.state in self.controller.TERMINAL:
            self._stop(self.controller.failure or self.controller.state)
        elif self.controller.external_paused:
            self.mode_reason = self.pig_stop_gate.reason(time.monotonic()) or "RELOCALIZING"
        elif self.scan_hold_since is not None:
            self.mode_reason = self.scan_hold_reason
        elif self.controller.obstacle is not None:
            self.mode_reason = "OBSTACLE_" + self.controller.obstacle["phase"]
        elif self.controller.relocalization is not None:
            self.mode_reason = "RELOCALIZING"
        else:
            self.mode_reason = "AUTO_RUNNING"

    def _stop(self, reason):
        self.mode = "STOP"
        self.mode_reason = reason
        self.manual_command = (0., 0.)
        self.manual_received = self.manual_since = None
        self.controller.pause_external()
        self._emit((0., 0.))
        self.publish_state()

    def _fresh(self):
        return (self.features is not None and self.features_received is not None and
                time.monotonic() - self.features_received <= self.MAX_RECEIVE_PAUSE)

    def set_mode(self, request, response):
        def reply(ok, reason):
            response.result.successful = ok
            response.result.reason = reason
            return response
        if (len(request.parameters) != 1 or request.parameters[0].name != "mode" or
                request.parameters[0].value.type != ParameterType.PARAMETER_STRING):
            return reply(False, "Expected one string parameter named mode")
        target = request.parameters[0].value.string_value
        if target not in ("AUTO", "MANUAL", "STOP"):
            return reply(False, "mode must be AUTO, MANUAL or STOP")
        if target == "STOP":
            self._stop("OPERATOR_STOP")
            return reply(True, "STOP")
        if target == self.mode:
            return reply(True, "Already " + target)
        # Every valid takeover first stops, including a refused AUTO request.
        self._stop("SWITCHING_MODE")
        if not self._fresh():
            self.mode_reason = "SCAN_NOT_READY"
            return reply(False, self.mode_reason)
        if self.controller.state in self.controller.TERMINAL:
            self.mode_reason = "RESET_AT_START_REQUIRED"
            return reply(False, self.mode_reason)
        if safe_command((0., 0.), self.features, self.controller.cfg, allow_reverse=True)[1]:
            self.mode_reason = "FOOTPRINT_BLOCKED"
            return reply(False, self.mode_reason)
        if target == "AUTO":
            if self.needs_restart:
                self.mode_reason = "RESET_AT_START_REQUIRED"
                return reply(False, self.mode_reason)
            if not self.auto_started and incoming_corridor(self.features, self.cfg) is None:
                self.mode_reason = "START_CORRIDOR_NOT_CONFIRMED"
                return reply(False, self.mode_reason)
            self.auto_started = True
            # Clear adapter timing only; keep route state and photo stop latch.
            self.last_received = self.observed_stamp = None
            self.scan_hold_since = self.scan_hold_reason = None
            self.recovery_frames = 0
            self.last_valid_receive = self.last_valid_stamp = None
            self.controller.pause_external()
            self.mode_reason = "RELOCALIZING"
        else:
            self.needs_restart = True
            self.manual_since = time.monotonic()
            self.mode_reason = "WAITING_MANUAL_COMMAND"
        self.mode = target
        self.publish_state()
        return reply(True, target + ": " + self.mode_reason)

    def reset_at_start(self, request, response):
        # This is an operator assertion, NOT global localization from LiDAR.
        if self.mode != "STOP" or not self._fresh():
            response.success = False
            response.message = "STOP and fresh scan required"
            return response
        if (safe_command((0., 0.), self.features, self.controller.cfg, allow_reverse=True)[1]
                or incoming_corridor(self.features, self.cfg) is None):
            response.success = False
            response.message = "Start corridor or footprint not clear"
            return response
        self.controller = TraditionalPatrolController(self.controller.cfg)
        self.controller.pause_external()
        self.last_command = (0., 0.)
        self.auto_started = self.needs_restart = False
        self.reported = self.reported_obstacle = self.reported_relocalization = None
        self.mode_reason = "RESET_AT_START_CONFIRMED"
        self._emit((0., 0.))
        self.publish_state()
        response.success = True
        response.message = "Route reset; remains STOP. Operator confirms physical start pose."
        return response

    def on_manual(self, message):
        if self.mode != "MANUAL":
            return  # Never queue commands sent before takeover.
        now = time.monotonic()
        previous = self.manual_received if self.manual_received is not None else self.manual_since
        if previous is None or now - previous >= self.manual_timeout:
            # A late command cannot erase an outage before the timer sees it.
            self._stop("MANUAL_TIMEOUT")
            return
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(v) for v in values) or any(v != 0 for v in values[1:5]):
            self._stop("INVALID_MANUAL_COMMAND")
            return
        self.manual_received = now
        self.manual_command = (float(np.clip(values[0], -self.manual_linear, self.manual_linear)),
                               float(np.clip(values[5], -self.manual_angular, self.manual_angular)))
        self._manual_output()

    def _manual_output(self):
        if self.mode != "MANUAL":
            return
        now = time.monotonic()
        since = self.manual_received if self.manual_received is not None else self.manual_since
        if since is None or now - since >= self.manual_timeout:
            self._stop("MANUAL_TIMEOUT")
        elif not self._fresh():
            self._stop("SCAN_NOT_READY")
        elif self.pig_stop_gate.reason(now) is not None:
            self._stop("EXTERNAL_STOP")
        else:
            # Reaction horizon includes scan aging plus the 0.1 s watchdog.
            command, blocked = safe_command(self.manual_command, self.features,
                self.controller.cfg, allow_reverse=True)
            if blocked:
                self._stop("MANUAL_OBSTACLE")
            else:
                self.mode_reason = "MANUAL_COMMAND" if self.manual_received is not None else "WAITING_MANUAL_COMMAND"
                self._emit(command)

    def on_pig_stop(self, message):
        if self.mode == "AUTO":
            super().on_pig_stop(message)
        else:
            self.pig_stop_gate.receive(message.data, time.monotonic())
            if self.mode == "MANUAL" and self.pig_stop_gate.reason(time.monotonic()) is not None:
                self._stop("EXTERNAL_STOP")

    def on_scan(self, scan):
        now = time.monotonic()
        if (self.mode == "MANUAL" and self.features_received is not None and
                now - self.features_received > self.MAX_RECEIVE_PAUSE):
            self._stop("SCAN_NOT_READY")
        try:
            stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            if not math.isfinite(stamp) or (self.input_stamp is not None and stamp <= self.input_stamp):
                raise SensorFault("Non-increasing scan stamp")
            raw = np.asarray(scan.ranges, dtype=float)
            angles = scan.angle_min + np.arange(raw.size) * scan.angle_increment + self.cfg.lidar_yaw
            angles = np.arctan2(np.sin(angles), np.cos(angles))
            known = ((np.isfinite(raw) & (raw >= scan.range_min) & (raw <= scan.range_max)) |
                     np.isposinf(raw) | lidar_ignore_mask(angles, self.cfg))
            if not np.all(known):
                raise SensorFault("Invalid LiDAR return")
            features = scan_features(scan, self.cfg)
            self.input_stamp = stamp
            self.features, self.features_received = features, now
        except (SensorFault, ValueError, FloatingPointError):
            self.features = None
            if self.mode == "AUTO":
                super().on_scan(scan)
            elif self.mode == "MANUAL":
                self._stop("INVALID_SCAN")
            return
        if self.mode == "AUTO":
            super().on_scan(scan)
        elif self.mode == "MANUAL":
            self._manual_output()
        else:
            # Update stationary geometry and freeze route timers without
            # advancing route counters. Discarded manual routes are not tracked.
            if self.auto_started and not self.needs_restart:
                self.controller.hold_external(features, stamp, release=False)
            self._emit((0., 0.))

    def check_scan(self):
        if self.mode == "AUTO":
            super().check_scan()
        elif self.mode == "MANUAL":
            self._manual_output()
        else:
            self._emit((0., 0.))


if __name__ == "__main__":
    main(node_factory=ModeControlNode)
