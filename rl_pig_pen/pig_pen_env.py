"""38-D LiDAR environment: corridor PPO training or hybrid patrol evaluation.

Corridor training uses only PPO actions. PatrolTrainingEnv groups deterministic
FSM maneuvers between PPO decisions for full-route training.
"""

import math
import subprocess
import threading
import time

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from navigation import (CONFIG_FILE, MAX_ANG, MAX_LIN, N_LIDAR, SensorFault,
                        collision_detected, load_config, local_reward,
                        safe_command, scan_features, wrap_angle)
from patrol_controller import PatrolController
from policy_config import policy_spaces


def _stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def _pose(odom):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
    result = (p.x, p.y, yaw)
    if not all(math.isfinite(v) for v in (*result, odom.twist.twist.linear.x)):
        raise SensorFault("Non-finite odometry")
    return result


class _RosNode(Node):
    def __init__(self):
        super().__init__("pig_pen_rl_env_node")
        self.condition = threading.Condition()
        self.scan = self.odom = None
        self.scan_received = self.odom_received = 0.0
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom_cb, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def _scan_cb(self, msg):
        with self.condition:
            self.scan, self.scan_received = msg, time.monotonic()
            self.condition.notify_all()

    def _odom_cb(self, msg):
        with self.condition:
            self.odom, self.odom_received = msg, time.monotonic()
            self.condition.notify_all()

    def get_data(self):
        with self.condition:
            return self.scan, self.odom, self.scan_received, self.odom_received

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self.publisher.publish(msg)


def _teleport(cfg, pose):
    x, y, yaw = pose
    request = (f'name: "{cfg.model_name}" position: {{x: {x:.6f}, y: {y:.6f}, z: 0.05}} '
               f'orientation: {{z: {math.sin(yaw / 2):.8f}, w: {math.cos(yaw / 2):.8f}}}')
    command = ["ign", "service", "-s", f"/world/{cfg.world_name}/set_pose",
               "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
               "--timeout", "5000", "--req", request]
    last_error = ""
    for attempt in range(3):
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        if result.returncode == 0 and "data: true" in result.stdout:
            return
        last_error = f"{result.stderr.strip()} {result.stdout.strip()}".strip()
        if attempt < 2:
            time.sleep(0.2)
    raise RuntimeError(f"Gazebo reset failed after 3 attempts: {last_error}")


class PigPenEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, *, mode="train", config_file=CONFIG_FILE):
        super().__init__()
        if mode not in ("train", "patrol"):
            raise ValueError("mode must be 'train' or 'patrol'")
        self.mode, self.cfg = mode, load_config(config_file)
        self.observation_space, self.action_space = policy_spaces()
        self._owns_ros = not rclpy.ok()
        if self._owns_ros:
            # Let Python handle Ctrl-C so PPO can save before ROS is closed.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._ros = _RosNode()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self.controller = PatrolController(self.cfg)
        self._closed = False
        self._episode_done = True
        self._last_obs = np.zeros(N_LIDAR + 2, dtype=np.float32)

    def _wait_sensor(self, min_stamp=None, timeout=None, min_received=None):
        timeout = self.cfg.sensor_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            scan, odom, scan_received, odom_received = self._ros.get_data()
            now = time.monotonic()
            if (scan is not None and odom is not None and
                    now - min(scan_received, odom_received) < self.cfg.sensor_timeout and
                    (min_received is None or min(scan_received, odom_received) > min_received)):
                stamp = _stamp(scan)
                if ((min_stamp is None or stamp >= min_stamp - 0.001) and
                        abs(_stamp(odom) - stamp) <= 0.10):
                    _pose(odom)
                    return scan, odom
            with self._ros.condition:
                self._ros.condition.wait(timeout=0.02)
        self._ros.publish_cmd(0.0, 0.0)
        raise SensorFault("Timed out waiting for fresh synchronized /scan and /odom")

    def _training_spawn(self):
        """Reset locations only; never navigation targets or observation inputs.

        These corridor interiors match pig_pen_16units.world. A different world
        requires new reset sampling and topology/threshold configuration.
        """
        if self.cfg.world_name != "pig_pen_16units_world":
            raise ValueError("Training reset sampler currently supports pig_pen_16units_world only")
        if self.np_random.random() < 0.5:
            x, y = 0.0, float(self.np_random.choice([-10.0, -8.5, -4.0, -2.5, 2.5, 4.0, 8.5, 10.0]))
            yaw = float(self.np_random.choice([-math.pi / 2, math.pi / 2]))
        else:
            x = float(self.np_random.choice([-2.5, 2.5]))
            # Cross aisles lie BETWEEN pen rows; main-aisle y samples would
            # place this lateral x inside a pen, even if LiDAR sees two walls.
            y = float(self.np_random.choice([-6.18, 0.0, 6.18]))
            yaw = float(self.np_random.choice([0.0, math.pi]))
        lateral = self.np_random.uniform(-0.08, 0.08)
        return (x - lateral * math.sin(yaw), y + lateral * math.cos(yaw),
                yaw + self.np_random.uniform(-0.12, 0.12))

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._episode_done = True
        self._ros.publish_cmd(0.0, 0.0)
        self._wait_sensor(timeout=8.0)
        attempts = self.cfg.reset_attempts if self.mode == "train" else 1
        for attempt in range(1, attempts + 1):
            spawn = self._training_spawn() if self.mode == "train" else self.cfg.start
            self._ros.publish_cmd(0.0, 0.0)
            _teleport(self.cfg, spawn)
            # The service accepts the pose asynchronously. Start settling from
            # messages received AFTER its reply, never from the pre-reset cache.
            current, _ = self._wait_sensor(min_received=time.monotonic(), timeout=8.0)
            scan, odom = self._wait_sensor(min_stamp=_stamp(current) + 0.5, timeout=8.0)
            confirmed = 0
            for frame in range(self.cfg.reset_validation_frames):
                self._ros.publish_cmd(0.0, 0.0)
                try:
                    features = scan_features(scan, self.cfg)
                    collision = collision_detected(features, self.cfg)
                    corridor = self.mode != "train" or features.walls_present(self.cfg)
                    valid = not collision and corridor
                    diagnostic = (f"left={features.left_edge_m:.3f}m, "
                                  f"right={features.right_edge_m:.3f}m, "
                                  f"wall_limit={self.cfg.wall_distance:.3f}m, collision={collision}")
                except SensorFault as exc:
                    valid = False
                    diagnostic = str(exc)
                confirmed = confirmed + 1 if valid else 0
                if confirmed >= self.cfg.confirm_frames:
                    break
                if frame + 1 < self.cfg.reset_validation_frames:
                    scan, odom = self._wait_sensor(min_stamp=_stamp(scan) + self.cfg.control_dt)
            if confirmed >= self.cfg.confirm_frames:
                break
            self._ros.get_logger().warning(
                f"[重生重試] 起點檢查失敗 {attempt}/{attempts}，座標 {spawn}: {diagnostic}")
        else:
            # Do not accept a collision, open-space reset or broken sensor as a
            # training corridor. Exhausted retries remain a diagnostic failure.
            self._ros.publish_cmd(0.0, 0.0)
            raise RuntimeError(
                f"Unable to validate {self.mode} reset after {attempts} attempt(s); "
                f"last spawn={spawn}: {diagnostic}. Check world geometry and LiDAR.")
        pose, stamp = _pose(odom), _stamp(scan)
        self.controller.reset(pose, stamp)
        self._steps = 0
        self._last_stamp = stamp
        self._motion_pose, self._motion_stamp = pose, stamp
        self._last_obs = features.observation
        self._latest_snapshot = (scan, odom)
        self._episode_done = False
        return self._last_obs.copy(), {"mode": self.mode, "reset_attempts": attempt,
                                       "reset_validation_frames": frame + 1, "spawn": tuple(spawn)}

    def step(self, action):
        if self._episode_done:
            raise RuntimeError("Call reset() before step() or after episode completion")
        try:
            return self._step(action)
        except SensorFault as exc:
            self._ros.publish_cmd(0.0, 0.0)
            self._episode_done = True
            info = self.controller.info() if self.mode == "patrol" else {}
            info.update(success=False, failure_reason="sensor_fault", sensor_error=str(exc))
            return self._last_obs.copy(), 0.0, False, True, info
        except BaseException:
            self._ros.publish_cmd(0.0, 0.0)
            raise

    def _step(self, action, *, snapshot=None, arbitrate=True):
        scan, odom = self._wait_sensor() if snapshot is None else snapshot
        self._latest_snapshot = (scan, odom)
        features = scan_features(scan, self.cfg)
        info = {"success": False, "mode": self.mode}
        if collision_detected(features, self.cfg):
            self._ros.publish_cmd(0.0, 0.0)
            self._episode_done = True
            if self.mode == "patrol":
                info.update(self.controller.info())
            info.update(collision=True, success=False, failure_reason="collision")
            return features.observation, -100.0, True, False, info

        command = action
        owner = "rl"
        if self.mode == "patrol":
            override = self.controller.command(features, _pose(odom), _stamp(scan)) if arbitrate else None
            if override is not None:
                command, owner = override, "fsm"
            info.update(self.controller.info())
            if self.controller.state in ("DONE", "FAILED"):
                self._ros.publish_cmd(0.0, 0.0)
                self._episode_done = True
                return features.observation, 0.0, True, False, info

        command, blocked = safe_command(command, features, self.cfg, odom.twist.twist.linear.x)
        self._ros.publish_cmd(*command)
        # One fresh LiDAR frame per nominal 1/12 simulated second. No RTF guess.
        scan, odom = self._wait_sensor(min_stamp=_stamp(scan) + self.cfg.control_dt)
        self._latest_snapshot = (scan, odom)
        # Stop while PPO computes/updates; it may take many simulated seconds.
        self._ros.publish_cmd(0.0, 0.0)
        features = scan_features(scan, self.cfg)
        pose, stamp = _pose(odom), _stamp(scan)
        self._steps += 1
        collision = collision_detected(features, self.cfg)
        corridor = max(features.left_edge_m, features.right_edge_m) <= self.cfg.open_distance
        reward = local_reward(odom.twist.twist.linear.x, features.balance_m, collision, corridor)
        terminated, truncated = collision, False
        info.update(controller=owner, safety_stop=blocked, collision=collision,
                    balance_m=features.balance_m, front_clearance_m=features.front_m,
                    actual_dt=stamp - self._last_stamp)
        if collision:
            info.update(success=False, failure_reason="collision")
        if self.mode == "train":
            # End the PPO segment BEFORE an FSM-controlled maneuver is needed.
            if not corridor and not collision:
                truncated = True
                info["segment_complete"] = True
            moved = math.hypot(pose[0] - self._motion_pose[0], pose[1] - self._motion_pose[1])
            turned = abs(wrap_angle(pose[2] - self._motion_pose[2]))
            if moved > 0.05 or turned > 0.08:
                self._motion_pose, self._motion_stamp = pose, stamp
            if stamp - self._motion_stamp > self.cfg.stuck_seconds and not collision:
                truncated = True
                info["failure_reason"] = "stuck"
                info.update(motion_distance_m=moved, motion_angle_rad=turned,
                            no_motion_sim_seconds=stamp-self._motion_stamp)
        limit = self.cfg.train_max_steps if self.mode == "train" else self.cfg.patrol_max_steps
        if self._steps >= limit and not terminated:
            truncated = True
            info["failure_reason"] = "time_limit"
        self._last_stamp, self._last_obs = stamp, features.observation
        self._episode_done = terminated or truncated
        hook = getattr(self, "progress_hook", None)
        if hook is not None:
            hook()
        return features.observation.copy(), reward, terminated, truncated, info

    def close(self):
        if self._closed:
            return
        self._ros.publish_cmd(0.0, 0.0)
        self._executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self._ros.destroy_node()
        if self._owns_ros and rclpy.ok():
            rclpy.shutdown()
        self._closed = True
