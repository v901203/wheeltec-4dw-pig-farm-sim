"""Gymnasium environment; only the isolated Gazebo training world is allowed."""
import math
import os
from pathlib import Path
import sys
import time
from dataclasses import replace

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rl_pig_pen"))
from navigation import load_config, scan_features, safe_command, lidar_ignore_mask
from tasks import TASKS, WORLD_NAME, CENTRE_Y, collision, distance, goal_reached, angle_error


def stamp(s):
    return s.sec+s.nanosec*1e-9


class JunctionEnv(gym.Env):
    metadata = {"render_modes": []}
    # 72 minimum-range sectors + 72 usability flags + task one-hot + measured
    # planar velocities + previous executed action. No world pose in observation.
    OBS_SIZE = 151

    def __init__(self, task=None, max_steps=1200, available_tasks=None):
        if (os.environ.get("ROS_DOMAIN_ID") != "42" or
                os.environ.get("IGN_PARTITION") != "4wd_rl_junction"):
            raise RuntimeError("Set ROS_DOMAIN_ID=42 and IGN_PARTITION=4wd_rl_junction; never use the real robot domain")
        if task is not None and task not in [t.name for t in TASKS]:
            raise ValueError("Unknown task")
        self.fixed_task = task
        self.available_indices = [i for i, t in enumerate(TASKS)
                                  if available_tasks is None or t.name in available_tasks]
        if not self.available_indices or (task and task not in [TASKS[i].name for i in self.available_indices]):
            raise ValueError("Requested task is outside the allowed task set")
        self.max_steps = max_steps
        self.dt = .2
        self.cfg = replace(load_config(), control_dt=self.dt, vision_required=False)
        self.action_space = gym.spaces.Box(-1., 1., shape=(2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-1., 1., shape=(self.OBS_SIZE,), dtype=np.float32)
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("rl_junction_env", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.control_client = self.node.create_client(ControlWorld, f"/world/{WORLD_NAME}/control")
        self.pose_client = self.node.create_client(SetEntityPose, f"/world/{WORLD_NAME}/set_pose")
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", 1)
        self.scan = self.odom = self.pose = None
        self.scan_stamp = self.odom_stamp = self.pose_stamp = -1.
        self.sim_time = 0.
        self.node.create_subscription(Clock, "/clock", self._clock, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.node.create_subscription(PoseStamped, "/rl/ground_truth", self._pose, qos_profile_sensor_data)
        self.active = False
        self.previous = np.zeros(2, dtype=np.float32)

    def _scan(self, msg):
        self.scan, self.scan_stamp = msg, stamp(msg.header.stamp)

    def _clock(self, msg):
        self.sim_time = stamp(msg.clock)

    def _odom(self, msg):
        self.odom, self.odom_stamp = msg, stamp(msg.header.stamp)

    def _pose(self, msg):
        self.pose_z = float(msg.pose.position.z)
        q = msg.pose.orientation
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])
        self.pose_stamp = stamp(msg.header.stamp)

    def _service(self, client, request):
        if not client.wait_for_service(timeout_sec=10.):
            raise RuntimeError(f"Missing Gazebo service bridge: {client.srv_name}")
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=10.)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"Gazebo service timed out: {client.srv_name}; no automatic step retry")
        if future.exception() is not None:
            raise RuntimeError(f"Gazebo service failed: {client.srv_name}") from future.exception()
        if not future.result().success:
            raise RuntimeError(f"Gazebo rejected request: {client.srv_name}")

    def _control(self, steps=0, reset_models=False):
        request = ControlWorld.Request()
        request.world_control.pause = True
        request.world_control.multi_step = steps
        request.world_control.reset.model_only = reset_models
        self._service(self.control_client, request)

    def _publish(self, action):
        msg = Twist()
        msg.linear.x, msg.angular.z = map(float, action)
        self.pub.publish(msg)

    def _before_advance(self):
        """Optional simulation disturbance, inside the same control transition."""

    def _drain(self, seconds=.08):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def _advance(self, seconds):
        before = self.sim_time
        self._control(steps=round(seconds/.001))
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
            times = (self.scan_stamp, self.odom_stamp, self.pose_stamp)
            if self.sim_time > before+seconds+.02:
                raise RuntimeError("World advanced beyond requested steps; do not use GUI play during training")
            if (self.sim_time >= before+seconds-.002 and
                    min(times) >= self.sim_time-.11 and max(times) <= self.sim_time+.02 and
                    max(times)-min(times) <= .12):
                if self.scan is not None and self.odom is not None and self.pose is not None:
                    return
        raise RuntimeError("No synchronized fresh scan/odom/ground_truth after stepping; "
            f"before={before}, clock={self.sim_time}, scan={self.scan_stamp}, "
            f"odom={self.odom_stamp}, pose={self.pose_stamp}; episode aborted")

    def _features(self):
        f = scan_features(self.scan, self.cfg)
        raw = np.asarray(self.scan.ranges)
        known = ((np.isfinite(raw) & (raw >= self.scan.range_min) & (raw <= self.scan.range_max)) |
                 np.isposinf(raw) | lidar_ignore_mask(f.angles, self.cfg))
        if not np.all(known):
            raise RuntimeError("Invalid unmasked scan return; cannot use episode as training data")
        return f

    def _observation(self):
        f = self._features()
        ignored = lidar_ignore_mask(f.angles, self.cfg)
        bins = np.clip(((f.angles+math.pi)/(2*math.pi)*72).astype(int), 0, 71)
        ranges, valid = np.ones(72), np.zeros(72)
        for i in range(72):
            take = (bins == i) & ~ignored
            if take.any():
                ranges[i] = np.clip(np.min(f.ranges[take])/10., 0., 1.)
                valid[i] = 1.
        task_code = np.eye(3)[self.task_index]
        velocity = self.odom.twist.twist
        measured = np.clip([velocity.linear.x/.2, velocity.angular.z/.6], -1, 1)
        return np.concatenate((ranges, valid, task_code, measured, self.previous)).astype(np.float32)

    def _assert_exclusive(self):
        self._drain(1.)
        publishers = self.node.get_publishers_info_by_topic("/cmd_vel")
        if len(publishers) != 1 or publishers[0].node_name != self.node.get_name():
            raise RuntimeError("Training requires exactly one /cmd_vel publisher")

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.active = False
        try:
            self._assert_exclusive()
            self._publish((0., 0.))
            self._drain()
            self._control(reset_models=True)
            self._drain(.3)
            self.task_index = ([t.name for t in TASKS].index(self.fixed_task) if self.fixed_task
                               else int(self.np_random.choice(self.available_indices)))
            self.task = TASKS[self.task_index]
            start = np.array(self.task.start)
            start[:2] += self.np_random.uniform(-.08, .08, 2)
            start[2] += self.np_random.uniform(-.08, .08)
            request = SetEntityPose.Request()
            request.entity.name = "wheeltec_mini"
            request.entity.type = request.entity.MODEL
            request.pose.position.x = float(start[0])
            request.pose.position.y = float(start[1])
            request.pose.position.z = .05
            request.pose.orientation.z = math.sin(start[2]/2)
            request.pose.orientation.w = math.cos(start[2]/2)
            self._service(self.pose_client, request)
            self._drain(.2)
            self.scan = self.odom = self.pose = None
            self.scan_stamp = self.odom_stamp = self.pose_stamp = -1.
            self._publish((0., 0.))
            self._drain()
            self._advance(1.)  # settle after teleport, with zero command
            if np.linalg.norm(self.pose[:2]-start[:2]) > .2 or collision(self.pose):
                raise RuntimeError("Reset pose not validated")
            velocity = self.odom.twist.twist
            if abs(velocity.linear.x) > .02 or abs(velocity.angular.z) > .03:
                raise RuntimeError("Robot not stationary after reset")
            self.previous = np.zeros(2, dtype=np.float32)
            self.steps = self.goal_frames = self.idle_steps = 0
            self.previous_distance = distance(self.pose, self.task)
            self.active = True
            return self._observation(), {"task": self.task.name}
        except Exception:
            self._publish((0., 0.))
            self._drain()
            raise

    def step(self, action):
        if not self.active:
            raise RuntimeError("Call reset before step")
        try:
            action = np.asarray(action, dtype=np.float32)
            if action.shape != (2,) or not np.all(np.isfinite(action)):
                raise ValueError("Expected two finite actions")
            requested = np.clip(action, -1, 1)*[.2, .6]
            if self.task.name == "left_to_right_reverse":
                # Preserve the real route: reverse through the junction, not a U-turn.
                requested[0] = -.1*(float(np.clip(action[0], -1, 1))+1.)
            executed, blocked = safe_command(requested, self._features(), self.cfg,
                measured_speed=abs(self.odom.twist.twist.linear.x), allow_reverse=True)
            self._publish(executed)
            self._drain()
            self._before_advance()
            self._advance(self.dt)
            self.steps += 1
            d = distance(self.pose, self.task)
            progress = float(np.clip(self.previous_distance-d, -.08, .08))
            self.previous_distance = d
            self.idle_steps = self.idle_steps+1 if abs(progress) < .001 else 0
            self.goal_frames = self.goal_frames+1 if goal_reached(self.pose, self.task) else 0
            hit = collision(self.pose, self.cfg.half_length, self.cfg.half_width)
            success = self.goal_frames >= 3 and not hit
            off_route = bool(abs(self.pose[0]) > 3.7 or abs(self.pose[1]-CENTRE_Y) > 2.8)
            wrong_heading = (self.task.name == "left_to_right_reverse" and
                             abs(angle_error(self.pose[2], math.pi)) > math.pi/3)
            no_progress = self.idle_steps >= 100
            normalized = (executed/[.2, .6]).astype(np.float32)
            reward = (8*progress-.02-.1*float(blocked)
                      -.02*float(np.sum((normalized-self.previous)**2)))
            reward += 20 if success else -20 if hit or off_route or wrong_heading else -5 if no_progress else 0
            self.previous = normalized
            terminated = bool(success or hit or off_route or wrong_heading or no_progress)
            truncated = bool(self.steps >= self.max_steps and not terminated)
            info = {"task": self.task.name, "success": success, "collision": hit,
                    "off_route": off_route, "wrong_heading": wrong_heading,
                    "no_progress": no_progress, "safety_blocked": bool(blocked),
                    "requested_action": requested.tolist(), "executed_action": executed.tolist(),
                    "distance_to_goal": d, "sim_seconds": self.steps*self.dt,
                    "heading_error_rad": abs(angle_error(self.pose[2], self.task.goal[2])),
                    "lateral_error_m": float(abs(np.dot(self.pose[:2]-self.task.goal[:2],
                        [-math.sin(self.task.goal[2]), math.cos(self.task.goal[2])])))}
            observation = self._observation()
            if terminated or truncated:
                self.active = False
                self._publish((0., 0.))
                self._drain()
            return observation, float(reward), terminated, truncated, info
        except Exception:
            self.active = False
            self._publish((0., 0.))
            self._drain()
            raise  # Infrastructure faults are NOT invented RL transitions.

    def close(self):
        self._publish((0., 0.))
        self._drain()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()
