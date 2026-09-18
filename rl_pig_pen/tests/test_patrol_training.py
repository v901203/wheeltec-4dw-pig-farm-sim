"""Exercise complete training episodes with real FSM and kinematic sensors."""

import contextlib
import io
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation import SensorFault, wrap_angle
from patrol_training_env import PatrolTrainingEnv
from test_environment import snapshot


class PatrolTrainingTests(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        for target in ("_RosNode", "SingleThreadedExecutor", "threading.Thread"):
            self.stack.enter_context(patch("pig_pen_env." + target))
        self.stack.enter_context(patch("pig_pen_env.rclpy.ok", return_value=True))
        self.stack.enter_context(patch("pig_pen_env._teleport", side_effect=self.teleport))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.env = PatrolTrainingEnv()
        self.addCleanup(self.env.close)
        self.command = np.zeros(2)
        self.stamp = 1.0
        self.teleports = 0
        self.pose = np.array(self.env.cfg.start)
        self.angles = np.linspace(-math.pi, math.pi, 361)
        self.boxes = []
        world = ET.parse(Path(__file__).resolve().parents[2] / "worlds/pig_pen_16units.world")
        for model in world.findall("./world/model"):
            if model.get("name", "").startswith("pen"):
                x, y, *_ = map(float, model.findtext("pose").split())
                sx, sy, _ = map(float, model.findtext("./link[@name='floor']/collision/geometry/box/size").split())
                self.boxes.append([x-sx/2, y-sy/2, x+sx/2, y+sy/2])
        self.boxes = np.array(self.boxes)
        self.env._ros.publish_cmd.side_effect = self.publish
        self.env._wait_sensor = self.sensor

    def teleport(self, cfg, pose):
        self.pose = np.array(pose, dtype=float)
        self.teleports += 1

    def publish(self, v, w):
        self.command = np.array([v, w])

    def sensor(self, min_stamp=None, **kwargs):
        if min_stamp is not None:
            dt = max(0.0, min_stamp-self.stamp)
            self.stamp += dt
            v, w = self.command
            self.pose[0] += v*math.cos(self.pose[2])*dt
            self.pose[1] += v*math.sin(self.pose[2])*dt
            self.pose[2] = wrap_angle(self.pose[2]+w*dt)
        directions = np.column_stack((np.cos(self.angles+self.pose[2]), np.sin(self.angles+self.pose[2])))
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (self.boxes[None, :, :2]-self.pose[:2]) / directions[:, None, :]
            t2 = (self.boxes[None, :, 2:]-self.pose[:2]) / directions[:, None, :]
        entry = np.max(np.minimum(t1, t2), axis=2)
        leave = np.min(np.maximum(t1, t2), axis=2)
        ranges = np.min(np.where((leave >= entry) & (entry > 0), entry, np.inf), axis=1)
        ranges[ranges > 25] = np.inf
        scan, odom = snapshot(self.stamp, self.command[0], ranges)
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[:2]
        odom.pose.pose.orientation.z = math.sin(self.pose[2]/2)
        odom.pose.pose.orientation.w = math.cos(self.pose[2]/2)
        return scan, odom

    def test_six_branches_and_returns_form_one_episode(self):
        obs, info = self.env.reset(seed=0)
        self.assertGreater(info["fsm_steps"], 0)
        self.assertEqual(info["state"], "MAIN")
        fsm_frames, previous_done, previous_reached = 0, 0, 0
        for step in range(12000):
            # Ideal local follower isolates episode semantics from learning.
            action = np.array([0.2, np.clip(2*wrap_angle(self.env.controller.heading-self.pose[2]), -0.6, 0.6)])
            self.env._ros.publish_cmd.reset_mock()
            obs, reward, terminated, truncated, info = self.env.step(action)
            # The first published command must really be this PPO decision.
            np.testing.assert_allclose(self.env._ros.publish_cmd.call_args_list[0].args, action)
            fsm_frames += info["fsm_steps"]
            self.assertTrue(self.env.observation_space.contains(obs))
            self.assertEqual(info["controller"], "rl")
            self.assertFalse(info["collision"])
            if info["branches_done"] > previous_done or info["endpoints_reached"] > previous_reached:
                self.assertGreater(reward, 5.0)
            previous_done, previous_reached = info["branches_done"], info["endpoints_reached"]
            if terminated or truncated:
                break
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info["success"])
        self.assertEqual(info["branches_done"], 6)
        self.assertEqual(info["endpoints_reached"], 6)
        self.assertGreater(reward, 90)
        self.assertEqual(info["policy_steps"], step+1)
        self.assertGreater(fsm_frames, 0)
        self.assertGreater(info["physical_steps"], step+1)
        self.assertEqual(self.teleports, 1)
        with self.assertRaisesRegex(RuntimeError, "reset"):
            self.env.step(action)

    def test_collision_during_automatic_maneuver_ends_policy_transition(self):
        self.env.reset()
        advance = self.env._advance_to_policy
        def collide():
            scan, _ = self.env._latest_snapshot
            scan.ranges[180] = 0.2
            return advance()
        with patch.object(self.env, "_advance_to_policy", side_effect=collide):
            _, reward, terminated, truncated, info = self.env.step([0.2, 0])
        self.assertEqual(reward, -100)
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertEqual(info["failure_reason"], "collision")
        np.testing.assert_array_equal(self.command, [0, 0])

    def test_stuck_ends_full_route_without_false_completion(self):
        self.env.reset()
        for _ in range(140):
            _, reward, terminated, truncated, info = self.env.step([0, 0])
            if terminated or truncated:
                break
        self.assertTrue(terminated)
        self.assertFalse(info["success"])
        self.assertEqual(info["failure_reason"], "stuck")
        self.assertLess(reward, -20)

    def test_sensor_fault_stops_during_macro_transition(self):
        self.env.reset()
        with patch.object(self.env, "_advance_to_policy", side_effect=SensorFault("stale")):
            _, reward, terminated, truncated, info = self.env.step([0.2, 0])
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["failure_reason"], "sensor_fault")
        self.assertEqual(reward, -20)
        self.assertIn("return_rate", info)
        np.testing.assert_array_equal(self.command, [0, 0])


if __name__ == "__main__":
    unittest.main()
