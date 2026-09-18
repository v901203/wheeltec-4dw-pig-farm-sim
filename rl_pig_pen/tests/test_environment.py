"""Exercise Gym transitions with injected ROS snapshots; no simulator required."""

from pathlib import Path
from dataclasses import replace
import math
import sys
import time
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation import SensorFault
from pig_pen_env import PigPenEnv
from test_navigation import corridor_scan, scan


def snapshot(stamp=1.0, speed=0.0, ranges=None):
    s = corridor_scan() if ranges is None else scan(ranges)
    header = NS(stamp=NS(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9)))
    s.header = header
    odom = NS(header=header,
              pose=NS(pose=NS(position=NS(x=0.0, y=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
              twist=NS(twist=NS(linear=NS(x=speed))))
    return s, odom


class EnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.patches = [patch("pig_pen_env.rclpy.ok", return_value=True),
                        patch("pig_pen_env._RosNode"),
                        patch("pig_pen_env.SingleThreadedExecutor"),
                        patch("pig_pen_env.threading.Thread"),
                        patch("pig_pen_env._teleport")]
        for p in self.patches:
            p.start()
        self.env = PigPenEnv()
        self.env._wait_sensor = Mock(return_value=snapshot())
        self.env.reset(seed=7)

    def tearDown(self):
        self.env.close()
        for p in reversed(self.patches):
            p.stop()

    def test_observation_contract_and_seeded_resets(self):
        obs, _ = self.env.reset(seed=10)
        self.assertTrue(self.env.observation_space.contains(obs))
        self.assertEqual(self.env.observation_space.shape, (38,))
        a = self.env._training_spawn()
        self.env.reset(seed=10)
        self.assertEqual(a, self.env._training_spawn())

    def test_training_spawns_clear_pen_footprints_in_actual_world(self):
        """Two visible walls alone cannot distinguish an aisle from a pen."""
        world = ET.parse(Path(__file__).resolve().parents[2] / "worlds/pig_pen_16units.world")
        boxes = []
        for model in world.findall("./world/model"):
            if model.get("name", "").startswith("pen"):
                x, y, *_ = map(float, model.findtext("pose").split())
                sx, sy, _ = map(float, model.findtext("./link[@name='floor']/collision/geometry/box/size").split())
                boxes.append((x-sx/2, y-sy/2, x+sx/2, y+sy/2))
        self.assertEqual(len(boxes), 16)
        self.env.reset(seed=42)
        main_count, branch_count = 0, 0
        for _ in range(512):
            x, y, yaw = self.env._training_spawn()
            length = self.env.cfg.half_length + self.env.cfg.collision_margin
            width = self.env.cfg.half_width + self.env.cfg.collision_margin
            dx = length * abs(math.cos(yaw)) + width * abs(math.sin(yaw))
            dy = length * abs(math.sin(yaw)) + width * abs(math.cos(yaw))
            for left, bottom, right, top in boxes:
                overlaps = x+dx > left and x-dx < right and y+dy > bottom and y-dy < top
                self.assertFalse(overlaps, f"Spawn {(x, y, yaw)} overlaps pen {(left, bottom, right, top)}")
            if abs(x) < 0.1:
                main_count += 1
                self.assertLess(abs(math.cos(yaw)), 0.13)
            else:
                branch_count += 1
                self.assertLess(abs(math.sin(yaw)), 0.13)
        self.assertGreater(main_count, 0)
        self.assertGreater(branch_count, 0)

    def test_train_never_executes_fsm_and_charges_elapsed_time(self):
        self.env.controller.command = Mock(side_effect=AssertionError("FSM used in training"))
        self.env._wait_sensor.side_effect = [snapshot(1), snapshot(1.1, speed=0)]
        obs, reward, terminated, truncated, info = self.env.step([0.4, 0])
        self.assertAlmostEqual(reward, -0.06)
        self.assertFalse(terminated or truncated)
        self.assertEqual(info["controller"], "rl")
        self.assertTrue(self.env.observation_space.contains(obs))
        self.env._ros.publish_cmd.assert_called_with(0.0, 0.0)

    def test_opening_ends_training_segment_without_center_penalty(self):
        values = np.full(361, 10.0)
        self.env._wait_sensor.side_effect = [snapshot(1), snapshot(1.1, speed=0.2, ranges=values)]
        _, reward, terminated, truncated, info = self.env.step([0.2, 0])
        self.assertAlmostEqual(reward, -0.06)
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertTrue(info["segment_complete"])
        self.assertFalse(info["success"])
        with self.assertRaises(RuntimeError):
            self.env.step([0.2, 0])

    def test_corridor_policy_rotation_does_not_count_as_stuck(self):
        before = snapshot(12)
        after = snapshot(12.1)
        after[1].pose.pose.orientation.z = math.sin(0.2/2)
        after[1].pose.pose.orientation.w = math.cos(0.2/2)
        self.env._wait_sensor.side_effect = [before, after]
        _, _, terminated, truncated, info = self.env.step([0, 0.6])
        self.assertFalse(terminated or truncated)
        self.assertNotEqual(info.get("failure_reason"), "stuck")

    def test_collision_overrides_success(self):
        self.env.mode = "patrol"
        self.env.controller.state = "DONE"
        raw = corridor_scan().ranges.copy()
        raw[180] = 0.2
        self.env._wait_sensor.return_value = snapshot(1, ranges=raw)
        _, reward, terminated, truncated, info = self.env.step([0.2, 0])
        self.assertEqual(reward, -100)
        self.assertTrue(terminated)
        self.assertFalse(truncated or info["success"])
        self.env._ros.publish_cmd.assert_called_with(0.0, 0.0)

    def test_sensor_fault_stops_and_truncates(self):
        self.env._wait_sensor.side_effect = SensorFault("stale scan")
        _, reward, terminated, truncated, info = self.env.step([0.2, 0])
        self.assertEqual(reward, 0)
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["failure_reason"], "sensor_fault")
        self.env._ros.publish_cmd.assert_called_with(0.0, 0.0)

    def test_new_collision_exact_penalty(self):
        raw = corridor_scan().ranges.copy()
        raw[180] = 0.2
        self.env._wait_sensor.side_effect = [snapshot(1), snapshot(1.1, speed=1, ranges=raw)]
        _, reward, terminated, _, info = self.env.step([0.4, 0])
        self.assertEqual(reward, -100)
        self.assertTrue(terminated)
        self.assertTrue(info["collision"])

    def test_invalid_action_stops_before_raising(self):
        with self.assertRaises(ValueError):
            self.env.step([np.nan, 0])
        self.env._ros.publish_cmd.assert_called_with(0.0, 0.0)

    def test_reset_waits_for_consecutive_corridor_frames(self):
        clear = snapshot(2.0)
        opening = snapshot(2.0, ranges=np.full(361, 10.0))
        # Two good frames interrupted by one opening must not finish validation.
        self.env._wait_sensor.side_effect = [snapshot(), snapshot(),
                                             clear, clear, opening, clear, clear, clear]
        obs, info = self.env.reset(seed=0)
        self.assertTrue(self.env.observation_space.contains(obs))
        self.assertEqual(info["reset_attempts"], 1)
        self.assertEqual(info["reset_validation_frames"], 6)
        self.env._ros.publish_cmd.assert_called_with(0.0, 0.0)

    def test_reset_resamples_failed_spawn_with_a_bound(self):
        self.env.cfg = replace(self.env.cfg, reset_attempts=2, reset_validation_frames=3)
        # Match the startup failure: only the right side falls beyond 1 m.
        sparse_right = corridor_scan().ranges.copy()
        sparse_right[80:101] = 1.525
        bad = snapshot(2.0, ranges=sparse_right)
        self.env._wait_sensor.side_effect = [snapshot(), snapshot(), bad, bad, bad,
                                             snapshot(), snapshot(3), snapshot(3.1), snapshot(3.2)]
        with patch.object(self.env, "_training_spawn", wraps=self.env._training_spawn) as spawn:
            obs, info = self.env.reset(seed=0)
            self.assertEqual(spawn.call_count, 2)
        self.assertTrue(self.env.observation_space.contains(obs))
        self.assertEqual(info["reset_attempts"], 2)

    def test_persistent_open_space_is_not_accepted_as_corridor(self):
        self.env.cfg = replace(self.env.cfg, reset_attempts=2, reset_validation_frames=3)
        self.env._wait_sensor.return_value = snapshot(ranges=np.full(361, 10.0))
        with patch.object(self.env, "_training_spawn", wraps=self.env._training_spawn) as spawn:
            with self.assertRaisesRegex(RuntimeError, r"after 2 attempt.*left=10.000m"):
                self.env.reset(seed=0)
            self.assertEqual(spawn.call_count, 2)
        self.assertTrue(self.env._episode_done)
        self.env._ros.publish_cmd.assert_called_with(0.0, 0.0)

    def test_reset_does_not_accept_collision(self):
        self.env.cfg = replace(self.env.cfg, reset_attempts=1, reset_validation_frames=3)
        raw = corridor_scan().ranges.copy()
        raw[180] = 0.2
        self.env._wait_sensor.return_value = snapshot(ranges=raw)
        with self.assertRaisesRegex(RuntimeError, "collision=True"):
            self.env.reset(seed=0)

    def test_reset_recovers_from_transient_invalid_scan(self):
        invalid = snapshot(2, ranges=np.full(361, np.nan))
        self.env._wait_sensor.side_effect = [snapshot(), snapshot(), invalid,
                                             snapshot(3), snapshot(3.1), snapshot(3.2)]
        _, info = self.env.reset(seed=0)
        self.assertEqual(info["reset_attempts"], 1)
        self.assertEqual(info["reset_validation_frames"], 4)

    def test_wait_sensor_rejects_pre_teleport_cache(self):
        marker = time.monotonic()
        cached, fresh = snapshot(1), snapshot(2)
        self.env._ros.get_data.side_effect = [(*cached, marker-0.1, marker-0.1),
                                              (*fresh, marker+0.001, marker+0.001)]
        scan, _ = PigPenEnv._wait_sensor(self.env, min_received=marker)
        self.assertIs(scan, fresh[0])

    def test_patrol_reset_accepts_clear_open_start(self):
        self.env.mode = "patrol"
        self.env._wait_sensor.return_value = snapshot(ranges=np.full(361, 10.0))
        _, info = self.env.reset(seed=0)
        self.assertEqual(info["reset_attempts"], 1)
        self.assertEqual(info["reset_validation_frames"], self.env.cfg.confirm_frames)


if __name__ == "__main__":
    unittest.main()
