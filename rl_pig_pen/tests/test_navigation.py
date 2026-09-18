import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation import (Config, SensorFault, collision_detected, load_config,
                        local_reward, safe_command, scan_features, wrap_angle)
from patrol_controller import PatrolController
from model_utils import (PATROL_MODEL_NAME, PATROL_POLICY_VERSION, latest_checkpoint,
                         validate_model, validate_patrol_version)


def scan(ranges, start=-math.pi, increment=math.pi / 180):
    return NS(ranges=ranges, angle_min=start, angle_increment=increment,
              range_min=0.1, range_max=25.0)


def corridor_scan(left=0.65, right=0.65):
    angles = np.linspace(-math.pi, math.pi, 361)
    sine = np.sin(angles)
    ranges = np.full(361, np.inf)
    np.divide(np.where(sine > 0, left, right), np.abs(sine), out=ranges, where=np.abs(sine) > 1e-6)
    ranges[ranges > 25] = np.inf
    return scan(ranges)


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_shape_bounds_and_center(self):
        f = scan_features(corridor_scan(), self.cfg)
        self.assertEqual(f.observation.shape, (38,))
        self.assertEqual(f.observation.dtype, np.float32)
        self.assertAlmostEqual(f.balance_m, 0, places=5)
        self.assertTrue(np.all((f.observation >= 0) & (f.observation <= 1)))
        self.assertTrue(f.walls_present(self.cfg))

    def test_balance_sign_and_symmetric_penalty(self):
        a = scan_features(corridor_scan(0.8, 0.4), self.cfg)
        b = scan_features(corridor_scan(0.4, 0.8), self.cfg)
        self.assertGreater(a.balance_m, 0)
        self.assertLess(b.balance_m, 0)
        self.assertAlmostEqual(local_reward(0.3, a.balance_m), local_reward(0.3, b.balance_m))
        self.assertLess(local_reward(0.3, a.balance_m), local_reward(0.3, 0))
        self.assertAlmostEqual(local_reward(0, 0), -0.05)
        self.assertEqual(local_reward(1, 0, collision=True), -100)

    def test_full_resolution_front_obstacle_between_samples(self):
        raw = np.full(361, 10.0)
        raw[181] = 0.6
        f = scan_features(scan(raw), self.cfg)
        self.assertAlmostEqual(f.front_m, 0.6)
        self.assertTrue(np.all(f.observation[:36] == 1))
        self.assertFalse(collision_detected(f, self.cfg))
        command, stopped = safe_command([1, 0], f, self.cfg)
        self.assertTrue(stopped)
        np.testing.assert_array_equal(command, [0, 0])

    def test_end_wall_requires_front_wall_and_side_walls(self):
        raw = corridor_scan(0.65, 0.65).ranges.copy()
        raw[165:196] = 0.6
        end = scan_features(scan(raw), self.cfg)
        self.assertTrue(end.end_wall_present(self.cfg))
        raw[80:101] = 2.0
        open_side = scan_features(scan(raw), self.cfg)
        self.assertFalse(open_side.end_wall_present(self.cfg))

    def test_angles_use_metadata_not_array_indices(self):
        original = corridor_scan(0.8, 0.4)
        reverse = scan(original.ranges[::-1], math.pi, -math.pi / 180)
        a, b = (scan_features(s, self.cfg) for s in (original, reverse))
        self.assertAlmostEqual(a.balance_m, b.balance_m)
        self.assertAlmostEqual(a.front_m, b.front_m)

    def test_invalid_data_is_not_open_space(self):
        for values in (np.full(361, np.nan), np.full(361, -np.inf), np.zeros(361)):
            with self.assertRaises(SensorFault):
                scan_features(scan(values), self.cfg)
        self.assertTrue(scan_features(scan(np.full(361, np.inf)), self.cfg).both_open(self.cfg))
        with self.assertRaises(SensorFault):
            scan_features(scan(np.ones(36), start=0), self.cfg)

    def test_footprint_and_turn_clearance(self):
        self.assertFalse(collision_detected(scan_features(corridor_scan(0.45, 0.45), self.cfg), self.cfg))
        raw = np.full(361, 10.0)
        raw[180] = 0.30
        f = scan_features(scan(raw), self.cfg)
        self.assertFalse(collision_detected(f, self.cfg))
        _, stopped = safe_command([0, 0.5], f, self.cfg)
        self.assertTrue(stopped)
        _, stopped = safe_command([0, 0.5], f, self.cfg, check_turn_clearance=False)
        self.assertFalse(stopped)

    def test_progress_reward_uses_measured_displacement(self):
        self.assertAlmostEqual(local_reward(0.2, 0.1), 1.05)
        self.assertLess(local_reward(-0.2, 0), local_reward(0, 0))
        self.assertAlmostEqual(local_reward(0.2, 0, actual_dt=1.0), 0.6)

    def test_sparse_rail_returns_are_not_an_opening(self):
        raw = np.full(361, 2.0)
        # About 14% of each narrow side sector hits a bar; the rest goes through.
        raw[260:263] = 0.65
        raw[80:83] = 0.65
        f = scan_features(scan(raw), self.cfg)
        self.assertTrue(f.walls_present(self.cfg))
        self.assertFalse(f.both_open(self.cfg))


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.fsm = PatrolController(self.cfg)
        self.fsm.reset((0, 0, 0), 0)
        self.wall = scan_features(corridor_scan(), self.cfg)
        self.open = scan_features(scan(np.full(361, 10.0)), self.cfg)

    def test_start_outside_does_not_count_as_junction(self):
        for i in range(5):
            self.fsm.command(self.open, (0.1*i, 0, 0), i*0.1)
        self.assertEqual(self.fsm.junction, 0)
        self.assertEqual(self.fsm.state, "APPROACH")

    def test_missing_junction_times_out_without_odometry(self):
        self.fsm.command(self.wall, (0, 0, 0), 91)
        self.assertEqual(self.fsm.failure, "state_timeout")
        self.assertFalse(self.fsm.info()["success"])
        self.assertEqual(self.fsm.info()["failed_state"], "APPROACH")

    def test_policy_in_place_rotation_is_motion_but_has_state_timeout(self):
        self.fsm.state = "MAIN"
        for stamp in range(1, 91):
            self.fsm.command(self.wall, (0, 0, wrap_angle(stamp*0.1)), stamp)
            self.assertIsNone(self.fsm.failure)
        self.fsm.command(self.wall, (0, 0, wrap_angle(9.1)), 91)
        self.assertEqual(self.fsm.failure, "state_timeout")
        self.assertEqual(self.fsm.info()["state_sim_seconds"], 91)

    def test_lidar_only_controller_does_not_infer_stuck_from_pose(self):
        self.fsm.state = "MAIN"
        for stamp in range(1, 12):
            self.fsm.command(self.wall, (0.01*(stamp % 2), 0, 0.02*(stamp % 2)), stamp)
        self.assertIsNone(self.fsm.failure)

    def test_translation_alone_is_still_valid_motion(self):
        self.fsm.state = "MAIN"
        for stamp in range(1, 21):
            self.fsm.command(self.wall, (0.06*stamp, 0, 0), stamp)
        self.assertIsNone(self.fsm.failure)

    def test_front_blockage_is_not_branch_completion(self):
        self.fsm.state = "OUTBOUND"
        self.fsm.walls_seen = True
        self.fsm.distance = 3.0
        s = corridor_scan()
        s.ranges[180] = 0.5
        f = scan_features(s, self.cfg)
        for i in range(4):
            self.fsm.command(f, (0.01*i, 0, 0), i*0.1)
        self.assertEqual(len(self.fsm.reached), 0)
        self.assertEqual(self.fsm.state, "OUTBOUND")

    def test_reached_without_return_is_not_success(self):
        self.fsm.reached = {(j, s) for j in range(1, 4) for s in (0, 1)}
        self.assertEqual(self.fsm.info()["coverage"], 1)
        self.assertEqual(self.fsm.info()["return_rate"], 0)
        self.assertFalse(self.fsm.info()["success"])

    def test_complete_patrol_in_world_layout(self):
        """Kinematic integration with solid pen outlines, not a Gazebo/RL benchmark."""
        root = ET.parse(Path(__file__).resolve().parents[2] / "worlds/pig_pen_16units.world")
        boxes = []
        for model in root.findall("./world/model"):
            if model.get("name", "").startswith("pen"):
                x, y, *_ = map(float, model.findtext("pose").split())
                sx, sy, _ = map(float, model.findtext("./link[@name='floor']/collision/geometry/box/size").split())
                boxes.append([x-sx/2, y-sy/2, x+sx/2, y+sy/2])
            elif model.get("name") == "corridor_end_walls":
                for link in model.findall("link"):
                    x, y, *_ = map(float, link.findtext("pose").split())
                    sx, sy, _ = map(float, link.findtext("./collision/geometry/box/size").split())
                    boxes.append([x-sx/2, y-sy/2, x+sx/2, y+sy/2])
        boxes = np.array(boxes)
        angles = np.linspace(-math.pi, math.pi, 361)
        pose = np.array(self.cfg.start, dtype=float)
        self.fsm.reset(tuple(pose), 0)
        states = set()
        transitions = []
        previous_state = self.fsm.state
        for step in range(12000):
            directions = np.column_stack((np.cos(angles+pose[2]), np.sin(angles+pose[2])))
            with np.errstate(divide="ignore", invalid="ignore"):
                t1 = (boxes[None, :, :2] - pose[:2]) / directions[:, None, :]
                t2 = (boxes[None, :, 2:] - pose[:2]) / directions[:, None, :]
            entry = np.max(np.minimum(t1, t2), axis=2)
            leave = np.min(np.maximum(t1, t2), axis=2)
            hits = np.where((leave >= entry) & (entry > 0), entry, np.inf)
            ranges = np.min(hits, axis=1)
            ranges[ranges > 25] = np.inf
            f = scan_features(scan(ranges), self.cfg)
            self.assertFalse(collision_detected(f, self.cfg), (step, pose, self.fsm.state))
            command = self.fsm.command(f, tuple(pose), step*self.cfg.control_dt)
            states.add(self.fsm.state)
            if self.fsm.state != previous_state:
                transitions.append((step, previous_state, self.fsm.state, tuple(pose)))
                previous_state = self.fsm.state
            if self.fsm.state in ("DONE", "FAILED"):
                break
            if command is None:
                # Perfect-policy oracle used only to validate LiDAR FSM route
                # transitions; production PPO receives no world heading.
                branch_states = {"ENTRY", "OUTBOUND", "RETURN", "RETURN_CENTER"}
                target = math.pi if self.fsm.state in branch_states else math.pi / 2.0
                command = [-0.2 if self.fsm.expect_reverse() else 0.2,
                           np.clip(2*wrap_angle(target-pose[2]), -0.6, 0.6)]
            command, _ = safe_command(command, f, self.cfg, allow_reverse=True)
            v, w = command
            pose[0] += v*math.cos(pose[2])*self.cfg.control_dt
            pose[1] += v*math.sin(pose[2])*self.cfg.control_dt
            pose[2] = wrap_angle(pose[2]+w*self.cfg.control_dt)
        self.assertEqual(self.fsm.state, "DONE", (step, pose, self.fsm.info(), states, transitions,
                                                     f.wall_parallel_error, f.wall_alignment_confidence,
                                                     f.front_wall_m, f.left_edge_m, f.right_edge_m))
        self.assertEqual(self.fsm.returned, {(j, s) for j in range(1, 4) for s in (0, 1)})
        self.assertEqual(self.fsm.info()["return_rate"], 1)


class ModelTests(unittest.TestCase):
    def test_old_49d_models_are_not_auto_selected_or_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "ppo_patrol49_final.zip").touch()
            self.assertIsNone(latest_checkpoint(directory, prefix=PATROL_MODEL_NAME))
            new = Path(directory, PATROL_MODEL_NAME + "_final.zip")
            new.touch()
            self.assertEqual(latest_checkpoint(directory, prefix=PATROL_MODEL_NAME), new)
        with self.assertRaisesRegex(ValueError, "patrol49_v2"):
            validate_patrol_version(NS())
        validate_patrol_version(NS(navigation_version=PATROL_POLICY_VERSION))

    def test_old_checkpoints_are_not_selected(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "ppo_pig_pen_final.zip").touch()
            self.assertIsNone(latest_checkpoint(directory))
            new = Path(directory, "ppo_corridor38_20000_steps.zip")
            new.touch()
            self.assertEqual(latest_checkpoint(directory), new)

    def test_incompatible_shape_rejected(self):
        import gymnasium as gym
        old = NS(observation_space=gym.spaces.Box(0, 1, (39,), dtype=np.float32))
        with self.assertRaisesRegex(ValueError, "does not match"):
            validate_model(old, gym.spaces.Box(0, 1, (38,), dtype=np.float32), None)

    def test_default_configuration_loads(self):
        self.assertEqual(load_config().junctions, 3)


if __name__ == "__main__":
    unittest.main()
