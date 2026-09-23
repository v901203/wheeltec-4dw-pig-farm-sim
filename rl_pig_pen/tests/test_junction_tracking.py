"""Acquired junction tracking under wall occlusion and sensor noise."""

import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from junction_localization import JunctionTracker
from lidar_patrol_controller import TraditionalPatrolController
from navigation import load_config, scan_features
from lidar_scene import StaticLidarScene


class JunctionTrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = StaticLidarScene()
        cls.cfg = load_config()

    def features(self, pose, *, hide=(), noise_seed=None):
        scan = self.scene.scan(pose)
        finite = np.isfinite(scan.ranges)
        angles = self.scene.angles + pose[2]
        world_x = pose[0] + scan.ranges[finite] * np.cos(angles[finite])
        mask = np.zeros(len(scan.ranges), dtype=bool)
        mask[finite] = np.any([np.abs(world_x - x) < .06 for x in hide], axis=0) if hide else False
        scan.ranges[mask] = np.inf
        if noise_seed is not None:
            finite = np.isfinite(scan.ranges)
            scan.ranges[finite] += np.random.default_rng(noise_seed).normal(0, .03, np.sum(finite))
        return scan_features(scan, self.cfg)

    def acquire(self, pose):
        controller = TraditionalPatrolController(self.cfg)
        geometry = controller._geometry(self.features(pose))
        self.assertIsNotNone(geometry)
        return JunctionTracker(geometry, 0., self.cfg)

    def test_missing_wall_tracks_measured_translation_without_commands(self):
        tracker = self.acquire((0., -6.764, math.pi / 2))
        for i in range(1, 41):
            pose = (0., -6.764 + .01 * i, math.pi / 2)
            features = self.features(pose, hide=(-.625,))
            if i == 1:
                self.assertIsNone(TraditionalPatrolController(self.cfg)._geometry(features))
            geometry = tracker.observe_walls(features)
            self.assertIsNotNone(geometry, i)
            self.assertTrue(tracker.update(geometry, i / 12), tracker.failure)
            self.assertLess(abs(geometry.centre[0] - (.584 - .01 * i)), .025)
            self.assertLess(abs(geometry.centre[1]), .025)

    def test_missing_wall_preserves_axis_identity_through_turn(self):
        tracker = self.acquire((0., -6.18, math.pi / 2))
        for i in range(1, 19):
            turn = math.radians(5 * i)
            geometry = tracker.observe_walls(self.features((0., -6.18, math.pi / 2 + turn), hide=(-.625,)))
            self.assertIsNotNone(geometry, i)
            self.assertTrue(tracker.update(geometry, i / 12), tracker.failure)
            self.assertLess(abs(tracker.incoming_axis + turn), .025)
            self.assertLess(np.linalg.norm(geometry.centre), .025)

    def test_parallel_walls_alone_or_no_returns_cannot_track(self):
        tracker = self.acquire((0., -6.764, math.pi / 2))
        features = self.features((0., -6.764, math.pi / 2), hide=(-.625, .625))
        self.assertIsNone(tracker.observe_walls(features))
        features.ranges[:] = 10.
        self.assertIsNone(tracker.observe_walls(features))

    def test_large_translation_cannot_reassociate_another_wall(self):
        tracker = self.acquire((0., -6.764, math.pi / 2))
        self.assertIsNone(tracker.observe_walls(self.features((0., -6.264, math.pi / 2), hide=(-.625,))))

    def test_three_centimetre_noise_with_one_hidden_wall(self):
        tracker = self.acquire((0., -6.764, math.pi / 2))
        for seed in range(12):
            geometry = tracker.observe_walls(self.features((0., -6.764, math.pi / 2),
                                                          hide=(-.625,), noise_seed=seed))
            self.assertIsNotNone(geometry, seed)
            self.assertLess(np.linalg.norm(geometry.centre - [.584, 0.]), .04)

    def test_controller_centres_with_occlusion_but_cannot_acquire_from_it(self):
        pose = np.array([0., -6.764, math.pi / 2])
        controller = TraditionalPatrolController(self.cfg)
        hidden = self.features(pose, hide=(-.625,))
        self.assertIsNone(controller._geometry(hidden))
        controller.tracker = self.acquire(pose)
        controller.state = "CENTER"
        saw_fallback = False
        for i in range(1, 401):
            features = self.features(pose, hide=(-.625,))
            v, w = controller.command(features, i / 12)
            saw_fallback |= controller.diagnostics.get("geometry_source") == "tracked_walls"
            self.assertNotEqual(controller.state, "FAILED", controller.failure_diagnostics)
            if controller.state == "TURN_LEFT":
                break
            pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
        self.assertTrue(saw_fallback)
        self.assertEqual(controller.state, "TURN_LEFT")
        self.assertLess(np.linalg.norm(controller.tracker.geometry.centre), .02)


if __name__ == "__main__":
    unittest.main()
