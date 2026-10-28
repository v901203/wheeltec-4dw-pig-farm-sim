"""Corner-post masking regression; synthetic scans, no robot or DDS output."""

from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_patrol_controller import TraditionalPatrolController
from navigation import (Config, MAX_RANGE, SensorFault, collision_detected,
                        command_clearance_masks, lidar_ignore_mask, load_config,
                        scan_features)

try:
    from fsm_lidar_patrol import LidarPatrolNode
except ImportError:
    LidarPatrolNode = None


ORIGINAL_SECTORS = ((33., 43.), (131., 141.), (-145., -134.), (-42., -30.))


class CornerMaskTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()
        self.angles = np.linspace(-np.pi, np.pi, 3601)

    def scan(self, angle=None, distance=.2):
        ranges = np.full(self.angles.shape, np.inf)
        if angle is not None:
            ranges[np.argmin(abs(self.angles - np.deg2rad(angle)))] = distance
        return SimpleNamespace(ranges=ranges, angle_min=-np.pi,
                               angle_increment=self.angles[1] - self.angles[0],
                               range_min=.1, range_max=25.)

    def test_defaults_match_config_with_four_centred_fifteen_degree_sectors(self):
        self.assertEqual(tuple(map(tuple, self.cfg.lidar_ignore_sectors_deg)),
                         Config().lidar_ignore_sectors_deg)
        for old, new in zip(ORIGINAL_SECTORS, self.cfg.lidar_ignore_sectors_deg):
            self.assertEqual(new[1] - new[0], 15)
            self.assertEqual(sum(new) / 2, sum(old) / 2)
        self.assertEqual(sum(hi - lo for lo, hi in self.cfg.lidar_ignore_sectors_deg), 60)

    def test_primary_detection_sectors_remain_visible(self):
        degrees = np.rad2deg(self.angles)
        protected = ((abs(degrees) <= 15) | (abs(degrees) >= 150) |
                     ((abs(degrees) >= 60) & (abs(degrees) <= 120)))
        self.assertFalse(lidar_ignore_mask(self.angles, self.cfg)[protected].any())

    def test_new_edge_returns_are_ignored_by_all_clearance_guards(self):
        old_cfg = replace(self.cfg, lidar_ignore_sectors_deg=ORIGINAL_SECTORS)
        for lo, hi in self.cfg.lidar_ignore_sectors_deg:
            for angle in (lo, lo + .1, hi - .1, hi):
                with self.subTest(angle=angle):
                    scan = self.scan(angle)
                    self.assertTrue(collision_detected(scan_features(scan, old_cfg), old_cfg))
                    features = scan_features(scan, self.cfg)
                    self.assertFalse(collision_detected(features, self.cfg))
                    self.assertTrue(np.all(features.ranges == MAX_RANGE))
                    for action in ((.2, 0), (-.2, 0), (0, .3), (.2, .1)):
                        masks, _ = command_clearance_masks(action, features, self.cfg)
                        self.assertFalse(any(mask.any() for mask in masks.values()))

    def test_obstacles_outside_mask_still_trigger_footprint_guard(self):
        angles = [0, 90, -90, 180]
        for lo, hi in self.cfg.lidar_ignore_sectors_deg:
            angles.extend((lo - .5, hi + .5))
        for angle in angles:
            with self.subTest(angle=angle):
                self.assertTrue(collision_detected(scan_features(self.scan(angle), self.cfg), self.cfg))

    def test_front_and_rear_obstacles_still_block_travel(self):
        for angle, action in ((0, (.2, 0)), (180, (-.2, 0))):
            with self.subTest(angle=angle):
                features = scan_features(self.scan(angle, .35), self.cfg)
                masks, _ = command_clearance_masks(action, features, self.cfg)
                self.assertTrue(masks["travel"].any())

    def test_invalid_front_data_is_not_hidden_by_self_mask(self):
        scan = self.scan(0, float("nan"))
        # A single invalid beam retains the existing conservative zero range;
        # enough invalid beams in the front sector reject the whole scan.
        self.assertTrue(collision_detected(scan_features(scan, self.cfg), self.cfg))
        scan.ranges[abs(self.angles) < np.deg2rad(2)] = np.nan
        with self.assertRaises(SensorFault):
            scan_features(scan, self.cfg)

    def test_mask_uses_body_frame_after_lidar_yaw(self):
        # Sensor +22.5 degrees becomes body +32.5 after yaw correction.
        scan = self.scan(22.5)
        self.assertTrue(collision_detected(scan_features(scan, self.cfg), self.cfg))
        rotated = replace(self.cfg, lidar_yaw=np.deg2rad(10))
        self.assertFalse(collision_detected(scan_features(scan, rotated), rotated))

    @unittest.skipIf(LidarPatrolNode is None, "Source ROS 2 for raw-hazard adapter")
    def test_raw_hazard_uses_same_mask_and_preserves_visible_obstacles(self):
        for angle, ignored in ((30.5, True), (45.5, True), (128.5, True),
                               (143.5, True), (-147., True), (-132., True),
                               (-43.5, True), (-28.5, True), (0, False),
                               (90, False), (-90, False), (180, False)):
            with self.subTest(angle=angle):
                node = SimpleNamespace(cfg=self.cfg, last_command=(.2, 0),
                                       controller=TraditionalPatrolController(self.cfg))
                result = LidarPatrolNode.measured_hazard(node, self.scan(angle))
                if ignored:
                    self.assertIsNone(result)
                    self.assertIsNone(node.controller.failure)
                else:
                    np.testing.assert_array_equal(result, [0, 0])
                    self.assertEqual(node.controller.failure, "obstacle_clearance")


if __name__ == "__main__":
    unittest.main()
