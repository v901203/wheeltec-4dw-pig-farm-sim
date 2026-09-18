"""Regression tests for sideways-wall aliasing; no ROS or ideal heading inputs."""

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_geometry import axis_angle
from navigation import (Config, corridor_progress, patrol_reward, scan_features)
from patrol_controller import PatrolController


def rotated_corridor(yaw=0.0, offset=0.0, rays=1640, sparse=False):
    """Two actual parallel lines, rotated relative to the vehicle x-axis."""
    angles = np.linspace(-math.pi, math.pi, rays)
    sine = np.sin(angles + yaw)
    ranges = np.full(rays, np.inf)
    np.divide(np.where(sine > 0, 0.65-offset, 0.65+offset), np.abs(sine),
              out=ranges, where=np.abs(sine) > 1e-8)
    if sparse:
        # 15% near-wall returns, other rays continue to structures behind rails.
        rng = np.random.default_rng(42)
        keep = rng.random(rays) < 0.15
        ranges[~keep] *= 3.0
        ranges += rng.normal(0, 0.003, rays)
    ranges[ranges > 25] = np.inf
    return NS(ranges=ranges, angle_min=angles[0], angle_increment=angles[1]-angles[0],
              range_min=0.1, range_max=25.0)


class AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_full_yaw_sweep_including_perpendicular_and_off_centre(self):
        for rays in (361, 1640):
            for offset in (-0.15, 0.0, 0.15):
                for degrees in range(-90, 91, 5):
                    with self.subTest(rays=rays, offset=offset, degrees=degrees):
                        yaw = math.radians(degrees)
                        f = scan_features(rotated_corridor(yaw, offset, rays), self.cfg)
                        self.assertGreater(f.wall_alignment_confidence, 0.9)
                        self.assertLess(abs(axis_angle(f.wall_parallel_error+yaw)), math.radians(0.5))
                        self.assertAlmostEqual(abs(f.wall_balance_m), 2*abs(offset), places=4)
                        self.assertFalse(f.junction_open(self.cfg))
                        self.assertFalse(f.junction_open(self.cfg, both=False))
                        self.assertFalse(f.end_wall_present(self.cfg))

    def test_sparse_noisy_rails_keep_orientation(self):
        for degrees in (-90, -60, -30, 0, 30, 60, 90):
            with self.subTest(degrees=degrees):
                yaw = math.radians(degrees)
                f = scan_features(rotated_corridor(yaw, sparse=True), self.cfg)
                self.assertGreater(f.wall_alignment_confidence, 0.5)
                self.assertLess(abs(axis_angle(f.wall_parallel_error+yaw)), math.radians(3))
                self.assertFalse(f.junction_open(self.cfg))

    def test_actual_world_rail_geometry_at_lidar_height(self):
        # Raycast the actual static collision bars, NOT solid pen floor outlines.
        # No Gazebo physics, dynamic pigs or trained policy are simulated here.
        world = ET.parse(Path(__file__).resolve().parents[2] / "worlds/pig_pen_16units.world")
        boxes, circles = [], []
        def xyz(element):
            pose = np.fromstring(element.findtext("pose", "0 0 0 0 0 0"), sep=" ")
            np.testing.assert_array_equal(pose[3:], 0)  # This world is axis-aligned.
            return pose[:3]
        for model in world.findall("./world/model"):
            for link in model.findall("link"):
                for collision in link.findall("collision"):
                    position = xyz(model) + xyz(link) + xyz(collision)
                    box = collision.find("geometry/box")
                    cylinder = collision.find("geometry/cylinder")
                    if box is not None:
                        size = np.fromstring(box.findtext("size"), sep=" ")
                        if abs(position[2]-0.245) <= size[2]/2:
                            boxes.append([*(position[:2]-size[:2]/2), *(position[:2]+size[:2]/2)])
                    if cylinder is not None and abs(position[2]-0.245) <= float(cylinder.findtext("length"))/2:
                        circles.append([*position[:2], float(cylinder.findtext("radius"))])
        boxes, circles = np.array(boxes), np.array(circles)
        angles = np.linspace(-math.pi, math.pi, 1640)
        for y in (-8.5, -8.0):
            for degrees in (0, 15, 30, 60, 80, 90):
                with self.subTest(y=y, degrees=degrees):
                    xy = np.array([0, y])
                    yaw = math.pi/2 + math.radians(degrees)
                    directions = np.column_stack((np.cos(angles+yaw), np.sin(angles+yaw)))
                    with np.errstate(divide="ignore", invalid="ignore"):
                        t1 = (boxes[None, :, :2]-xy) / directions[:, None, :]
                        t2 = (boxes[None, :, 2:]-xy) / directions[:, None, :]
                    entry = np.max(np.minimum(t1, t2), axis=2)
                    leave = np.min(np.maximum(t1, t2), axis=2)
                    ranges = np.min(np.where((leave >= entry) & (entry > 0), entry, np.inf), axis=1)
                    nearby = np.linalg.norm(circles[:, :2]-xy, axis=1) < 4
                    relative, radii = circles[nearby, :2]-xy, circles[nearby, 2]
                    projection = directions @ relative.T
                    cross = directions[:, 0, None]*relative[None, :, 1] - directions[:, 1, None]*relative[None, :, 0]
                    discriminant = radii**2-cross**2
                    hits = projection-np.sqrt(np.maximum(0, discriminant))
                    ranges = np.minimum(ranges, np.min(np.where((discriminant >= 0) & (hits > 0), hits, np.inf), axis=1))
                    ranges[ranges > 25] = np.inf
                    scan = NS(ranges=ranges, angle_min=angles[0], angle_increment=angles[1]-angles[0],
                              range_min=.1, range_max=25.)
                    f = scan_features(scan, self.cfg)
                    self.assertGreater(f.wall_alignment_confidence, .8)
                    self.assertLess(abs(axis_angle(f.wall_parallel_error+math.radians(degrees))), math.radians(1))
                    self.assertFalse(f.junction_open(self.cfg))
                    self.assertFalse(f.end_wall_present(self.cfg))

    def test_reversed_scan_metadata_and_lidar_mount_yaw(self):
        original = rotated_corridor(math.radians(40))
        reversed_scan = NS(**vars(original))
        reversed_scan.ranges = original.ranges[::-1]
        reversed_scan.angle_min = math.pi
        reversed_scan.angle_increment *= -1
        for scan in (original, reversed_scan):
            f = scan_features(scan, self.cfg)
            self.assertAlmostEqual(f.wall_parallel_error, math.radians(-40), places=5)
        mounted = scan_features(original, replace(self.cfg, lidar_yaw=math.radians(10)))
        self.assertAlmostEqual(mounted.wall_parallel_error, math.radians(-30), places=5)

    def test_single_wall_or_no_wall_is_unknown_not_parallel(self):
        for one_wall in (True, False):
            scan = rotated_corridor()
            angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
            scan.ranges[angles < 0 if one_wall else np.ones(len(angles), dtype=bool)] = np.inf
            f = scan_features(scan, self.cfg)
            self.assertEqual(f.wall_alignment_confidence, 0.0)
            self.assertFalse(f.aligned())

    def test_rotating_in_corridor_cannot_earn_route_events(self):
        for state in ("MAIN", "RETURN", "OUTBOUND"):
            fsm = PatrolController(self.cfg)
            fsm.reset(stamp=0)
            fsm.state = state
            fsm.walls_seen = True
            stamp = 0.0
            for degrees in (0, 30, 60, 80, 90, -80, -60, -30):
                f = scan_features(rotated_corridor(math.radians(degrees)), self.cfg)
                for _ in range(self.cfg.confirm_frames+1):
                    stamp += self.cfg.control_dt
                    fsm.command(f, stamp)
                self.assertEqual(fsm.state, state)
                self.assertEqual(fsm.junction, 0)
                self.assertFalse(fsm.reached)
                self.assertFalse(fsm.returned)

    def test_four_way_opening_and_real_end_wall_still_work(self):
        opening = rotated_corridor()
        angles = opening.angle_min + np.arange(len(opening.ranges))*opening.angle_increment
        # Four solid quadrants beyond a 1.3m-wide cross-shaped opening.
        opening.ranges = np.minimum(25, np.maximum(0.65/np.maximum(abs(np.cos(angles)), 1e-8),
                                                   0.65/np.maximum(abs(np.sin(angles)), 1e-8)))
        f = scan_features(opening, self.cfg)
        self.assertTrue(f.junction_open(self.cfg))
        self.assertEqual(f.wall_alignment_confidence, 0.0)  # Both axes are plausible here.
        fsm = PatrolController(self.cfg)
        fsm.reset(stamp=0)
        fsm.state = "MAIN"
        for i in range(self.cfg.confirm_frames):
            fsm.command(f, (i+1)*self.cfg.control_dt)
        self.assertEqual((fsm.state, fsm.junction), ("CENTER", 1))
        end = rotated_corridor()
        forward = np.cos(angles) > 0
        end.ranges[forward] = np.minimum(end.ranges[forward], 0.8/np.cos(angles[forward]))
        self.assertTrue(scan_features(end, self.cfg).end_wall_present(self.cfg))

    def test_completed_branches_still_require_final_wall(self):
        fsm = PatrolController(self.cfg)
        fsm.reset(stamp=0)
        fsm.state, fsm.junction = "MAIN", 3
        fsm.returned = {(j, s) for j in range(1, 4) for s in (0, 1)}
        scan = rotated_corridor()
        scan.ranges[:] = np.inf
        f = scan_features(scan, self.cfg)
        for i in range(5):
            fsm.command(f, (i+1)*self.cfg.control_dt)
        self.assertEqual(fsm.state, "MAIN")
        self.assertFalse(fsm.info()["success"])


class AlignmentRewardTests(unittest.TestCase):
    def test_forward_reward_decreases_with_deviation(self):
        rewards = []
        for degrees in (0, 30, 60, 90):
            yaw = math.radians(degrees)
            f = scan_features(rotated_corridor(yaw), Config())
            start = (0, 0, yaw)
            end = (0.02*math.cos(yaw), 0.02*math.sin(yaw), yaw)
            progress = corridor_progress(start, end, f)
            self.assertAlmostEqual(progress, 0.02*math.cos(yaw))
            reward, terms = patrol_reward(progress, f)
            self.assertAlmostEqual(reward, sum(terms.values()))
            rewards.append(reward)
        self.assertTrue(all(a > b for a, b in zip(rewards, rewards[1:])))
        self.assertAlmostEqual(rewards[0], 0.07)
        self.assertAlmostEqual(rewards[-1], -0.15)

    def test_reverse_intent_not_command_sign_defines_progress(self):
        f = scan_features(rotated_corridor(math.radians(20)), Config())
        start = (0, 0, math.radians(20))
        for reverse, dx, expected in ((False, .02, .02), (False, -.02, -.02),
                                      (True, -.02, .02), (True, .02, -.02)):
            progress = corridor_progress(start, (dx, 0, start[2]), f, reverse=reverse)
            self.assertAlmostEqual(progress, expected)

    def test_rotation_unknown_geometry_and_stall_do_not_earn_progress(self):
        f = scan_features(rotated_corridor(), Config())
        self.assertEqual(corridor_progress((0, 0, 0), (0, 0, 1), f), 0)
        unknown = replace(f, wall_alignment_confidence=0.0)
        self.assertEqual(corridor_progress((0, 0, 0), (0.02, 0, 0), unknown), 0)
        self.assertLess(patrol_reward(0, unknown)[0], 0)
        self.assertLess(patrol_reward(0, f)[0], 0)

    def test_measured_progress_clipped_and_collision_overrides(self):
        f = scan_features(rotated_corridor(), Config())
        self.assertAlmostEqual(corridor_progress((0, 0, 0), (10, 0, 0), f, actual_dt=.1), .15)
        reward, terms = patrol_reward(10, f, collision=True)
        self.assertEqual(reward, -100)
        self.assertEqual(sum(terms.values()), reward)


if __name__ == "__main__":
    unittest.main()
