"""Closed-loop junction tests. Poses produce scans, never enter the controller."""

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from junction_localization import JunctionTracker, junction_geometry
from navigation import Config, scan_features, wrap_angle
from patrol_controller import PatrolController
from lidar_scene import StaticLidarScene


def cross_scan(pose=(0, 0, 0), *, rays=361, room=False):
    angles = np.linspace(-math.pi, math.pi, rays)
    # Open cross: x-directed road width 1.30m, y-directed road width 1.20m.
    boxes = np.array([[.6, .65, 8, 8], [-8, .65, -.6, 8],
                      [-8, -8, -.6, -.65], [.6, -8, 8, -.65]])
    if room:
        boxes = np.array([[-2, -.7, 2, -.65], [-2, .65, 2, .7],
                          [-.65, -2, -.6, 2], [.6, -2, .65, 2]])
    pose = np.asarray(pose)
    directions = np.column_stack((np.cos(angles+pose[2]), np.sin(angles+pose[2])))
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (boxes[None, :, :2]-pose[:2]) / directions[:, None, :]
        t2 = (boxes[None, :, 2:]-pose[:2]) / directions[:, None, :]
    entry = np.max(np.minimum(t1, t2), axis=2)
    leave = np.min(np.maximum(t1, t2), axis=2)
    ranges = np.min(np.where((leave >= entry) & (entry > 0), entry, np.inf), axis=1)
    ranges[ranges > 25] = np.inf
    return NS(ranges=ranges, angle_min=angles[0], angle_increment=angles[1]-angles[0],
              range_min=.1, range_max=25.)


class JunctionGeometryTests(unittest.TestCase):
    def test_all_world_junctions_with_actual_rail_gaps(self):
        cfg, scene = Config(), StaticLidarScene()
        for y in (-6.18, 0, 6.18):
            for offset in (-.3, 0, .3):
                for degrees in (0, 30, 60, 90):
                    yaw = math.pi/2+math.radians(degrees)
                    f = scan_features(scene.scan((0, y+offset, yaw)), cfg)
                    geometry = junction_geometry(f, cfg)
                    self.assertIsNotNone(geometry, (y, offset, degrees))
                    expected = -offset*np.array([math.sin(yaw), math.cos(yaw)])
                    np.testing.assert_allclose(geometry.centre, expected, atol=.015)

    def test_measured_centre_across_positions_and_orientations(self):
        cfg = Config()
        for xy in ((-.35, 0), (0, 0), (.15, -.12)):
            for degrees in range(-90, 91, 15):
                with self.subTest(xy=xy, degrees=degrees):
                    yaw = math.radians(degrees)
                    geometry = junction_geometry(scan_features(cross_scan((*xy, yaw)), cfg), cfg)
                    self.assertIsNotNone(geometry)
                    c, s = math.cos(yaw), math.sin(yaw)
                    expected = -np.array([[c, s], [-s, c]]) @ xy
                    np.testing.assert_allclose(geometry.centre, expected, atol=.025)

    def test_room_is_not_a_crossing(self):
        cfg = Config()
        self.assertIsNone(junction_geometry(scan_features(cross_scan(room=True), cfg), cfg))

    def test_motion_matches_geometry_not_commands(self):
        cfg = Config()
        def observe(pose):
            return junction_geometry(scan_features(cross_scan(pose), cfg), cfg)
        tracker = JunctionTracker(observe((0, 0, 0)), 0, cfg)
        self.assertTrue(tracker.update(observe((.02, -.01, .1)), .1))
        np.testing.assert_allclose(tracker.delta_pose, [.02, -.01, .1], atol=.015)
        self.assertTrue(tracker.update(observe((.02, -.01, .1)), .2))
        np.testing.assert_allclose(tracker.delta_pose, 0, atol=1e-6)

    def test_tracking_rejects_large_scan_gap_and_axis_jump(self):
        cfg = Config()
        first = junction_geometry(scan_features(cross_scan(), cfg), cfg)
        for pose, stamp, reason in (((0, 0, 0), .6, "junction_scan_gap"),
                                     ((0, 0, .5), .1, "junction_axis_jump"),
                                     ((.3, 0, 0), .1, "junction_position_jump")):
            tracker = JunctionTracker(first, 0, cfg)
            changed = junction_geometry(scan_features(cross_scan(pose), cfg), cfg)
            self.assertFalse(tracker.update(changed, stamp))
            self.assertEqual(tracker.failure, reason)


class JunctionControlTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.stamp = 0.0

    def start(self, pose=(0, 0, 0), state="CENTER"):
        fsm = PatrolController(self.cfg)
        fsm.reset(stamp=0)
        fsm.state = state
        fsm.junction = 1
        fsm.seen_open = True
        self.assertTrue(fsm._acquire_junction(scan_features(cross_scan(pose), self.cfg), 0))
        return fsm

    def command(self, fsm, pose=(0, 0, 0), dt=.1):
        self.stamp += dt
        return fsm.command(scan_features(cross_scan(pose), self.cfg), self.stamp)

    def test_stalled_center_never_transitions_on_elapsed_frames(self):
        for state in ("CENTER", "RETURN_CENTER"):
            self.stamp = 0
            fsm = self.start((-.3, 0, 0), state)
            for _ in range(150):
                self.assertIsNone(self.command(fsm, (-.3, 0, 0)))
            self.assertEqual(fsm.state, state)
            self.assertFalse(fsm.returned)

    def test_centre_arrival_and_fresh_scan_confirmation(self):
        fsm = self.start((-.3, 0, 0))
        for x in (-.2, -.1, -.04):
            self.command(fsm, (x, 0, 0))
            self.assertEqual(fsm.state, "CENTER")
        # Re-reading the same frame cannot finish the confirmation window.
        features = scan_features(cross_scan((-.04, 0, 0)), self.cfg)
        for _ in range(50):
            fsm.command(features, self.stamp)
        self.assertEqual(fsm.state, "CENTER")
        self.command(fsm, (-.04, 0, 0))
        self.command(fsm, (-.04, 0, 0))
        self.assertEqual(fsm.state, "TURN")

    def test_unchanged_scan_cannot_complete_turn_even_if_commanded(self):
        fsm = self.start()
        fsm._start_turn(+1, "ENTRY", 0)
        for _ in range(150):
            cmd = self.command(fsm)
            self.assertEqual(cmd[0], 0)
            self.assertGreater(cmd[1], 0)
        self.assertEqual(fsm.state, "TURN")
        self.assertAlmostEqual(fsm.info()["turn_error_rad"], math.pi/2, places=3)

    def test_both_turn_directions_close_loop_despite_slip_and_variable_scan_rate(self):
        for direction in (-1, 1):
            for gain in (.4, 1.0):
                with self.subTest(direction=direction, gain=gain):
                    self.stamp = 0
                    fsm = self.start()
                    fsm._start_turn(direction, "ENTRY", 0)
                    yaw = 0.0
                    for step in range(350):
                        dt = (.06, .11, .15)[step % 3]
                        command = self.command(fsm, (0, 0, yaw), dt)
                        if fsm.state != "TURN":
                            break
                        yaw = wrap_angle(yaw + gain*float(command[1])*dt)
                    self.assertEqual(fsm.state, "ENTRY", fsm.info())
                    self.assertLess(abs(wrap_angle(yaw-direction*math.pi/2)), self.cfg.yaw_tolerance+.01)
                    self.assertGreater(step, 20)

    def test_overshoot_reverses_turn_command(self):
        fsm = self.start()
        fsm._start_turn(+1, "ENTRY", 0)
        for yaw in np.linspace(0, math.pi/2+.12, 20):
            command = self.command(fsm, (0, 0, yaw))
        self.assertEqual(fsm.state, "TURN")
        self.assertLess(command[1], 0)

    def test_missing_geometry_stops_then_fails_without_false_success(self):
        fsm = self.start()
        fsm._start_turn(+1, "ENTRY", 0)
        scan = cross_scan()
        scan.ranges[:] = np.inf
        features = scan_features(scan, self.cfg)
        for i in range(25):
            command = fsm.command(features, (i+1)*.1)
            np.testing.assert_array_equal(command, [0, 0])
        self.assertEqual(fsm.state, "FAILED")
        self.assertEqual(fsm.failure, "junction_geometry_lost")
        self.assertFalse(fsm.info()["success"])

    def test_turn_drift_stops_without_translation_override(self):
        fsm = self.start()
        fsm._start_turn(+1, "ENTRY", 0)
        command = self.command(fsm, (.18, 0, .1))
        np.testing.assert_array_equal(command, [0, 0])
        self.assertEqual(fsm.failure, "junction_turn_drift")

    def test_return_count_requires_measured_centre(self):
        fsm = self.start((-.3, 0, 0), "RETURN_CENTER")
        fsm.side = 0
        fsm.return_reverse = True
        for x in (-.2, -.1, -.02, -.02, -.02):
            self.command(fsm, (x, 0, 0))
        self.assertEqual(fsm.returned, {(1, 0)})
        self.assertEqual((fsm.state, fsm.side), ("ENTRY", 1))
        self.assertTrue(fsm.expect_reverse())

    def test_old_pose_argument_is_ignored_even_during_turn(self):
        fsm = self.start()
        fsm._start_turn(+1, "ENTRY", 0)
        features = scan_features(cross_scan(), self.cfg)
        for i in range(60):
            fsm.command(features, (999, -999, math.pi/2), (i+1)*.1)
        self.assertEqual(fsm.state, "TURN")

    def test_junction_behind_vehicle_is_not_counted_again(self):
        fsm = PatrolController(self.cfg)
        fsm.reset(stamp=0)
        fsm.state, fsm.junction = "MAIN", 2
        scene = StaticLidarScene()
        features = scan_features(scene.scan((.017, .528, math.pi/2)), self.cfg)
        self.assertTrue(features.junction_open(self.cfg))  # Real rail-gap alias.
        for i in range(10):
            fsm.command(features, (i+1)*.1)
        self.assertEqual((fsm.state, fsm.junction), ("MAIN", 2))


if __name__ == "__main__":
    unittest.main()
