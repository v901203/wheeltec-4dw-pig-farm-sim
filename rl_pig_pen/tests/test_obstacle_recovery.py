"""Obstacle supervisor tests: synthetic faults, real static walls, no Gazebo."""

import math
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from junction_localization import JunctionTracker
from lidar_patrol_controller import TraditionalPatrolController
from navigation import load_config, scan_features
from lidar_scene import StaticLidarScene


class ObstacleRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = replace(load_config(), control_dt=.5)
        cls.scene = StaticLidarScene()

    def features(self, pose=None, front=None, rear=None):
        scan = self.scene.scan(self.cfg.start if pose is None else pose)
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        if front is not None:
            scan.ranges[np.abs(angles) < .01] = front
        if rear is not None:
            scan.ranges[np.abs(angles) > math.pi - .01] = rear
        return scan_features(scan, self.cfg)

    def held(self, *, sign=1, state="CENTER", action=None):
        pose = (0., -6.58, math.pi / 2)
        controller = TraditionalPatrolController(self.cfg)
        geometry = controller._geometry(self.features(pose))
        self.assertIsNotNone(geometry)
        controller.state = state
        controller.last_stamp = controller.state_stamp = 0.
        controller.tracker = JunctionTracker(geometry, 0., controller.cfg)
        controller.junctions_done, controller.endpoints_reached = 1, 2
        features = self.features(pose, **({"front": .39} if sign > 0 else {"rear": .39}))
        np.testing.assert_array_equal(controller._safe(action or [sign * .12, 0.], features), [0., 0.])
        self.assertIsNone(controller.failure)
        self.assertIsNotNone(controller.obstacle)
        return controller, geometry, pose

    def step(self, controller, geometry, pose, index, *, progress=0., sign=1,
             blocked=True, reverse_blocked=False, yaw=0., lateral=0., allow_retreat=True):
        # Geometry displacement is measured input, never computed by controller
        # from commanded velocity. Fixed obstacle gets farther during retreat.
        current = replace(geometry, centre=geometry.centre + [sign * progress, lateral],
                          axes=tuple(a - yaw for a in geometry.axes))
        ranges = {"front" if sign > 0 else "rear": .39 + progress} if blocked else {}
        if reverse_blocked:
            ranges["rear" if sign > 0 else "front"] = .36
        features = self.features(pose, **ranges)
        with patch.object(controller, "_geometry", return_value=current):
            return controller.command(features, index / 12, allow_retreat=allow_retreat)

    def test_forward_and_reverse_escape_opposite_travel_and_stop_at_measured_distance(self):
        for sign, state in ((1, "CENTER"), (-1, "CROSS_REVERSE")):
            with self.subTest(sign=sign):
                c, g, pose = self.held(sign=sign, state=state)
                for i in (1, 2):
                    np.testing.assert_array_equal(self.step(c, g, pose, i, sign=sign), [0, 0])
                np.testing.assert_allclose(self.step(c, g, pose, 3, sign=sign), [-sign * .05, 0])
                np.testing.assert_allclose(self.step(c, g, pose, 4, progress=.05, sign=sign), [-sign * .05, 0])
                np.testing.assert_array_equal(self.step(c, g, pose, 5, progress=.11, sign=sign), [0, 0])
                self.assertEqual(c.obstacle["reason"], "retreat_distance_reached")
                for i in range(6, 20):
                    np.testing.assert_array_equal(self.step(c, g, pose, i, progress=.11, sign=sign), [0, 0])
                self.assertEqual(c.state, state)
                self.assertEqual((c.junctions_done, c.endpoints_reached), (1, 2))
                self.assertTrue(c.obstacle["attempted"])

    def test_reverse_path_blocked_waits_without_repeated_attempts(self):
        c, g, pose = self.held()
        np.testing.assert_array_equal(self.step(c, g, pose, 1, reverse_blocked=True), [0, 0])
        self.assertEqual(c.obstacle["reason"], "reverse_path_blocked")
        for i in range(2, 8):
            np.testing.assert_array_equal(self.step(c, g, pose, i), [0, 0])
        self.assertTrue(c.obstacle["attempted"])

    def test_new_obstacle_during_escape_stops_immediately(self):
        c, g, pose = self.held()
        for i in range(1, 4):
            self.step(c, g, pose, i)
        self.assertEqual(c.obstacle["phase"], "RETREAT")
        np.testing.assert_array_equal(self.step(c, g, pose, 4, reverse_blocked=True), [0, 0])
        self.assertEqual(c.obstacle["phase"], "WAIT")

    def test_frozen_pose_aborts_escape_not_fake_distance_completion(self):
        c, g, pose = self.held()
        for i in range(1, 65):
            command = self.step(c, g, pose, i)
        np.testing.assert_array_equal(command, [0, 0])
        self.assertEqual(c.obstacle["progress"], 0.)
        self.assertEqual(c.obstacle["reason"], "retreat_watchdog")

    def test_pose_drift_interrupts_escape(self):
        for kwargs in ({"yaw": .09}, {"lateral": .07}, {"progress": -.04}):
            with self.subTest(kwargs=kwargs):
                c, g, pose = self.held()
                for i in range(1, 4):
                    self.step(c, g, pose, i)
                np.testing.assert_array_equal(self.step(c, g, pose, 4, **kwargs), [0, 0])
                self.assertEqual(c.obstacle["reason"], "retreat_pose_uncertain")

    def test_rotation_guard_never_authorizes_translation(self):
        c, g, pose = self.held()
        c.obstacle = None
        c.state = "TURN_LEFT"
        c._safe([0., .3], self.features(pose, rear=.32))
        for i in range(1, 8):
            with patch.object(c, "_geometry", return_value=g):
                np.testing.assert_array_equal(c.command(self.features(pose, rear=.32), i / 12), [0, 0])
        self.assertFalse(c.obstacle["attempted"])
        self.assertEqual(c.state, "TURN_LEFT")

    def test_footprint_contact_still_latches_and_never_auto_resumes(self):
        c, g, pose = self.held()
        with patch.object(c, "_geometry", return_value=g):
            np.testing.assert_array_equal(c.command(self.features(pose, front=.2), 1 / 12), [0, 0])
            np.testing.assert_array_equal(c.command(self.features(pose), 2 / 12), [0, 0])
        self.assertEqual(c.failure, "obstacle_clearance")

    def test_corridor_wait_freezes_route_timeout_and_resumes_only_after_clear_scans(self):
        c = TraditionalPatrolController(replace(self.cfg, state_timeout=.2))
        c.last_stamp = c.state_stamp = 0.
        blocked = self.features(front=.39)
        c._safe([.2, 0.], blocked)
        with patch("lidar_patrol_controller.incoming_corridor", return_value=(0., 0.)):
            for i in range(1, 61):
                command = c.command(blocked, i / 12)
                if i < 3 or i >= 51:
                    np.testing.assert_array_equal(command, [0, 0])
                else:
                    self.assertLessEqual(command[0], 0)
            self.assertIsNone(c.failure)
            self.assertTrue(c.obstacle["attempted"])
            for i in (61, 62, 63):
                np.testing.assert_array_equal(c.command(self.features(), i / 12), [0, 0])
                if i < 63:
                    self.assertIsNotNone(c.obstacle)
            self.assertIsNone(c.obstacle)
            self.assertGreater(c.command(self.features(), 64 / 12)[0], 0)
        self.assertEqual((c.junctions_done, c.endpoints_reached), (0, 0))

    def test_release_hysteresis_and_interrupted_confirmation(self):
        c, g, pose = self.held()
        c.obstacle["attempted"] = True
        # Outside original .4448 m envelope, but inside release margin.
        with patch.object(c, "_geometry", return_value=g):
            for i in range(1, 6):
                np.testing.assert_array_equal(c.command(self.features(pose, front=.46), i / 12), [0, 0])
                self.assertEqual(c.obstacle["clear_frames"], 0)
        self.step(c, g, pose, 6, blocked=False)
        self.assertEqual(c.obstacle["clear_frames"], 1)
        self.step(c, g, pose, 7)
        self.assertEqual(c.obstacle["clear_frames"], 0)
        for i in (8, 9):
            self.step(c, g, pose, i, blocked=False)
            self.assertIsNotNone(c.obstacle)
        self.step(c, g, pose, 10, blocked=False)
        self.assertIsNone(c.obstacle)

    def test_sensor_pause_stops_escape_and_cannot_authorize_release(self):
        c, g, pose = self.held()
        for i in range(1, 4):
            self.step(c, g, pose, i)
        for i in (4, 5, 6):
            np.testing.assert_array_equal(self.step(c, g, pose, i, blocked=False, allow_retreat=False), [0, 0])
            self.assertIsNotNone(c.obstacle)
        for i in (7, 8, 9):
            np.testing.assert_array_equal(self.step(c, g, pose, i), [0, 0])
        self.assertTrue(c.obstacle["attempted"])

    def test_duplicate_stamp_cannot_count_toward_obstacle_release(self):
        c, g, pose = self.held()
        self.step(c, g, pose, 1, blocked=False)
        self.assertEqual(c.obstacle["clear_frames"], 1)
        np.testing.assert_array_equal(self.step(c, g, pose, 1, blocked=False), [0, 0])
        self.assertEqual(c.obstacle["clear_frames"], 0)
        for i in (2, 3):
            self.step(c, g, pose, i, blocked=False)
            self.assertIsNotNone(c.obstacle)

    def test_sensor_and_geometry_faults_are_not_masked_by_obstacle_wait(self):
        for fault in ("gap", "jump", "missing"):
            with self.subTest(fault=fault):
                c, g, pose = self.held()
                if fault == "gap":
                    c.command(self.features(pose), 1.)
                    self.assertEqual(c.failure, "scan_gap")
                elif fault == "jump":
                    with patch.object(c.tracker, "observe_walls", return_value=None):
                        self.step(c, g, pose, 1, progress=.5)
                    self.assertEqual(c.failure, "junction_position_jump")
                else:
                    with patch.object(c, "_geometry", return_value=None):
                        for i in range(1, 9):
                            np.testing.assert_array_equal(c.command(self.features(pose), i / 12), [0, 0])
                    self.assertEqual(c.failure, "junction_geometry_lost")

    def test_real_wall_tracking_retreat_then_removal_resumes_centre(self):
        c, _, initial = self.held()
        pose = np.array(initial, dtype=float)
        saw_retreat = False
        for i in range(1, 101):
            progress = initial[1] - pose[1]
            features = self.features(pose, front=.39 + progress if i < 65 else None)
            v, w = c.command(features, i / 12)
            self.assertIsNone(c.failure, c.failure_diagnostics)
            if i < 65:
                self.assertLessEqual(v, 0)
                saw_retreat |= v < 0
                self.assertLessEqual(progress, .15)
            pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
            if i > 65 and v > 0:
                break
        else:
            self.fail("did not resume original centre approach after obstacle removal")
        self.assertTrue(saw_retreat)
        self.assertEqual(c.state, "CENTER")
        self.assertEqual((c.junctions_done, c.endpoints_reached), (1, 2))

    def test_full_crossing_survives_forward_and_reverse_obstacle_encounters(self):
        c = TraditionalPatrolController(self.cfg)
        pose = np.array([0., -8., math.pi / 2])
        injected, retreated = set(), set()
        obstacle = None
        for i in range(1, 5001):
            state = c.state
            near_centre = (c.tracker is not None and
                           np.linalg.norm(c.tracker.geometry.centre) < .8)
            if (obstacle is None and state in ("CENTER", "CROSS_REVERSE")
                    and state not in injected and near_centre):
                sign = -1 if state == "CROSS_REVERSE" else 1
                point = pose[:2] + sign * .39 * np.array([math.cos(pose[2]), math.sin(pose[2])])
                obstacle = (point, i + 65, sign, state)
                injected.add(state)
            scan = self.scene.scan(pose)
            if obstacle is not None:
                point, until, sign, held_state = obstacle
                if i < until:
                    delta = point - pose[:2]
                    angle = math.atan2(delta[1], delta[0]) - pose[2]
                    angle = (angle + math.pi) % (2 * math.pi) - math.pi
                    index = int(round((angle - scan.angle_min) / scan.angle_increment))
                    scan.ranges[index] = min(scan.ranges[index], np.linalg.norm(delta))
                else:
                    obstacle = None
            before = (c.junctions_done, c.endpoints_reached)
            v, w = c.command(scan_features(scan, self.cfg), i / 12)
            self.assertIsNone(c.failure, (c.failure, c.diagnostics))
            if c.obstacle is not None:
                self.assertEqual((c.junctions_done, c.endpoints_reached), before)
                if c.obstacle["phase"] == "RETREAT":
                    retreated.add(c.state)
            pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
            if c.junctions_done == 1:
                break
        else:
            self.fail(("crossing did not finish after obstacle removal", c.state, c.obstacle))
        self.assertEqual(injected, {"CENTER", "CROSS_REVERSE"})
        self.assertEqual(retreated, injected)
        self.assertEqual(c.endpoints_reached, 2)
        self.assertEqual(c.state, "MAIN")

    def test_tracking_boundary_waits_instead_of_retreating_out_of_view(self):
        c, g, pose = self.held()
        edge = replace(g, centre=np.array([1.40, 0.]))
        c.tracker = JunctionTracker(edge, 0., c.cfg)
        c.obstacle["reference"] = None
        for i in range(1, 6):
            with patch.object(c, "_geometry", return_value=edge):
                np.testing.assert_array_equal(c.command(self.features(pose, front=.39), i / 12), [0, 0])
        self.assertFalse(c.obstacle["attempted"])
        self.assertEqual(c.obstacle["reason"], "retreat_unobservable_or_ineligible")

    def corridor_hold(self, state="MAIN", *, sweep_only=False):
        c = TraditionalPatrolController(self.cfg)
        c.state, c.last_stamp, c.state_stamp = state, 0., 0.
        sign = -1 if state in ("REVERSE_LEFT", "RIGHT_OUTBOUND") else 1
        pose = (0., -9., math.pi / 2)
        f = self.features(pose, **({"front": .39} if sign > 0 else {"rear": .39}))
        command = [sign * .2, 0.]
        if sweep_only:
            f = self.features(pose)
            index = np.argmin(abs(f.angles - math.radians(174.948)))
            f.ranges[index] = .520852
            command = [-.2, .0325789]
        c.stop_for_obstacle(command, f)
        return c, f, pose, sign

    def test_all_straight_states_retreat_opposite_travel_after_three_scans(self):
        for state in TraditionalPatrolController.CORRIDOR_STATES:
            with self.subTest(state=state):
                c, f, _, sign = self.corridor_hold(state)
                for i in (1, 2):
                    np.testing.assert_array_equal(c.command(f, i / 12), [0, 0])
                np.testing.assert_allclose(c.command(f, 3 / 12), [-sign * .05, 0])
                self.assertEqual(c.state, state)
                self.assertEqual((c.junctions_done, c.endpoints_reached), (0, 0))
                self.assertEqual(c.obstacle["mode"], "clearance")

    def test_reported_rear_sweep_alone_can_retreat_forward(self):
        c, f, _, _ = self.corridor_hold("RIGHT_OUTBOUND", sweep_only=True)
        self.assertEqual(c.obstacle["guards"], {"sweep"})
        for i in (1, 2, 3):
            v, w = c.command(f, i / 12)
        self.assertEqual((v, w), (.05, 0.))

    def test_corridor_escape_stops_on_clearance_but_does_not_resume_into_fixed_obstacle(self):
        c, f, pose, _ = self.corridor_hold()
        for i in (1, 2, 3):
            c.command(f, i / 12)
        farther = self.features(pose, front=.56)
        np.testing.assert_array_equal(c.command(farther, 4 / 12), [0, 0])
        self.assertEqual(c.obstacle["reason"], "retreat_clearance_restored")
        for i in range(5, 14):
            np.testing.assert_array_equal(c.command(farther, i / 12), [0, 0])
            self.assertIsNotNone(c.obstacle)
        for i in (14, 15, 16):
            np.testing.assert_array_equal(c.command(self.features(pose), i / 12), [0, 0])
        self.assertIsNone(c.obstacle)
        self.assertGreater(c.command(self.features(pose), 17 / 12)[0], 0)

    def test_corridor_escape_stops_if_reverse_path_blocks_or_sensor_pauses(self):
        for fault in ("obstacle", "pause", "footprint"):
            with self.subTest(fault=fault):
                c, f, pose, _ = self.corridor_hold()
                for i in (1, 2, 3):
                    c.command(f, i / 12)
                unsafe = self.features(pose, front=.39, rear=.20 if fault == "footprint" else .36)
                result = c.command(f if fault == "pause" else unsafe, 4 / 12,
                                   allow_retreat=fault != "pause")
                np.testing.assert_array_equal(result, [0, 0])
                if fault == "footprint":
                    self.assertEqual(c.failure, "obstacle_clearance")
                else:
                    self.assertEqual(c.obstacle["phase"], "WAIT")
                    np.testing.assert_array_equal(c.command(f, 5 / 12), [0, 0])

    def test_branch_end_still_confirms_while_obstacle_stopped(self):
        for state, pose, target in (("LEFT_OUTBOUND", (-3.25, -6.18, math.pi), "REVERSE_LEFT"),
                                    ("RIGHT_OUTBOUND", (3.25, -6.18, math.pi), "RETURN_RIGHT")):
            with self.subTest(state=state):
                c, _, _, sign = self.corridor_hold(state)
                f = self.features(pose)
                for i in (1, 2):
                    np.testing.assert_array_equal(c.command(f, i / 12), [0, 0])
                    self.assertEqual(c.state, state)
                    self.assertEqual(c.endpoints_reached, 0)
                np.testing.assert_array_equal(c.command(f, 3 / 12), [0, 0])
                self.assertEqual(c.state, target)
                self.assertEqual(c.endpoints_reached, 1)
                self.assertIsNone(c.obstacle)
                v, _ = c.command(f, 4 / 12)
                self.assertLess(v * sign, 0)

    def test_sensor_pause_resets_end_confirmation_and_never_advances_route(self):
        c, _, _, _ = self.corridor_hold("RIGHT_OUTBOUND")
        f = self.features((3.25, -6.18, math.pi))
        c.command(f, 1 / 12)
        self.assertEqual(c.obstacle["end_frames"], 1)
        for i in (2, 3, 4):
            np.testing.assert_array_equal(c.command(f, i / 12, allow_retreat=False), [0, 0])
            self.assertEqual(c.endpoints_reached, 0)
        for i in (5, 6):
            c.command(f, i / 12)
            self.assertEqual(c.state, "RIGHT_OUTBOUND")
        c.command(f, 7 / 12)
        self.assertEqual(c.state, "RETURN_RIGHT")

    def test_approaching_end_wall_slows_before_guard_intervention(self):
        for state in ("LEFT_OUTBOUND", "RIGHT_OUTBOUND"):
            c, _, _, sign = self.corridor_hold(state)
            c.obstacle = None
            pose = (-3.05 if sign > 0 else 3.05, -6.18, math.pi)
            v, _ = c.command(self.features(pose), 1 / 12)
            self.assertAlmostEqual(v, sign * .07)
            self.assertEqual(c.state, state)


if __name__ == "__main__":
    unittest.main()
