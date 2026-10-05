"""Static-world LiDAR + ideal kinematics, not a Gazebo physics validation."""

import math
from pathlib import Path
import sys
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_patrol_controller import TraditionalPatrolController, incoming_corridor
from navigation import (collision_detected, command_clearance_masks, load_config,
                        safe_command, scan_features, wrap_angle)
from lidar_scene import StaticLidarScene
from junction_localization import JunctionGeometry, JunctionTracker


class LidarPatrolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = StaticLidarScene()
        cls.cfg = load_config()

    def features(self, pose):
        return scan_features(self.scene.scan(pose), self.cfg)

    def run_route(self, initial, junction_limit=None, *, control_cfg=None):
        controller = TraditionalPatrolController(control_cfg or self.cfg)
        pose = np.array(initial, dtype=float)
        transitions = []
        for step in range(12000):
            features = self.features(pose)
            self.assertFalse(collision_detected(features, self.cfg), repr(pose))
            v, w = controller.command(features, step / 12)
            if not transitions or transitions[-1] != controller.state:
                transitions.append(controller.state)
            self.assertNotEqual(controller.state, "FAILED",
                                (controller.failure, pose, controller.diagnostics))
            if controller.state == "DONE":
                break
            if junction_limit and controller.junctions_done >= junction_limit:
                break
            if controller.state in ("TURN_LEFT", "TURN_MAIN"):
                self.assertEqual(v, 0)
            if controller.state in ("REVERSE_LEFT", "CROSS_REVERSE", "RIGHT_OUTBOUND"):
                self.assertLessEqual(v, 0)
            # Pose is only part of this test's plant; never supplied to control.
            pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
        else:
            self.fail(("route did not finish", controller.state, pose))
        return controller, pose, transitions

    def test_full_three_junction_six_endpoint_route(self):
        controller, pose, transitions = self.run_route(self.cfg.start)
        self.assertEqual(controller.state, "DONE")
        self.assertEqual(controller.junctions_done, 3)
        self.assertEqual(controller.endpoints_reached, 6)
        cycle = ["MAIN", "CENTER", "TURN_LEFT", "ENTRY_LEFT", "LEFT_OUTBOUND",
                 "REVERSE_LEFT", "CROSS_REVERSE", "RIGHT_OUTBOUND", "RETURN_RIGHT",
                 "RETURN_CENTER", "TURN_MAIN", "EXIT_MAIN"]
        self.assertEqual(transitions, cycle * 3 + ["FINAL_MAIN", "DONE"])
        self.assertGreater(pose[1], 10.5)
        self.assertLess(abs(pose[0]), .08)
        self.assertLess(abs(wrap_angle(pose[2] - math.pi / 2)), .08)
        np.testing.assert_array_equal(controller.command(self.features(pose), 2000), [0, 0])

    def test_offset_start_completes_one_full_crossing(self):
        controller, pose, _ = self.run_route((.08, -8., math.pi / 2 + .08), 1)
        self.assertEqual(controller.junctions_done, 1)
        self.assertEqual(controller.endpoints_reached, 2)
        self.assertLess(abs(pose[0]), .08)

    def test_full_route_with_node_watchdog_braking_horizon(self):
        controller, pose, _ = self.run_route(self.cfg.start, control_cfg=replace(self.cfg, control_dt=.5))
        self.assertEqual(controller.state, "DONE")
        self.assertEqual((controller.junctions_done, controller.endpoints_reached), (3, 6))
        self.assertGreater(pose[1], 10.5)

    def test_duplicate_scans_cannot_confirm_end(self):
        controller = TraditionalPatrolController(self.cfg)
        controller.state = "FINAL_MAIN"
        controller.junctions_done = self.cfg.junctions
        controller.endpoints_reached = 2 * self.cfg.junctions
        features = self.features((0, 11.12, math.pi / 2))
        controller.command(features, 1.)
        for _ in range(10):
            np.testing.assert_array_equal(controller.command(features, 1.), [0, 0])
        self.assertEqual(controller.state, "FINAL_MAIN")
        controller.command(features, 1.1)
        controller.command(features, 1.2)
        self.assertEqual(controller.state, "DONE")

    def test_completed_branches_ignore_false_junction_until_real_main_end(self):
        controller = TraditionalPatrolController(self.cfg)
        controller.junctions_done = self.cfg.junctions
        controller.endpoints_reached = 2 * self.cfg.junctions
        # A stale candidate must also be cleared on entering the final leg.
        controller.tracker = object()
        controller.confirm = self.cfg.confirm_frames - 1
        features = self.features((0., 8., math.pi / 2))
        np.testing.assert_array_equal(controller.command(features, 0.), [0, 0])
        self.assertEqual(controller.state, "FINAL_MAIN")
        self.assertIsNone(controller.tracker)
        false_junction = TraditionalPatrolController(self.cfg)._geometry(
            self.features((0., 5.60, math.pi / 2)))
        self.assertIsNotNone(false_junction)
        with patch.object(controller, "_geometry", return_value=false_junction) as detector:
            for i, y in enumerate(np.linspace(8., 11.12, 81), 1):
                command = controller.command(self.features((0., y, math.pi / 2)), i / 12)
                self.assertEqual(controller.state, "FINAL_MAIN")
                if y < 10.8:
                    self.assertGreater(command[0], 0)
            for i in (82, 83):
                np.testing.assert_array_equal(controller.command(
                    self.features((0., 11.12, math.pi / 2)), i / 12), [0, 0])
            detector.assert_not_called()
        self.assertEqual(controller.state, "DONE")
        self.assertEqual(controller.junctions_done, 3)
        self.assertEqual(controller.endpoints_reached, 6)

    def test_main_end_before_branch_completion_is_not_success(self):
        for completed, endpoints in ((0, 0), (2, 4), (3, 4)):
            with self.subTest(completed=completed, endpoints=endpoints):
                controller = TraditionalPatrolController(self.cfg)
                controller.junctions_done = completed
                controller.endpoints_reached = endpoints
                features = self.features((0., 11.12, math.pi / 2))
                for i in range(5):
                    np.testing.assert_array_equal(controller.command(features, i / 12), [0, 0])
                self.assertEqual(controller.failure, "main_end_before_patrol_complete")

    def test_gap_and_reversed_clock_latch_failure(self):
        features = self.features(self.cfg.start)
        for stamp in (.5, 2.):
            with self.subTest(stamp=stamp):
                controller = TraditionalPatrolController(self.cfg)
                controller.command(features, 1.)
                np.testing.assert_array_equal(controller.command(features, stamp), [0, 0])
                self.assertEqual(controller.state, "FAILED")
                np.testing.assert_array_equal(controller.command(features, 2.1), [0, 0])

    def test_near_obstacle_is_failure_not_route_completion(self):
        controller = TraditionalPatrolController(self.cfg)
        scan = self.scene.scan(self.cfg.start)
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        scan.ranges[np.abs(angles) < .05] = .20
        command = controller.command(scan_features(scan, self.cfg), 0.)
        np.testing.assert_array_equal(command, [0, 0])
        self.assertEqual(controller.failure, "obstacle_clearance")

    def test_clearance_diagnostics_identify_guard_and_triggering_ray(self):
        from types import SimpleNamespace
        cases = [([.2, 0.], [.20], [0.], "footprint"),
                 ([.2, 0.], [.39, .35], [0., math.pi], "travel"),
                 ([0., .3], [.32], [math.pi], "rotation"),
                 ([-.2, 0.], [.39], [math.pi], "travel")]
        for action, ranges, angles, guard in cases:
            with self.subTest(guard=guard, action=action):
                controller = TraditionalPatrolController(self.cfg)
                controller.state = "ENTRY_LEFT"
                features = SimpleNamespace(ranges=np.array(ranges), angles=np.array(angles))
                np.testing.assert_array_equal(controller._safe(action, features), [0, 0])
                details = (controller.failure_diagnostics if guard == "footprint"
                           else controller.obstacle["details"])
                hit = next(item for item in details["clearance_hits"] if item["guard"] == guard)
                self.assertEqual(hit["ray_index"], 0)
                self.assertEqual(hit["range_m"], ranges[0])
                if guard == "footprint":
                    controller.fail("scan_receive_timeout")
                    self.assertEqual(controller.failure, "obstacle_clearance")
                    self.assertEqual(controller.failure_diagnostics, details)
                else:
                    self.assertIsNone(controller.failure)
                    self.assertEqual(controller.state, "ENTRY_LEFT")

    def test_moving_wall_correction_uses_arc_sweep_in_both_directions(self):
        from types import SimpleNamespace
        # Reproduce the reported 29.1 cm left wall as a physical wall, not a
        # lone point. The corrective signs steer the swept body away from it.
        x = np.linspace(-1., 1., 401)
        y = np.full_like(x, .29116)
        features = SimpleNamespace(ranges=np.hypot(x, y), angles=np.arctan2(y, x))
        cases = (([.2, -.189916], [.2, .189916]),
                 ([-.2, .189916], [-.2, -.189916]))
        for away, toward in cases:
            with self.subTest(away=away):
                output, blocked = safe_command(away, features, replace(self.cfg, control_dt=.5),
                                               allow_reverse=True)
                np.testing.assert_allclose(output, away)
                self.assertFalse(blocked)
            with self.subTest(toward=toward):
                masks, _ = command_clearance_masks(toward, features,
                                                   replace(self.cfg, control_dt=.5))
                output, blocked = safe_command(toward, features, replace(self.cfg, control_dt=.5),
                                               allow_reverse=True)
                np.testing.assert_array_equal(output, [0., 0.])
                self.assertTrue(blocked)
                self.assertTrue(np.any(masks["sweep"]))
                self.assertFalse(np.any(masks["rotation"]))

    def test_in_place_turn_retains_full_rotation_clearance(self):
        from types import SimpleNamespace
        x, y = np.array([.01816]), np.array([.29116])
        features = SimpleNamespace(ranges=np.hypot(x, y), angles=np.arctan2(y, x))
        controller = TraditionalPatrolController(replace(self.cfg, control_dt=.5))
        controller.state = "TURN_LEFT"
        np.testing.assert_array_equal(controller._safe([0., -.189916], features), [0., 0.])
        self.assertIn("rotation", controller.obstacle["guards"])
        self.assertNotIn("sweep", controller.obstacle["guards"])

    def test_pen_cross_walls_do_not_finish_main(self):
        controller = TraditionalPatrolController(self.cfg)
        features = self.features((0, -9.9167, math.pi / 2))
        for i in range(4):
            self.assertGreater(controller.command(features, i / 12)[0], 0)
        self.assertEqual(controller.state, "MAIN")

    def test_reverse_centres_in_opposite_steering_direction(self):
        features = self.features((1.5, -6.10, math.pi))
        corridor = incoming_corridor(features, self.cfg)
        self.assertIsNotNone(corridor)
        controller = TraditionalPatrolController(self.cfg)
        forward = controller._corridor_drive(features, corridor, 1)
        reverse = controller._corridor_drive(features, corridor, -1)
        self.assertGreater(forward[1], 0)
        self.assertLess(reverse[1], 0)

    def test_uninformative_scan_stops_without_inventing_heading(self):
        scan = self.scene.scan(self.cfg.start)
        scan.ranges[:] = np.inf
        controller = TraditionalPatrolController(self.cfg)
        np.testing.assert_array_equal(controller.command(scan_features(scan, self.cfg), 0.), [0, 0])

    def turning_controller(self):
        controller = TraditionalPatrolController(self.cfg)
        features = self.features((0, -6.18, math.pi / 2))
        for i in range(7):
            controller.command(features, i / 12)
        self.assertEqual(controller.state, "TURN_LEFT")
        return controller

    def test_geometry_loss_during_turn_stops_then_faults(self):
        controller = self.turning_controller()
        scan = self.scene.scan((0, -6.18, math.pi / 2))
        scan.ranges[:] = np.inf
        features = scan_features(scan, self.cfg)
        for i in range(7, 15):
            np.testing.assert_array_equal(controller.command(features, i / 12), [0, 0])
        self.assertEqual(controller.failure, "junction_geometry_lost")
        self.assertEqual(controller.failure_diagnostics["geometry_source"], "unavailable")
        self.assertGreater(controller.failure_diagnostics["geometry_missing_seconds"],
                           self.cfg.junction_tracking_max_gap)
        self.assertIn("last_centre_x", controller.failure_diagnostics)

    def test_abrupt_axis_change_during_turn_is_rejected(self):
        controller = self.turning_controller()
        features = self.features((0, -6.18, math.pi / 2 + .6))
        np.testing.assert_array_equal(controller.command(features, 7 / 12), [0, 0])
        self.assertEqual(controller.failure, "junction_axis_jump")

    def centre_controller(self, state, pose):
        controller = TraditionalPatrolController(self.cfg)
        geometry = controller._geometry(self.features(pose))
        self.assertIsNotNone(geometry)
        controller.state = state
        controller.tracker = JunctionTracker(geometry, 0., controller.cfg)
        return controller

    def test_pure_lateral_error_repositions_then_centres_both_states(self):
        # Exact LiDAR geometry isolates nonholonomic control from wall fitting.
        # The simulated pose is NEVER given to the controller.
        features = self.features((0, -6.18, math.pi / 2))
        for state in ("CENTER", "RETURN_CENTER"):
            for offset in (-.1, .1):
                with self.subTest(state=state, offset=offset):
                    controller = TraditionalPatrolController(self.cfg)
                    controller.state = state
                    pose = np.array([0., -offset, 0.])
                    def geometry():
                        c, s = math.cos(pose[2]), math.sin(pose[2])
                        return JunctionGeometry(np.array([[c, s], [-s, c]]) @ -pose[:2],
                                                (-pose[2], math.pi / 2 - pose[2]), 1., (1.25, 1.04))
                    controller.tracker = JunctionTracker(geometry(), 0., controller.cfg)
                    reversed_once = False
                    with patch.object(controller, "_geometry", side_effect=lambda _: geometry()):
                        for step in range(1, 1001):
                            v, w = controller.command(features, step / 12)
                            self.assertNotEqual(controller.state, "FAILED", controller.failure_diagnostics)
                            if controller.state != state:
                                break
                            reversed_once |= v < 0
                            pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
                        else:
                            self.fail(("lateral centring stalled", pose, controller.diagnostics))
                    self.assertTrue(reversed_once)
                    self.assertEqual(controller.state, "TURN_LEFT" if state == "CENTER" else "TURN_MAIN")
                    self.assertLessEqual(np.linalg.norm(pose[:2]), controller.cfg.junction_centre_tolerance)
                    self.assertLessEqual(abs(pose[2]), self.cfg.yaw_tolerance)

    def test_lateral_reposition_with_real_wall_fitting(self):
        pose = np.array([.1, -6.18, math.pi / 2])
        controller = self.centre_controller("CENTER", pose)
        for step in range(1, 1001):
            features = self.features(pose)
            self.assertFalse(collision_detected(features, self.cfg))
            v, w = controller.command(features, step / 12)
            self.assertNotEqual(controller.state, "FAILED", (controller.failure, controller.diagnostics))
            if controller.state == "TURN_LEFT":
                break
            pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
        else:
            self.fail(("real-wall lateral centring stalled", pose, controller.diagnostics))
        self.assertLess(np.linalg.norm(pose[:2] - [0, -6.18]), self.cfg.junction_centre_tolerance + .005)

    def test_lateral_reposition_keeps_rear_obstacle_guard(self):
        controller = self.centre_controller("CENTER", (.1, -6.18, math.pi / 2))
        scan = self.scene.scan((.1, -6.18, math.pi / 2))
        scan.ranges[0] = .35
        with patch.object(controller, "_geometry", return_value=controller.tracker.geometry):
            np.testing.assert_array_equal(controller.command(scan_features(scan, self.cfg), 1 / 12), [0, 0])
        self.assertIsNone(controller.failure)
        self.assertIsNotNone(controller.obstacle)

    def test_reposition_is_bounded_and_does_not_finish_on_elapsed_time(self):
        controller = self.centre_controller("CENTER", (.1, -6.18, math.pi / 2))
        features = self.features((.1, -6.18, math.pi / 2))
        geometry = controller.tracker.geometry
        with patch.object(controller, "_geometry", return_value=geometry):
            for step in range(1, 1101):
                controller.command(features, step / 12)
                if controller.state == "FAILED":
                    break
        self.assertEqual(controller.failure, "state_timeout")
        self.assertEqual(controller.centre_retries, 1)
        controller = self.centre_controller("CENTER", (.1, -6.18, math.pi / 2))
        controller.centre_retries = 2
        for i in range(1, self.cfg.confirm_frames + 1):
            controller.command(features, i / 12)
        self.assertEqual(controller.failure, "centre_reposition_limit")

    def turn_region_controller(self, state="RETURN_CENTER", *, cfg=None, lateral=-.023432282115415204,
                            along=.013363411381133042, heading=0.):
        controller = TraditionalPatrolController(cfg or self.cfg)
        direction = np.array([math.cos(heading), math.sin(heading)])
        normal = np.array([-direction[1], direction[0]])
        geometry = JunctionGeometry(along * direction + lateral * normal,
                                    (heading, heading + math.pi / 2), 1., (1.25, 1.04))
        controller.state, controller.centre_retries = state, 2
        controller.tracker = JunctionTracker(geometry, 0., controller.cfg)
        return controller, geometry

    def test_reported_small_residual_aligns_both_states_without_third_retreat(self):
        features = self.features((0., 0., math.pi))
        for state in ("CENTER", "RETURN_CENTER"):
            for side in (-1, 1):
                with self.subTest(state=state, side=side):
                    heading = side * math.radians(6.6449002977181415)
                    controller, initial = self.turn_region_controller(
                        state, heading=heading, lateral=side * .023432282115415204)
                    controller.junctions_done, controller.endpoints_reached = 1, 4
                    yaw = 0.
                    aligned_frames = 0
                    for step in range(1, 121):
                        c, s = math.cos(yaw), math.sin(yaw)
                        geometry = replace(initial, centre=np.array([[c, s], [-s, c]]) @ initial.centre,
                                           axes=(heading - yaw, heading + math.pi / 2 - yaw))
                        with patch.object(controller, "_geometry", return_value=geometry):
                            v, w = controller.command(features, step / 12)
                        self.assertNotEqual(controller.state, "FAILED", controller.failure_diagnostics)
                        self.assertEqual(v, 0.)
                        if abs(heading - yaw) <= self.cfg.yaw_tolerance:
                            aligned_frames += 1
                        else:
                            aligned_frames = 0
                        if controller.state != state:
                            break
                        self.assertEqual(controller.centre_retries, 2)
                        yaw += w / 12
                    else:
                        self.fail("small residual did not reach turn-ready state")
                    self.assertGreaterEqual(aligned_frames, self.cfg.confirm_frames)
                    self.assertEqual(controller.state, "TURN_LEFT" if state == "CENTER" else "TURN_MAIN")
                    self.assertEqual((controller.junctions_done, controller.endpoints_reached), (1, 4))

    def test_five_cm_region_is_available_without_any_retries(self):
        controller, geometry = self.turn_region_controller(along=0., lateral=.045)
        controller.centre_retries = 0
        with patch.object(controller, "_geometry", return_value=geometry):
            for i in range(1, 4):
                v, _ = controller.command(self.features((0., 0., math.pi)), i / 12)
                self.assertEqual(v, 0)
                self.assertEqual(controller.centre_retries, 0)
                self.assertFalse(controller.centre_retreat)
                self.assertEqual(controller.state, "TURN_MAIN" if i == 3 else "RETURN_CENTER")
        self.assertEqual(controller.cfg.junction_centre_tolerance, .05)

    def test_turn_region_respects_stricter_configured_tolerance(self):
        controller, geometry = self.turn_region_controller(cfg=replace(self.cfg, junction_centre_tolerance=.02))
        features = self.features((0., 0., math.pi))
        with patch.object(controller, "_geometry", return_value=geometry):
            for i in range(1, self.cfg.confirm_frames + 1):
                np.testing.assert_array_equal(controller.command(features, i / 12), [0., 0.])
        self.assertEqual(controller.cfg.junction_centre_tolerance, .02)
        self.assertEqual(controller.failure, "centre_reposition_limit")

    def test_outside_turn_region_requires_consecutive_failure_confirmation(self):
        controller, initial = self.turn_region_controller(along=0.)
        features = self.features((0., 0., math.pi))
        # A single outlier cannot trigger failure or authorize a turn. A new
        # in-band sample resets the out-of-band run; three bad samples fail.
        for i, lateral in enumerate((.051, .048, .051, .053, .054), 1):
            with patch.object(controller, "_geometry", return_value=replace(initial, centre=np.array([0., lateral]))):
                np.testing.assert_array_equal(controller.command(features, i / 12), [0., 0.])
            self.assertEqual(controller.state, "FAILED" if i == 5 else "RETURN_CENTER")
        details = controller.failure_diagnostics
        self.assertEqual(details["centre_retries"], 2)
        self.assertEqual(details["centre_limit_reason"], "retries_exhausted")
        self.assertAlmostEqual(details["centre_distance_m"], .054)
        self.assertEqual(details["centre_target_tolerance_m"], .05)

    def test_turn_region_needs_three_consecutive_in_band_frames(self):
        controller, initial = self.turn_region_controller(along=0.)
        features = self.features((0., 0., math.pi))
        for i, lateral in enumerate((.048, .051, .048, .047, .046), 1):
            with patch.object(controller, "_geometry", return_value=replace(initial, centre=np.array([0., lateral]))):
                np.testing.assert_array_equal(controller.command(features, i / 12), [0., 0.])
            self.assertEqual(controller.state, "TURN_MAIN" if i == 5 else "RETURN_CENTER")

    def test_turn_region_still_checks_rotation_clearance(self):
        controller, geometry = self.turn_region_controller(heading=.116)
        scan = self.scene.scan((0., 0., math.pi))
        scan.ranges[0] = .32
        with patch.object(controller, "_geometry", return_value=geometry):
            np.testing.assert_array_equal(controller.command(scan_features(scan, self.cfg), 1 / 12), [0., 0.])
        self.assertIsNone(controller.failure)
        self.assertIn("rotation", controller.obstacle["guards"])

    def test_aligned_pose_requires_rotation_clearance_before_turn_transition(self):
        controller, geometry = self.turn_region_controller(along=0., lateral=.045)
        scan = self.scene.scan((0., 0., math.pi))
        scan.ranges[0] = .32
        features = scan_features(scan, self.cfg)
        with patch.object(controller, "_geometry", return_value=geometry):
            for i in range(1, 4):
                np.testing.assert_array_equal(controller.command(features, i / 12), [0., 0.])
                self.assertNotEqual(controller.state, "TURN_MAIN")
        self.assertIsNone(controller.failure)
        self.assertEqual(controller.obstacle["details"]["clearance_check"], "before_turn")

    def test_turn_region_is_radial_not_independent_five_cm_axis_limits(self):
        controller, geometry = self.turn_region_controller(along=.04, lateral=.04)
        controller.centre_retries = 0
        with patch.object(controller, "_geometry", return_value=geometry):
            for i in range(1, 6):
                controller.command(self.features((0., 0., math.pi)), i / 12)
                self.assertEqual(controller.state, "RETURN_CENTER")
        self.assertGreater(controller.diagnostics["centre_distance_m"], .05)

    def test_offset_turn_region_completes_entry_into_branch_and_main(self):
        for state, yaw, target in (("CENTER", math.pi / 2, "LEFT_OUTBOUND"),
                                   ("RETURN_CENTER", math.pi, "MAIN")):
            for dx, dy in ((.045, 0.), (-.045, 0.), (0., .045), (0., -.045)):
                with self.subTest(state=state, offset=(dx, dy)):
                    pose = np.array([dx, -6.18 + dy, yaw])
                    controller = self.centre_controller(state, pose)
                    controller.cfg = replace(controller.cfg, control_dt=.5)
                    controller.junctions_done, controller.endpoints_reached = 1, 4
                    transitions = [state]
                    for i in range(1, 1001):
                        features = self.features(pose)
                        self.assertFalse(collision_detected(features, self.cfg))
                        v, w = controller.command(features, i / 12)
                        self.assertNotEqual(controller.state, "FAILED", (pose, controller.failure_diagnostics))
                        self.assertEqual(controller.centre_retries, 0)
                        if controller.state != transitions[-1]:
                            transitions.append(controller.state)
                        if controller.state == target:
                            break
                        pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
                    else:
                        self.fail(("offset entry did not complete", pose, controller.diagnostics))
                    self.assertEqual(transitions, [state, "TURN_LEFT", "ENTRY_LEFT", target]
                                     if state == "CENTER" else [state, "TURN_MAIN", "EXIT_MAIN", target])
                    corridor = incoming_corridor(self.features(pose), controller.cfg)
                    self.assertIsNotNone(corridor)
                    self.assertLessEqual(abs(corridor[0]), self.cfg.entry_handoff_yaw_tolerance)
                    self.assertLessEqual(abs(corridor[1] / 2), self.cfg.entry_handoff_wall_offset)
                    self.assertTrue(controller.diagnostics["entry_wall_confirmed"])
                    self.assertGreaterEqual(controller.diagnostics["entry_depth"], self.cfg.entry_distance)
                    self.assertEqual(controller.junctions_done, 1 if state == "CENTER" else 2)

    def test_entry_does_not_complete_without_aligned_measured_corridor(self):
        features = self.features((-.95, -6.18, math.pi))
        for state in ("ENTRY_LEFT", "CROSS_REVERSE", "EXIT_MAIN"):
            incoming = {"ENTRY_LEFT": -math.pi / 2, "CROSS_REVERSE": 0.,
                        "EXIT_MAIN": math.pi / 2}[state]
            centre = np.array([.95 if state == "CROSS_REVERSE" else -.95, 0.])
            geometry = JunctionGeometry(centre, (incoming, incoming + math.pi / 2), 1., (1.25, 1.04))
            for corridor in (None, (.15, 0.), (0., .26)):
                with self.subTest(state=state, corridor=corridor):
                    controller = TraditionalPatrolController(self.cfg)
                    controller.state = state
                    controller.tracker = JunctionTracker(geometry, 0., controller.cfg)
                    # Tracker's incoming axis is already known during entry;
                    # acquisition normally selected it before the 90° turn.
                    controller.tracker.incoming_axis = incoming
                    with patch.object(controller, "_geometry", return_value=geometry), \
                            patch("lidar_patrol_controller.incoming_corridor", return_value=corridor):
                        for i in range(1, 5):
                            controller.command(features, i / 12)
                            self.assertEqual(controller.state, state, controller.failure_diagnostics)
                            self.assertFalse(controller.diagnostics["entry_wall_confirmed"])
                    self.assertEqual(controller.junctions_done, 0)

    def test_safe_corridor_handoff_accepts_reported_entry_residual(self):
        features = self.features((-.95, -6.18, math.pi))
        geometry = JunctionGeometry(np.array([-1.374, -.077]),
                                    (-math.pi / 2, 0.), 1., (1.25, 1.04))
        controller = TraditionalPatrolController(self.cfg)
        controller.state = "ENTRY_LEFT"
        controller.tracker = JunctionTracker(geometry, 0., controller.cfg)
        controller.tracker.incoming_axis = -math.pi / 2
        reported_corridor = (-.0722, -.206)  # 4.14 degrees, 10.3 cm offset
        with patch.object(controller, "_geometry", return_value=geometry), \
                patch("lidar_patrol_controller.incoming_corridor", return_value=reported_corridor):
            for i in range(1, self.cfg.confirm_frames + 1):
                command = controller.command(features, i / 12)
                self.assertEqual(command[0], 0.)
                if i == self.cfg.confirm_frames:
                    self.assertEqual(command[1], 0.)
        self.assertEqual(controller.state, "LEFT_OUTBOUND")
        self.assertTrue(controller.diagnostics["entry_wall_confirmed"])
        self.assertFalse(controller.diagnostics["entry_wall_precise"])

    def test_entry_slows_while_corridor_is_safe_but_not_precise(self):
        features = self.features((-.75, -6.18, math.pi))
        geometry = JunctionGeometry(np.array([-.75, -.08]),
                                    (-math.pi / 2, 0.), 1., (1.25, 1.04))
        controller = TraditionalPatrolController(self.cfg)
        controller.state = "ENTRY_LEFT"
        controller.tracker = JunctionTracker(geometry, 0., controller.cfg)
        controller.tracker.incoming_axis = -math.pi / 2
        with patch.object(controller, "_geometry", return_value=geometry), \
                patch("lidar_patrol_controller.incoming_corridor", return_value=(-.07, -.16)):
            v, _ = controller.command(features, 1 / 12)
        self.assertAlmostEqual(v, .07)
        self.assertEqual(controller.state, "ENTRY_LEFT")

    def test_spurious_full_detection_does_not_preempt_current_walls(self):
        pose = (0., 5.60, math.pi / 2)
        controller = self.centre_controller("CENTER", pose)
        controller.junctions_done, controller.endpoints_reached = 2, 4
        reference = controller.tracker.geometry
        wrong = replace(reference, centre=reference.centre + [.5, 0.])
        with patch("lidar_patrol_controller.junction_geometry", return_value=wrong):
            for i in range(1, 9):
                v, _ = controller.command(self.features(pose), i / 12)
                self.assertGreater(v, 0)
                self.assertEqual(controller.state, "CENTER")
                self.assertEqual(controller.diagnostics["geometry_source"],
                                 "tracked_walls_after_rejection")
                rejected = controller.diagnostics["rejected_geometry"]
                self.assertEqual(rejected["reason"], "junction_position_jump")
                self.assertGreater(rejected["translation_delta_m"], .25)
                self.assertLess(np.linalg.norm(controller.tracker.geometry.centre - reference.centre), .025)
        self.assertEqual(controller.junctions_done, 2)
        self.assertIsNone(controller.failure)

    def test_actual_position_jump_stops_then_relocalizes(self):
        controller = self.centre_controller("CENTER", (0., 5.60, math.pi / 2))
        reference = controller.tracker.geometry
        moved_features = self.features((0., 6.10, math.pi / 2))
        command = controller.command(moved_features, 1 / 12)
        np.testing.assert_array_equal(command, [0, 0])
        self.assertIsNone(controller.failure)
        self.assertEqual(controller.state, "CENTER")
        self.assertEqual(controller.relocalization["reason"], "junction_position_jump")
        rejected = controller.relocalization["diagnostics"]["rejected_geometry"]
        self.assertGreater(rejected["translation_delta_m"], .25)
        np.testing.assert_array_equal(rejected["previous_centre"], reference.centre)
        self.assertIs(controller.tracker.geometry, reference)
        for index in range(1, self.cfg.confirm_frames + 1):
            np.testing.assert_array_equal(
                controller.command(moved_features, (index + 1) / 12), [0, 0])
        self.assertIsNone(controller.relocalization)
        self.assertIsNone(controller.failure)
        self.assertEqual(controller.state, "CENTER")
        self.assertEqual(controller.diagnostics["relocalization_status"], "recovered")

    def test_bad_detection_without_visible_walls_cannot_use_cached_centre(self):
        pose = (0., 5.60, math.pi / 2)
        controller = self.centre_controller("CENTER", pose)
        wrong = replace(controller.tracker.geometry, centre=np.array([1.1, 0.]))
        features = self.features(pose)
        features.ranges[:] = 10.
        with patch("lidar_patrol_controller.junction_geometry", return_value=wrong):
            np.testing.assert_array_equal(controller.command(features, 1 / 12), [0, 0])
        self.assertEqual(controller.failure, "junction_position_jump")

    def test_small_overshoot_recovers_in_both_centring_states(self):
        cases = [("CENTER", (.012, -6.114, math.pi / 2), "TURN_LEFT"),
                 ("RETURN_CENTER", (-.066, -6.192, math.pi), "TURN_MAIN")]
        for state, initial, target in cases:
            with self.subTest(state=state):
                pose = np.array(initial, dtype=float)
                controller = self.centre_controller(state, pose)
                saw_reverse = False
                for i in range(1, 251):
                    features = self.features(pose)
                    self.assertFalse(collision_detected(features, self.cfg))
                    v, w = controller.command(features, i / 12)
                    self.assertNotEqual(controller.state, "FAILED", controller.failure)
                    self.assertGreaterEqual(v, -.05)
                    saw_reverse |= v < 0
                    if controller.state == target:
                        break
                    pose += np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), w]) / 12
                self.assertTrue(saw_reverse)
                self.assertEqual(controller.state, target)
                self.assertLessEqual(np.linalg.norm(controller.tracker.geometry.centre),
                                     controller.cfg.junction_centre_tolerance)

    def test_overshoot_recovery_still_stops_for_rear_obstacle(self):
        pose = (.012, -6.114, math.pi / 2)
        controller = self.centre_controller("CENTER", pose)
        features = self.features(pose)
        features.ranges[np.abs(features.angles) > math.pi - .02] = .30
        np.testing.assert_array_equal(controller.command(features, 1 / 12), [0, 0])
        self.assertIsNone(controller.failure)
        self.assertLess(controller.obstacle["details"]["requested_v"], 0)

    def test_large_overshoot_remains_a_failure(self):
        pose = (0., -5.99, math.pi / 2)
        controller = self.centre_controller("CENTER", pose)
        np.testing.assert_array_equal(controller.command(self.features(pose), 1 / 12), [0, 0])
        self.assertEqual(controller.failure, "centre_overshoot")

    def test_unmoved_overshoot_cannot_finish_by_elapsed_time(self):
        pose = (.012, -6.114, math.pi / 2)
        controller = self.centre_controller("CENTER", pose)
        features = self.features(pose)
        for i in range(1, 31):
            self.assertLess(controller.command(features, i / 12)[0], 0)
        self.assertEqual(controller.state, "CENTER")


if __name__ == "__main__":
    unittest.main()
