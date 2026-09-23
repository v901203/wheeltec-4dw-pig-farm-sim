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
from navigation import collision_detected, load_config, scan_features, wrap_angle
from lidar_scene import StaticLidarScene
from junction_localization import JunctionTracker


class LidarPatrolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = StaticLidarScene()
        cls.cfg = load_config()

    def features(self, pose):
        return scan_features(self.scene.scan(pose), self.cfg)

    def run_route(self, initial, junction_limit=None):
        controller = TraditionalPatrolController(self.cfg)
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
                details = controller.failure_diagnostics
                self.assertEqual(details["failed_state"], "ENTRY_LEFT")
                hit = next(item for item in details["clearance_hits"] if item["guard"] == guard)
                self.assertEqual(hit["ray_index"], 0)
                self.assertEqual(hit["range_m"], ranges[0])
                controller.fail("scan_receive_timeout")
                self.assertEqual(controller.failure, "obstacle_clearance")
                self.assertEqual(controller.failure_diagnostics, details)

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

    def test_actual_position_jump_remains_stopped_and_records_both_centres(self):
        controller = self.centre_controller("CENTER", (0., 5.60, math.pi / 2))
        reference = controller.tracker.geometry
        command = controller.command(self.features((0., 6.10, math.pi / 2)), 1 / 12)
        np.testing.assert_array_equal(command, [0, 0])
        self.assertEqual(controller.failure, "junction_position_jump")
        rejected = controller.failure_diagnostics["rejected_geometry"]
        self.assertGreater(rejected["translation_delta_m"], .25)
        np.testing.assert_array_equal(rejected["previous_centre"], reference.centre)
        self.assertIs(controller.tracker.geometry, reference)

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
        self.assertEqual(controller.failure, "obstacle_clearance")
        self.assertLess(controller.failure_diagnostics["requested_v"], 0)

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
