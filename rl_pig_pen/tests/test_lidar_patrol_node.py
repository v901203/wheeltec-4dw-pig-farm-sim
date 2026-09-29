"""Check ROS adapter stop behaviour without creating ROS participants."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from fsm_lidar_patrol import LidarPatrolNode
except ImportError:
    LidarPatrolNode = None

from lidar_patrol_controller import TraditionalPatrolController
from navigation import load_config
from lidar_scene import StaticLidarScene


@unittest.skipIf(LidarPatrolNode is None, "Source ROS 2 to test the ROS adapter")
class PatrolNodeTests(unittest.TestCase):
    def setUp(self):
        # Bypass Node.__init__: no DDS participant, serial port or robot output.
        self.node = object.__new__(LidarPatrolNode)
        self.node.cfg = load_config()
        self.node.controller = TraditionalPatrolController(self.node.cfg)
        self.node.pub = Mock()
        self.node.last_received = None
        self.node.reported = None
        self.node.init_scan_health()
        self.node.get_logger = Mock(return_value=Mock())

    def assert_stopped(self):
        command = self.node.pub.publish.call_args.args[0]
        self.assertEqual(command.linear.x, 0)
        self.assertEqual(command.angular.z, 0)

    def test_startup_and_receive_timeout_stop(self):
        self.node.check_scan()
        self.assert_stopped()
        self.node.last_received = 1.
        with patch("fsm_lidar_patrol.time.monotonic", return_value=2.):
            self.node.check_scan()
        self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        with patch("fsm_lidar_patrol.time.monotonic", return_value=4.2):
            self.node.check_scan()
        self.assertEqual(self.node.controller.failure, "scan_receive_timeout")

    def test_invalid_scan_pauses_then_latches_if_persistent(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
        scan.ranges[:] = float("nan")
        with patch("fsm_lidar_patrol.time.monotonic", return_value=1.):
            self.node.on_scan(scan)
        self.assert_stopped()
        self.assertEqual(self.node.controller.state, "MAIN")
        for i in range(1, 33):
            self.deliver(scan, 1. + i / 10)
        self.assertEqual(self.node.controller.state, "FAILED")

    def deliver(self, scan, stamp, *, wall_time=None):
        scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=round(stamp * 1e9)))
        with patch("fsm_lidar_patrol.time.monotonic", return_value=stamp if wall_time is None else wall_time):
            self.node.on_scan(scan)

    def test_single_nan_recovers_only_after_three_fresh_good_scans(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        self.assertGreater(self.node.last_command[0], 0)
        original = scan.ranges[820]
        scan.ranges[820] = float("nan")
        self.deliver(scan, 1.08)
        self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        scan.ranges[820] = original
        for stamp in (1.16, 1.24):
            self.deliver(scan, stamp)
            self.assert_stopped()
        self.deliver(scan, 1.32)
        self.assertGreater(self.node.last_command[0], 0)
        self.assertIsNone(self.node.scan_hold_since)

    def test_valid_obstacle_with_nan_stops_without_authorizing_retreat(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        scan.ranges[820] = .39  # inside the previous command's braking envelope
        scan.ranges[900] = float("nan")
        self.deliver(scan, 1.08)
        self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        self.assertIsNotNone(self.node.controller.obstacle)
        self.assertIsNotNone(self.node.scan_hold_since)

    def test_recovery_does_not_bypass_lost_geometry_or_timestamp_gap(self):
        import numpy as np
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        scan.ranges[820] = float("nan")
        self.deliver(scan, 1.08)
        scan.ranges[:] = np.inf
        for stamp in (1.16, 1.24, 1.32):
            self.deliver(scan, stamp)
            self.assert_stopped()
        self.assertEqual(self.node.recovery_frames, 0)
        scan = StaticLidarScene().scan(self.node.cfg.start)
        # Short wall-time pause cannot override a large sensor-time jump.
        scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0))
        with patch("fsm_lidar_patrol.time.monotonic", return_value=1.4):
            self.node.on_scan(scan)
        self.assertEqual(self.node.controller.failure, "scan_gap")

    def test_receive_gap_pauses_before_hard_timeout(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        with patch("fsm_lidar_patrol.time.monotonic", return_value=1.2):
            self.node.check_scan()
        self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        for stamp in (1.24, 1.32, 1.40):
            self.deliver(scan, stamp)
        self.assertGreater(self.node.last_command[0], 0)

    def test_duplicate_frames_cannot_recover_and_eventually_fail(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        for now in [1.08 + i / 10 for i in range(32)]:
            with patch("fsm_lidar_patrol.time.monotonic", return_value=now):
                self.node.on_scan(scan)
            self.assert_stopped()
        self.assertEqual(self.node.controller.state, "FAILED")

    def tick(self, now):
        with patch("fsm_lidar_patrol.time.monotonic", return_value=now):
            self.node.check_scan()

    def test_slow_simulation_and_jitter_do_not_erase_recovery_progress(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1., wall_time=0.)
        previous = 0.
        for index, now in enumerate((.21, .45, .73), 1):
            # Repeated timer observations of a receive gap are not bad frames.
            self.tick(previous + .18)
            self.tick(previous + .19)
            self.assert_stopped()
            self.assertEqual(self.node.recovery_frames, index - 1)
            self.deliver(scan, 1. + index / 12, wall_time=now)
            if index < 3:
                self.assert_stopped()
                self.assertEqual(self.node.recovery_frames, index)
                self.assertEqual(self.node.controller.junctions_done, 0)
            previous = now
        self.assertIsNone(self.node.controller.failure)
        self.assertIsNone(self.node.scan_hold_since)
        self.assertGreater(self.node.last_command[0], 0)

    def test_low_real_scan_rate_validates_each_gap_not_total_pause(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        self.tick(1.18)
        for index, now in enumerate((1.21, 1.42, 1.63), 1):
            self.deliver(scan, now)
            self.assertIsNone(self.node.controller.failure)
            if index < 3:
                self.assert_stopped()
                self.assertAlmostEqual(self.node.controller.last_stamp, now)
        self.assertGreater(self.node.last_command[0], 0)
        # The learned receive budget handles normal 0.21 s intervals, but
        # never becomes the much longer stopped-recovery deadline.
        self.tick(1.99)
        self.assert_stopped()
        self.assertEqual(self.node.recovery_frames, 0)
        self.deliver(scan, 2.02)
        self.assert_stopped()
        self.assertEqual(self.node.recovery_frames, 1)

    def test_true_receive_loss_stops_immediately_but_fails_after_recovery_deadline(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        self.tick(1.2)
        self.tick(1.51)
        self.assertIsNone(self.node.controller.failure)
        self.assert_stopped()
        self.tick(4.2)
        self.assertEqual(self.node.controller.failure, "scan_receive_timeout")
        self.assert_stopped()
        details = self.node.controller.failure_diagnostics
        self.assertGreater(details["scan_receive_age_s"], 3.)
        self.assertEqual(details["recovery_frames"], 0)
        self.deliver(scan, 1.6)
        self.assert_stopped()

    def test_late_packet_uses_same_three_frame_recovery_gate(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        self.deliver(scan, 1.08, wall_time=1.6)
        self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        self.assertEqual(self.node.recovery_frames, 1)
        self.deliver(scan, 1.16, wall_time=1.82)
        self.assert_stopped()
        self.deliver(scan, 1.24, wall_time=2.04)
        self.assertGreater(self.node.last_command[0], 0)
        self.assertIsNone(self.node.scan_hold_since)

    def test_reported_point_five_nine_second_gap_recovers(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.node.receive_pause = .37123042087070646
        self.deliver(scan, 1., wall_time=1.)
        self.tick(1.40)
        self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        self.deliver(scan, 1.08, wall_time=1.589)
        self.assertEqual(self.node.recovery_frames, 1)
        self.deliver(scan, 1.16, wall_time=1.81)
        self.assert_stopped()
        self.deliver(scan, 1.24, wall_time=2.03)
        self.assertGreater(self.node.last_command[0], 0)
        self.assertIsNone(self.node.controller.failure)

    def test_packet_after_full_receive_deadline_still_fails(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1., wall_time=1.)
        self.deliver(scan, 1.08, wall_time=4.3)
        self.assert_stopped()
        self.assertEqual(self.node.controller.failure, "scan_receive_timeout")

    def test_bad_scan_resets_confirmation_but_not_overall_deadline(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        self.tick(1.18)
        self.deliver(scan, 1.21)
        self.assertEqual(self.node.recovery_frames, 1)
        original = scan.ranges[820]
        scan.ranges[820] = float("nan")
        self.deliver(scan, 1.29)
        self.assertEqual(self.node.recovery_frames, 0)
        self.assertAlmostEqual(self.node.scan_hold_since, 1. + 1 / 6)
        scan.ranges[820] = original
        for now in (1.37, 1.45):
            self.deliver(scan, now)
            self.assert_stopped()
        self.deliver(scan, 1.53)
        self.assertGreater(self.node.last_command[0], 0)

    def test_turn_recovery_updates_perception_not_route_progress(self):
        import math
        scene = StaticLidarScene()
        scan = scene.scan((0, -6.18, self.node.cfg.start[2]))
        for i in range(7):
            self.deliver(scan, 1 + i / 12)
        self.assertEqual(self.node.controller.state, "TURN_LEFT")
        self.tick(1.69)
        for i in (1, 2):
            self.deliver(scan, 1.5 + i * .21)
            self.assert_stopped()
            self.assertAlmostEqual(self.node.controller.tracker.stamp, 1.5 + i * .21)
            self.assertEqual(self.node.controller.state, "TURN_LEFT")
            self.assertEqual(self.node.controller.confirm, 0)
        # Restoring reception cannot excuse a discontinuous road orientation.
        jumped = scene.scan((0, -6.18, self.node.cfg.start[2] + math.radians(35)))
        self.deliver(jumped, 2.13)
        self.assert_stopped()
        self.assertEqual(self.node.controller.failure, "junction_axis_jump")

    def test_valid_but_unlocalized_scans_cannot_extend_recovery_forever(self):
        import numpy as np
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        scan.ranges[820] = float("nan")
        self.deliver(scan, 1.08)
        scan.ranges[:] = np.inf
        # Sensor time advances slowly: wall-time deadline still applies.
        for i in range(1, 33):
            self.deliver(scan, 1.08 + i * .005, wall_time=1.08 + i / 10)
        self.assert_stopped()
        self.assertTrue(self.node.controller.failure.startswith("scan_recovery_timeout"))
        self.assertEqual(self.node.controller.failure_diagnostics["recovery_frames"], 0)

    def test_slow_stream_runs_continuously_after_initial_confirmation(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        now = 0.
        for i in range(60):
            self.deliver(scan, 1. + i / 12, wall_time=now)
            if i >= 4:
                self.assertIsNone(self.node.scan_hold_since)
                self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)
                self.assertIsNone(self.node.controller.failure)
            interval = (.21, .25, .28, .23)[i % 4]
            # Run the watchdog during normal inter-frame waiting, including
            # the old .17 s threshold, not only at packet arrival times.
            self.tick(now + .10)
            self.tick(now + .19)
            if i >= 4:
                self.assertIsNone(self.node.scan_hold_since)
                self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)
            now += interval
        self.node.get_logger.return_value.warning.assert_called_once()
        self.assertLessEqual(self.node.receive_pause, .4)

    def test_adaptive_budget_is_capped_and_true_loss_still_stops(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        for i in range(5):
            self.deliver(scan, 1. + i / 12, wall_time=i * .39)
        self.assertEqual(self.node.receive_pause, .4)
        self.tick(1.56 + .41)
        self.assert_stopped()
        self.tick(1.56 + .51)
        self.assertIsNone(self.node.controller.failure)
        self.tick(1.56 + .4 + self.node.cfg.sensor_timeout + .01)
        self.assertEqual(self.node.controller.failure, "scan_receive_timeout")
        self.assertEqual(self.node.controller.failure_diagnostics["scan_pause_threshold_s"], .4)

    def test_invalid_or_duplicate_scans_cannot_train_receive_budget(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        self.deliver(scan, 1., wall_time=1.3)
        scan.ranges[820] = float("nan")
        self.deliver(scan, 1.1, wall_time=1.6)
        self.assertEqual(len(self.node.receive_intervals), 0)
        self.assertAlmostEqual(self.node.receive_pause, 1 / 6)
        self.assert_stopped()

    def test_braking_guard_covers_longer_watchdog_reaction_forward_and_reverse(self):
        import numpy as np
        from types import SimpleNamespace
        self.assertAlmostEqual(self.node.controller.cfg.control_dt, .5)
        for sign in (1, -1):
            controller = TraditionalPatrolController(self.node.controller.cfg)
            features = SimpleNamespace(ranges=np.array([.46]),
                                       angles=np.array([0. if sign > 0 else np.pi]))
            np.testing.assert_array_equal(controller._safe([sign * .2, 0.], features), [0., 0.])
            self.assertIsNone(controller.failure)
            self.assertGreater(controller.obstacle["details"]["travel_limit_m"], .46)

    def test_nan_during_turn_recovers_without_advancing_counters(self):
        scan = StaticLidarScene().scan((0, -6.18, self.node.cfg.start[2]))
        for i in range(7):
            self.deliver(scan, 1 + i / 12)
        self.assertEqual(self.node.controller.state, "TURN_LEFT")
        original = scan.ranges[820]
        scan.ranges[820] = float("nan")
        self.deliver(scan, 1 + 7 / 12)
        scan.ranges[820] = original
        for i in (8, 9):
            self.deliver(scan, 1 + i / 12)
            self.assert_stopped()
        self.deliver(scan, 1 + 10 / 12)
        self.assertGreater(self.node.pub.publish.call_args.args[0].angular.z, 0)
        self.assertEqual(self.node.controller.junctions_done, 0)
        self.assertEqual(self.node.controller.endpoints_reached, 0)

    def test_repeated_timestamp_cannot_repeat_motion(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
        self.node.on_scan(scan)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)
        self.node.on_scan(scan)
        self.assert_stopped()

    def test_failure_log_retains_original_reason_and_raw_hit(self):
        import json
        import numpy as np
        scan = StaticLidarScene().scan(self.node.cfg.start)
        scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        scan.ranges[np.abs(angles) < .05] = .2
        self.node.on_scan(scan)
        self.assert_stopped()
        logger = self.node.get_logger.return_value
        logged = logger.error.call_args.args[0]
        details = json.loads(logged.split("停止診斷：", 1)[1])
        self.assertEqual(details["failed_state"], "MAIN")
        self.assertTrue(all(hit["raw_valid"] for hit in details["clearance_hits"]))
        self.assertTrue(all(hit["raw_range_m"] == .2 for hit in details["clearance_hits"]))
        with patch("fsm_lidar_patrol.time.monotonic", return_value=self.node.last_received + 1):
            self.node.check_scan()
        self.assertEqual(self.node.controller.failure, "obstacle_clearance")
        logger.error.assert_called_once()

    def test_obstacle_wait_and_removal_resume_without_restart(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        original = scan.ranges[820]
        scan.ranges[820] = .39
        for i in range(1, 14):
            self.deliver(scan, 1 + i / 12)
            if i < 3:
                self.assert_stopped()
            else:
                self.assertLessEqual(self.node.last_command[0], 0)
            self.assertIsNone(self.node.controller.failure)
        hold = self.node.controller.obstacle
        self.assertTrue(hold["attempted"])
        self.assertEqual(hold["mode"], "clearance")
        self.assertTrue(all(hit["raw_valid"] for hit in hold["details"]["clearance_hits"]))
        self.assertEqual(hold["details"]["clearance_hits"][0]["raw_range_m"], .39)
        scan.ranges[820] = original
        for i in range(14, 17):
            self.deliver(scan, 1 + i / 12)
            self.assert_stopped()
        self.deliver(scan, 1 + 17 / 12)
        self.assertGreater(self.node.last_command[0], 0)
        self.assertIsNone(self.node.controller.obstacle)
        self.assertEqual(self.node.controller.junctions_done, 0)

    def test_obstacle_does_not_hide_invalid_stream_or_receive_loss(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        scan.ranges[820] = .39
        self.deliver(scan, 1.08)
        self.assertIsNotNone(self.node.controller.obstacle)
        self.tick(1.7)
        self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        self.tick(4.3)
        self.assertEqual(self.node.controller.failure, "scan_receive_timeout")

    def test_nan_and_obstacle_require_both_health_and_clearance_confirmation(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        front, other = scan.ranges[820], scan.ranges[900]
        scan.ranges[820], scan.ranges[900] = .39, float("nan")
        self.deliver(scan, 1 + 1 / 12)
        self.assert_stopped()
        scan.ranges[900] = other
        for i in (2, 3, 4):
            self.deliver(scan, 1 + i / 12)
            self.assert_stopped()
        self.assertIsNone(self.node.scan_hold_since)
        self.assertIsNotNone(self.node.controller.obstacle)
        scan.ranges[820] = front
        for i in (5, 6, 7):
            self.deliver(scan, 1 + i / 12)
            self.assert_stopped()
        self.deliver(scan, 1 + 8 / 12)
        self.assertGreater(self.node.last_command[0], 0)

    def test_duplicate_scan_breaks_obstacle_clear_confirmation(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        self.deliver(scan, 1.)
        front = scan.ranges[820]
        scan.ranges[820] = .39
        self.deliver(scan, 1.08)
        scan.ranges[820] = front
        self.deliver(scan, 1.16)
        self.assertEqual(self.node.controller.obstacle["clear_frames"], 1)
        self.deliver(scan, 1.16, wall_time=1.24)
        self.assert_stopped()
        self.assertEqual(self.node.controller.obstacle["clear_frames"], 0)
        self.assertIsNotNone(self.node.scan_hold_since)

    def test_raw_hazard_during_retreat_stops_and_retains_original_direction(self):
        from junction_localization import JunctionTracker
        from navigation import scan_features
        pose = (0., -6.58, self.node.cfg.start[2])
        scan = StaticLidarScene().scan(pose)
        c = self.node.controller
        g = c._geometry(scan_features(scan, self.node.cfg))
        c.tracker = JunctionTracker(g, 1., c.cfg)
        c.state, c.last_stamp, c.state_stamp = "CENTER", 1., 1.
        scan.ranges[820] = .39
        c.stop_for_obstacle([.12, 0.], scan_features(scan, self.node.cfg))
        for i in range(1, 4):
            self.deliver(scan, 1 + i / 12)
        self.assertLess(self.node.last_command[0], 0)
        scan.ranges[0] = .36
        self.deliver(scan, 1 + 4 / 12)
        self.assert_stopped()
        self.assertEqual(c.obstacle["phase"], "WAIT")
        self.assertEqual(c.obstacle["command"], (.12, 0.))
        self.assertTrue(c.obstacle["attempted"])

    def test_receive_pause_interrupts_escape_without_restarting_it(self):
        from junction_localization import JunctionTracker
        from navigation import scan_features
        pose = (0., -6.58, self.node.cfg.start[2])
        scan = StaticLidarScene().scan(pose)
        c = self.node.controller
        g = c._geometry(scan_features(scan, self.node.cfg))
        c.tracker = JunctionTracker(g, 1., c.cfg)
        c.state, c.last_stamp, c.state_stamp = "CENTER", 1., 1.
        scan.ranges[820] = .39
        c.stop_for_obstacle([.12, 0.], scan_features(scan, self.node.cfg))
        for i in range(1, 4):
            self.deliver(scan, 1 + i / 12)
        self.assertLess(self.node.last_command[0], 0)
        self.tick(1.44)
        self.assert_stopped()
        for i in range(6, 13):
            self.deliver(scan, 1 + i / 12)
            self.assert_stopped()
        self.assertIsNone(self.node.controller.failure)
        self.assertIsNone(self.node.scan_hold_since)
        self.assertTrue(self.node.controller.obstacle["attempted"])


if __name__ == "__main__":
    unittest.main()
