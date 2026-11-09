"""Recovery invariants with synthetic LiDAR; no robot commands."""
import unittest
from dataclasses import replace
from unittest.mock import patch
import test_lidar_patrol_node as node_tests
from navigation import load_config, scan_features
from lidar_patrol_controller import TraditionalPatrolController
from lidar_scene import StaticLidarScene

class PersistentControllerTests(unittest.TestCase):
    def test_contact_wait_preserves_route_then_releases(self):
        cfg = replace(load_config(), persistent_recovery=True)
        c = TraditionalPatrolController(cfg)
        c.junctions_done, c.endpoints_reached = 2, 4
        features = scan_features(StaticLidarScene().scan(cfg.start), cfg)
        with patch.object(c, "_clearance_details", return_value={"clearance_hits": [{"guard": "footprint"}]}):
            for _ in range(8):
                self.assertEqual(list(c.stop_for_obstacle([.1, 0], features)), [0, 0])
        self.assertEqual(c.state, "MAIN")
        self.assertEqual(c.obstacle["reason"], "footprint_wait")
        self.assertTrue(c.obstacle["attempted"])
        for i in range(cfg.confirm_frames):
            self.assertEqual(list(c.command(features, i * .1)), [0, 0])
        self.assertIsNone(c.obstacle)
        self.assertEqual((c.junctions_done, c.endpoints_reached), (2, 4))

    def test_long_gap_reacquires_without_resetting_progress(self):
        cfg = replace(load_config(), persistent_recovery=True)
        c = TraditionalPatrolController(cfg)
        f = scan_features(StaticLidarScene().scan(cfg.start), cfg)
        c.command(f, 1.)
        c.junctions_done, c.endpoints_reached = 2, 4
        c.command(f, 100.)
        self.assertEqual(c.last_stamp, 100.)
        self.assertIsNotNone(c.relocalization)
        for i in range(cfg.confirm_frames):
            c.command(f, 100.1 + i * .1)
        self.assertIsNone(c.relocalization)
        self.assertEqual((c.junctions_done, c.endpoints_reached), (2, 4))

@unittest.skipIf(node_tests.LidarPatrolNode is None, "ROS required")
class PersistentNodeTests(unittest.TestCase):
    def test_receive_outage_waits_and_recovers(self):
        harness = node_tests.PatrolNodeTests()
        harness.setUp()
        n = harness.node
        n.cfg = replace(n.cfg, persistent_recovery=True)
        n.controller = TraditionalPatrolController(n.cfg)
        n.controller.junctions_done = 2
        n.controller.endpoints_reached = 4
        scan = StaticLidarScene().scan(n.cfg.start)
        harness.deliver(scan, 1.)
        with patch("fsm_lidar_patrol.time.monotonic", return_value=100.):
            n.check_scan()
        self.assertEqual(n.controller.state, "MAIN")
        self.assertIsNotNone(n.scan_hold_since)
        for i in range(12):
            harness.deliver(scan, 101. + i * .1)
        self.assertIsNone(n.scan_hold_since)
        self.assertEqual((n.controller.junctions_done, n.controller.endpoints_reached), (2, 4))

    def test_invalid_scans_never_authorize_motion(self):
        harness = node_tests.PatrolNodeTests()
        harness.setUp()
        n = harness.node
        n.cfg = replace(n.cfg, persistent_recovery=True)
        n.controller = TraditionalPatrolController(n.cfg)
        scan = StaticLidarScene().scan(n.cfg.start)
        scan.ranges[:] = float("nan")
        for stamp in (1., 10., 100., 1000.):
            harness.deliver(scan, stamp)
            harness.assert_stopped()
        self.assertEqual(n.controller.state, "MAIN")
        self.assertEqual(n.recovery_frames, 0)
        self.assertIsNotNone(n.scan_hold_since)
