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
        self.assertEqual(self.node.controller.failure, "scan_receive_timeout")

    def test_invalid_scan_stops_and_latches(self):
        scan = StaticLidarScene().scan(self.node.cfg.start)
        scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
        scan.ranges[:] = float("nan")
        self.node.on_scan(scan)
        self.assert_stopped()
        self.assertEqual(self.node.controller.state, "FAILED")

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


if __name__ == "__main__":
    unittest.main()
