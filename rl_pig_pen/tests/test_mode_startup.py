"""Startup modes without DDS participants or vehicle output."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation import load_config
from lidar_patrol_controller import TraditionalPatrolController
try:
    from control_modes import ModeControlNode, LidarPatrolNode
except ImportError:
    ModeControlNode = None

@unittest.skipIf(ModeControlNode is None, 'ROS required')
class ModeStartupTests(unittest.TestCase):
    def build(self, mode):
        def base_init(n):
            n.cfg = load_config()
            n.controller = TraditionalPatrolController(n.cfg)
        def parameter(n, name, default):
            return SimpleNamespace(value=mode if name == 'initial_mode' else default)
        with patch.object(LidarPatrolNode, '__init__', base_init), \
             patch.object(ModeControlNode, 'declare_parameter', parameter), \
             patch.object(ModeControlNode, 'create_publisher', return_value=Mock()), \
             patch.object(ModeControlNode, 'create_subscription', return_value=Mock()), \
             patch.object(ModeControlNode, 'create_service', return_value=Mock()), \
             patch.object(ModeControlNode, 'publish_state'), \
             patch.object(ModeControlNode, 'get_logger', return_value=Mock()):
            return ModeControlNode()

    def test_auto_arms_but_requires_localization_before_moving(self):
        n = self.build('AUTO')
        self.assertEqual(n.mode, 'AUTO')
        self.assertTrue(n.auto_started)
        self.assertTrue(n.controller.external_paused)
        self.assertEqual(n.commanded, (0., 0.))
        self.assertFalse(n.needs_restart)
        self.assertEqual(n.controller.junctions_done, 0)

    def test_stop_default_and_invalid_manual_startup(self):
        n = self.build('STOP')
        self.assertEqual(n.mode, 'STOP')
        self.assertTrue(n.controller.external_paused)
        with self.assertRaises(ValueError):
            self.build('MANUAL')

    def test_manual_obstacle_stops_before_logging_original_command(self):
        import json
        n = self.build('STOP')
        n.mode = 'MANUAL'
        n.manual_received = n.manual_since = 100.
        n.manual_command = (.05, 0.)
        n.features = object()
        n.pub = Mock()
        n.pig_stop_gate = Mock()
        n.pig_stop_gate.reason.return_value = None
        logger = Mock()
        details = dict(requested_v=.05, requested_w=0., clearance_hits=[
            dict(guard='travel', angle_deg=-28., range_m=.4)])
        def describe(requested, features):
            self.assertEqual(n.mode, 'STOP')
            self.assertEqual(n.commanded, (0., 0.))
            self.assertEqual(requested, (.05, 0.))
            return details
        with patch('control_modes.time.monotonic', return_value=100.), \
             patch.object(n, '_fresh', return_value=True), \
             patch('control_modes.safe_command', return_value=((0., 0.), True)), \
             patch.object(n, 'publish_state'), \
             patch.object(n, 'get_logger', return_value=logger), \
             patch.object(n.controller, '_clearance_details', side_effect=describe):
            n._manual_output()
        self.assertEqual(n.mode_reason, 'MANUAL_OBSTACLE')
        payload = logger.warning.call_args.args[0].split('：', 1)[1]
        self.assertEqual(json.loads(payload), details)
