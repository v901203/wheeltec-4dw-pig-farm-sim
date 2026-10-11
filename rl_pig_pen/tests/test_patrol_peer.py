"""Peer protocol and stationary holds; no DDS, serial output or robot motion."""

from dataclasses import replace
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from junction_localization import JunctionGeometry, JunctionTracker
from lidar_patrol_controller import TraditionalPatrolController
from navigation import load_config, scan_features
from lidar_scene import StaticLidarScene

try:
    from fsm_lidar_patrol import LidarPatrolNode, PigStopGate, STATE_LABELS
except ImportError:
    LidarPatrolNode = None


@unittest.skipIf(LidarPatrolNode is None, "Source ROS 2 to test the peer adapter")
class PeerGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = PigStopGate(replace(load_config(), vision_required=True))

    def clear(self, start=0.):
        for i in range(17):
            self.gate.receive(False, start + i / 10)

    def test_wait_for_peer_then_continuous_clear(self):
        self.assertEqual(self.gate.reason(0), "WAITING_PEER")
        self.gate.receive(False, 0)
        self.assertEqual(self.gate.reason(.2), "WAITING_CLEAR")
        self.clear()
        self.assertIsNone(self.gate.reason(1.6))

    def test_true_resets_clear_window(self):
        self.clear()
        self.gate.receive(True, 1.7)
        self.assertEqual(self.gate.reason(1.7), "STOP_REQUESTED")
        self.gate.receive(False, 1.8)
        self.assertEqual(self.gate.reason(1.9), "WAITING_CLEAR")

    def test_timeout_resets_clear_even_without_watchdog_tick(self):
        self.clear()
        self.gate.receive(False, 3.)
        self.assertEqual(self.gate.reason(3.), "WAITING_CLEAR")

    def test_stale_false_never_releases(self):
        self.clear()
        self.assertEqual(self.gate.reason(2.7), "VISION_TIMEOUT")

    def test_default_no_peer_does_not_stop_but_true_still_stops(self):
        self.assertFalse(load_config().vision_required)
        gate = PigStopGate(load_config())
        self.assertIsNone(gate.reason(100))
        gate.receive(True, 100)
        self.assertEqual(gate.reason(101), "STOP_REQUESTED")
        self.assertEqual(gate.reason(200), "STOP_REQUESTED")

    def test_optional_peer_false_and_silence_do_not_create_hold(self):
        gate = PigStopGate(load_config())
        gate.receive(False, 0.)
        self.assertIsNone(gate.reason(0.))
        self.assertIsNone(gate.reason(100.))

    def test_optional_stop_needs_fresh_clear_not_silence(self):
        gate = PigStopGate(load_config())
        gate.receive(True, 0.)
        gate.receive(False, .1)
        self.assertEqual(gate.reason(10.), "WAITING_CLEAR")
        for i in range(17):
            gate.receive(False, 11. + i / 10)
        self.assertIsNone(gate.reason(12.6))
        self.assertIsNone(gate.reason(100.))


class ExternalHoldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config()
        cls.features = scan_features(StaticLidarScene().scan(cls.cfg.start), cls.cfg)

    def controller(self, state="MAIN"):
        ctrl = TraditionalPatrolController(self.cfg)
        ctrl.state = state
        ctrl.last_stamp = 1.
        ctrl.state_stamp = .5
        ctrl.junctions_done, ctrl.endpoints_reached = 1, 3
        ctrl.pause_external()
        return ctrl

    def test_all_route_states_hold_counters_and_timeout(self):
        states = ("MAIN", "FINAL_MAIN", "CENTER", "TURN_LEFT", "ENTRY_LEFT",
                  "LEFT_OUTBOUND", "REVERSE_LEFT", "CROSS_REVERSE", "RIGHT_OUTBOUND",
                  "RETURN_RIGHT", "RETURN_CENTER", "TURN_MAIN", "EXIT_MAIN")
        for state in states:
            with self.subTest(state=state):
                ctrl = self.controller(state)
                for stamp in (2., 80., 200.):
                    np.testing.assert_array_equal(ctrl.command(self.features, stamp), [0, 0])
                self.assertEqual((ctrl.state, ctrl.junctions_done, ctrl.endpoints_reached),
                                 (state, 1, 3))
                self.assertAlmostEqual(ctrl.last_stamp - ctrl.state_stamp, .5)
                self.assertIsNone(ctrl.failure)

    def test_resume_requires_three_fresh_localized_scans(self):
        ctrl = self.controller()
        for stamp in (1.1, 1.2):
            ctrl.hold_external(self.features, stamp, release=True)
            self.assertTrue(ctrl.external_paused)
        ctrl.hold_external(self.features, 1.3, release=True)
        self.assertFalse(ctrl.external_paused)
        self.assertEqual(ctrl.state, "MAIN")

    def test_duplicate_or_missing_geometry_breaks_confirmation(self):
        ctrl = self.controller()
        ctrl.hold_external(self.features, 1.1, release=True)
        ctrl.hold_external(self.features, 1.1, release=True)
        self.assertEqual(ctrl.external_resume_frames, 0)
        with patch("lidar_patrol_controller.incoming_corridor", return_value=None):
            ctrl.hold_external(self.features, 1.2, release=True)
        self.assertEqual(ctrl.external_resume_frames, 0)
        self.assertTrue(ctrl.external_paused)

    def test_entry_cannot_skip_depth_using_only_parallel_walls(self):
        ctrl = self.controller("CROSS_REVERSE")
        for i in range(4):
            ctrl.hold_external(self.features, 1.1 + i / 10, release=True)
        self.assertTrue(ctrl.external_paused)
        self.assertEqual(ctrl.entry_phase, "JUNCTION")
        ctrl.entry_phase = "WALL_CONFIRM"
        for i in range(3):
            ctrl.hold_external(self.features, 1.5 + i / 10, release=True)
        self.assertFalse(ctrl.external_paused)
        self.assertEqual(ctrl.state, "CROSS_REVERSE")

    def test_interrupted_turn_preserves_original_incoming_axis(self):
        ctrl = self.controller("TURN_LEFT")
        geometry = JunctionGeometry(np.array([.01, .02]), (-.6, math.pi / 2 - .6),
                                    1., (1.25, 1.04))
        ctrl.tracker = JunctionTracker(geometry, 1., self.cfg, heading_hint=-.6)
        with patch.object(ctrl, "_geometry", return_value=geometry):
            for stamp in (50., 50.1, 50.2):
                ctrl.hold_external(self.features, stamp, release=True)
        self.assertFalse(ctrl.external_paused)
        self.assertEqual(ctrl.state, "TURN_LEFT")
        self.assertAlmostEqual(ctrl.tracker.incoming_axis, -.6)

    def test_turn_geometry_jump_keeps_old_reference_and_stops(self):
        ctrl = self.controller("TURN_MAIN")
        geometry = JunctionGeometry(np.zeros(2), (0., math.pi / 2), 1.)
        ctrl.tracker = JunctionTracker(geometry, 1., self.cfg)
        jumped = replace(geometry, centre=np.array([1., 0.]))
        with patch.object(ctrl, "_geometry", return_value=jumped):
            for i in range(4):
                ctrl.hold_external(self.features, 1.1 + i / 10, release=True)
        self.assertTrue(ctrl.external_paused)
        np.testing.assert_array_equal(ctrl.tracker.geometry.centre, [0, 0])

    def test_retreat_is_interrupted_and_not_automatically_restarted(self):
        ctrl = self.controller()
        ctrl.obstacle = dict(phase="RETREAT", attempted=False)
        ctrl.pause_external()
        self.assertEqual(ctrl.obstacle["phase"], "WAIT")
        self.assertTrue(ctrl.obstacle["attempted"])

    def test_terminal_failure_and_reverse_time_remain_latched(self):
        ctrl = self.controller()
        ctrl.hold_external(self.features, .9, release=True)
        self.assertEqual(ctrl.failure, "scan_time_reversed")
        for i in range(4):
            ctrl.hold_external(self.features, 2. + i / 10, release=True)
        self.assertEqual(ctrl.state, "FAILED")

    def test_peer_config_rejects_invalid_values(self):
        for setting in ({"vision_required": "false"}, {"vision_status_timeout": 0},
                        {"pig_stop_clear_seconds": -1}, {"state_publish_period": float("nan")}):
            with self.subTest(setting=setting), patch("navigation.Path.read_text",
                                                     return_value=json.dumps(setting)):
                with self.assertRaises(ValueError):
                    load_config()


@unittest.skipIf(LidarPatrolNode is None, "Source ROS 2 to test the peer adapter")
class PeerNodeTests(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(LidarPatrolNode)
        self.node.cfg = load_config()
        self.node.controller = TraditionalPatrolController(self.node.cfg)
        self.node.pub = Mock()
        self.node.state_pub = Mock()
        self.node.get_logger = Mock(return_value=Mock())
        self.node.last_received = None
        self.node.reported = None
        self.node.init_scan_health()
        self.scan = StaticLidarScene().scan(self.node.cfg.start)

    def request(self, stop, now):
        with patch("fsm_lidar_patrol.time.monotonic", return_value=now):
            self.node.on_pig_stop(SimpleNamespace(data=stop))

    def scan_at(self, now):
        self.scan.header = SimpleNamespace(stamp=SimpleNamespace(
            sec=int(now), nanosec=round((now % 1) * 1e9)))
        with patch("fsm_lidar_patrol.time.monotonic", return_value=now):
            self.node.on_scan(self.scan)

    def assert_stopped(self):
        msg = self.node.pub.publish.call_args.args[0]
        self.assertEqual((msg.linear.x, msg.angular.z), (0., 0.))

    def ready(self):
        for i in range(22):
            now = 1 + i / 10
            self.request(False, now)
            self.scan_at(now)
        self.assertFalse(self.node.controller.external_paused)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_startup_stops_without_peer(self):
        self.node.cfg = replace(self.node.cfg, vision_required=True)
        self.node.init_scan_health()
        self.scan_at(1.)
        self.assert_stopped()
        self.assertTrue(self.node.controller.external_paused)

    def test_true_immediately_stops_both_axes_and_false_alone_cannot_move(self):
        self.ready()
        self.request(True, 3.2)
        self.assert_stopped()
        for i in range(18):
            self.request(False, 3.3 + i / 10)
        self.assert_stopped()
        self.assertTrue(self.node.controller.external_paused)

    def test_request_deferred_during_manoeuvres_but_accepted_on_straight_road(self):
        ctrl = self.node.controller
        for state in STATE_LABELS:
            if state in ctrl.TERMINAL:
                continue
            with self.subTest(state=state):
                ctrl.state = state
                ctrl.external_paused = False
                self.request(True, 1.)
                self.assertEqual(ctrl.external_paused, state in ctrl.CORRIDOR_STATES)
        ctrl.state = "TURN_LEFT"
        ctrl.external_paused = False
        self.request(True, 2.)
        ctrl.state = "LEFT_OUTBOUND"
        with patch("fsm_lidar_patrol.time.monotonic", return_value=2.1):
            self.node.publish((.2, .1))
        self.assert_stopped()

    def test_peer_timeout_stops_even_during_turn(self):
        self.node.cfg = replace(self.node.cfg, vision_required=True)
        self.node.init_scan_health()
        ctrl = self.node.controller
        ctrl.state = "TURN_LEFT"
        ctrl.external_paused = False
        self.request(True, 1.)
        self.node.last_received = 2.1
        with patch("fsm_lidar_patrol.time.monotonic", return_value=2.1):
            self.node.check_scan()
        self.assertTrue(ctrl.external_paused)
        self.assert_stopped()

    def test_default_patrol_without_peer_and_after_false_timeout(self):
        self.scan_at(1.)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)
        self.request(False, 1.1)
        for i in range(1, 25):
            self.scan_at(1. + i / 10)
        self.assertFalse(self.node.controller.external_paused)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_optional_true_remains_stopped_after_peer_disconnect(self):
        self.scan_at(1.)
        self.request(True, 1.1)
        for i in range(1, 25):
            self.scan_at(1. + i / 10)
            self.assert_stopped()
        self.assertTrue(self.node.controller.external_paused)

    def test_full_stop_release_cycle_retains_route_and_counters(self):
        self.ready()
        self.node.controller.endpoints_reached = 2
        self.request(True, 3.2)
        for i in range(1, 22):
            now = 3.2 + i / 10
            self.request(False, now)
            self.scan_at(now)
        self.assertFalse(self.node.controller.external_paused)
        self.assertEqual(self.node.controller.endpoints_reached, 2)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_sensor_failure_cannot_be_overridden_by_false(self):
        self.request(False, 1.)
        self.scan.ranges[:] = float("nan")
        for i in range(35):
            now = 1 + i / 10
            self.request(False, now)
            self.scan_at(now)
        self.assertEqual(self.node.controller.state, "FAILED")
        self.assert_stopped()

    def test_state_reports_route_even_while_paused(self):
        self.node.controller.state = "RIGHT_OUTBOUND"
        self.node.publish_state()
        self.assertEqual(self.node.state_pub.publish.call_args.args[0].data, "RIGHT_OUTBOUND")
        self.node.publish_state()
        self.assertEqual(self.node.state_pub.publish.call_count, 2)

    def test_constructor_wires_topics_heartbeat_and_startup_override(self):
        # Exercise construction without creating a ROS participant. Actual
        # message classes and QoS profiles are used, not fake ROS modules.
        from std_msgs.msg import Bool, String
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
        with patch("fsm_lidar_patrol.Node.__init__", return_value=None), \
                patch.object(LidarPatrolNode, "declare_parameter",
                             return_value=SimpleNamespace(value=True)), \
                patch.object(LidarPatrolNode, "create_publisher") as publishers, \
                patch.object(LidarPatrolNode, "create_subscription") as subscriptions, \
                patch.object(LidarPatrolNode, "create_timer") as timers, \
                patch.object(LidarPatrolNode, "get_logger", return_value=Mock()):
            node = LidarPatrolNode()
            self.assertTrue(node.cfg.vision_required)
            self.assertTrue(node.controller.external_paused)
            state_call = next(call for call in publishers.call_args_list
                              if call.args[1] == "/patrol/state")
            self.assertIs(state_call.args[0], String)
            self.assertEqual(state_call.args[2].durability, DurabilityPolicy.TRANSIENT_LOCAL)
            stop_call = next(call for call in subscriptions.call_args_list
                             if call.args[1] == "/pig_face/stop_requested")
            self.assertIs(stop_call.args[0], Bool)
            self.assertEqual(stop_call.args[3].reliability, ReliabilityPolicy.RELIABLE)
            self.assertEqual(stop_call.args[3].durability, DurabilityPolicy.VOLATILE)
            self.assertTrue(any(call.args[:2] == (.5, node.publish_state)
                                for call in timers.call_args_list))


if __name__ == "__main__":
    unittest.main()
