"""Exercise exit ordering without opening DDS or publishing robot commands."""

from pathlib import Path
import signal
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import fsm_lidar_patrol as patrol
except ImportError:
    patrol = None


@unittest.skipIf(patrol is None, "Source ROS 2 to test the ROS adapter")
class PatrolShutdownTests(unittest.TestCase):
    def run_exit(self, mode):
        node = Mock()
        node.context.ok.return_value = mode != "external"
        events = []
        node.publish.side_effect = lambda command: events.append(("publish", command))
        node.destroy_node.side_effect = lambda: events.append(("destroy",))
        handlers = {}

        def spin(*args, **kwargs):
            if mode == "keyboard":
                raise KeyboardInterrupt()
            if mode == "external":
                raise patrol.ExternalShutdownException()
            # Invoke our installed signal handler, not rclpy's shutdown handler.
            handlers[signal.SIGINT if mode == "sigint" else signal.SIGTERM](0, None)

        with patch.object(patrol, "LidarPatrolNode", return_value=node), \
                patch.object(patrol.rclpy, "init") as init, \
                patch.object(patrol.rclpy, "ok", return_value=True), \
                patch.object(patrol.rclpy, "spin_once", side_effect=spin), \
                patch.object(patrol, "try_shutdown", side_effect=lambda: events.append(("shutdown",))), \
                patch.object(patrol.signal, "getsignal", return_value="original"), \
                patch.object(patrol.signal, "signal", side_effect=lambda sig, h: handlers.update({sig: h})), \
                patch.object(patrol.sys, "stderr"):
            patrol.main()
        init.assert_called_once_with(args=None, signal_handler_options=patrol.SignalHandlerOptions.NO)
        self.assertEqual(handlers, {signal.SIGINT: "original", signal.SIGTERM: "original"})
        return events

    def test_sigint_sends_zero_before_destroy_and_shutdown(self):
        self.assertEqual(self.run_exit("sigint"), [("publish", (0., 0.)), ("destroy",), ("shutdown",)])

    def test_sigterm_also_requests_orderly_stop(self):
        self.assertEqual(self.run_exit("sigterm")[0], ("publish", (0., 0.)))

    def test_keyboard_interrupt_fallback_stops(self):
        self.assertEqual(self.run_exit("keyboard")[0], ("publish", (0., 0.)))

    def test_already_closed_context_does_not_publish_or_shutdown_twice(self):
        self.assertEqual(self.run_exit("external"), [("destroy",), ("shutdown",)])


if __name__ == "__main__":
    unittest.main()
