"""Serial safety tests: mocked ROS/serial endpoints, no hardware writes."""

from pathlib import Path
import struct
import sys
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
try:
    from cmdvel_to_stm32 import CmdVelToSTM32
    from geometry_msgs.msg import Twist
except ImportError:
    CmdVelToSTM32 = None


@unittest.skipIf(CmdVelToSTM32 is None, "Source ROS 2 to test the serial adapter")
class CommandWatchdogTests(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(CmdVelToSTM32)
        self.node.lock = threading.Lock()
        self.node.ser = Mock()
        self.node.get_logger = Mock(return_value=Mock())
        self.node.last_twist = Twist()
        self.node.last_command_time = None
        self.node.command_timeout = .5
        self.node.timed_out = False

    def packet_velocity(self):
        packet = self.node.ser.write.call_args.args[0]
        self.assertEqual(len(packet), 11)
        self.assertEqual((packet[0], packet[-1]), (0x7B, 0x7D))
        checksum = 0
        for byte in packet[:9]:
            checksum ^= byte
        self.assertEqual(checksum, packet[9])
        return struct.unpack(">hhh", packet[3:9])

    def receive(self, stamp, x=.2, y=-.1, w=.3):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = x, y, w
        with patch("cmdvel_to_stm32.time.monotonic", return_value=stamp):
            self.node.cb_cmdvel(msg)

    def tick(self, stamp):
        with patch("cmdvel_to_stm32.time.monotonic", return_value=stamp):
            self.node.timer_cb()
        return self.packet_velocity()

    def test_startup_expiry_repeated_zero_and_fresh_command(self):
        self.assertEqual(self.tick(0.), (0, 0, 0))
        self.receive(1.)
        self.assertEqual(self.tick(1.49), (200, -100, 300))
        for stamp in (1.5, 2., 10.):
            self.assertEqual(self.tick(stamp), (0, 0, 0))
        self.node.get_logger.return_value.warning.assert_called_once()
        self.receive(11., x=-.05)
        self.assertEqual(self.tick(11.1), (-50, -100, 300))

    def test_nonfinite_input_revokes_cached_motion(self):
        for bad in (float("nan"), float("inf"), -float("inf")):
            self.receive(1.)
            self.receive(1.1, w=bad)
            self.assertEqual(self.tick(1.2), (0, 0, 0))

    def test_write_error_does_not_replay_old_motion(self):
        self.receive(1.)
        self.node.ser.write.side_effect = OSError("link lost")
        self.tick(1.1)
        self.node.ser.write.side_effect = None
        self.assertEqual(self.tick(1.2), (0, 0, 0))

    def test_orderly_shutdown_sends_zero_before_close(self):
        self.receive(1.)
        self.node.stop_and_close()
        self.assertEqual(self.packet_velocity(), (0, 0, 0))
        self.assertEqual([call[0] for call in self.node.ser.method_calls], ["write", "close"])

    def test_large_finite_values_cannot_break_stop_timer(self):
        self.receive(1., x=1e308, y=-1e308)
        self.assertEqual(self.tick(1.1), (32767, -32768, 300))
        self.assertEqual(self.tick(1.6), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
