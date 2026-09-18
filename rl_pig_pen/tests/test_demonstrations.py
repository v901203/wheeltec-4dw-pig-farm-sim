from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from demonstrations import (DemoWriter, demonstration_sample, load_demonstrations,
                            split_demonstrations)
from navigation import Config
from record_demonstrations import DemonstrationRecorder
from test_navigation import corridor_scan


class DemonstrationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_good_corridor_control_is_preserved(self):
        sample, reason = demonstration_sample(corridor_scan(), [0.2, 0.3], self.cfg)
        self.assertEqual(reason, "accepted")
        self.assertEqual(sample[0].shape, (38,))
        np.testing.assert_allclose(sample[1], [0.2, 0.3])

    def test_reverse_turn_idle_and_unsafe_commands_are_excluded(self):
        for action, reason in (([-0.2, 0], "unsupported_action"), ([2, 0], "unsupported_action"),
                               ([0, 0.5], "turn_in_place"), ([0, 0], "idle"),
                               ([float("nan"), 0], "invalid_action")):
            self.assertEqual(demonstration_sample(corridor_scan(), action, self.cfg)[1], reason)
        s = corridor_scan()
        s.ranges[180] = 0.6
        self.assertEqual(demonstration_sample(s, [0.8, 0], self.cfg)[1], "unsafe_command")
        self.assertEqual(demonstration_sample(s, [0, 0], self.cfg)[1], "accepted")
        s.ranges[180] = 0.2
        self.assertEqual(demonstration_sample(s, [0, 0], self.cfg)[1], "collision")

    def test_chunk_roundtrip_and_final_partial_flush(self):
        obs, action = demonstration_sample(corridor_scan(), [0.2, 0], self.cfg)[0]
        with tempfile.TemporaryDirectory() as directory:
            writer = DemoWriter(directory, self.cfg, chunk_size=2)
            for i in range(3):
                writer.add(obs, action, i, i, 0.01)
            self.assertEqual(writer.saved, 2)
            writer.flush()
            loaded_obs, loaded_actions, groups = load_demonstrations(directory, self.cfg)
            self.assertEqual(len(list(writer.directory.glob("chunk_*.npz"))), 2)
            np.testing.assert_array_equal(loaded_obs, np.tile(obs, (3, 1)))
            np.testing.assert_array_equal(loaded_actions, np.tile(action, (3, 1)))
            self.assertEqual(len(np.unique(groups)), 1)

    def test_split_holds_out_whole_sessions(self):
        groups = np.repeat(["session-a", "session-b", "session-c"], 100)
        train, val, _ = split_demonstrations(groups)
        self.assertFalse(set(groups[train]) & set(groups[val]))
        train, val, _ = split_demonstrations(np.repeat("single", 100))
        self.assertEqual(val.min()-train.max()-1, 12)

    def test_wrong_mounting_is_rejected(self):
        from dataclasses import replace
        obs, action = demonstration_sample(corridor_scan(), [0.2, 0], self.cfg)[0]
        with tempfile.TemporaryDirectory() as directory:
            writer = DemoWriter(directory, self.cfg)
            writer.add(obs, action, 1, 1, 0)
            writer.flush()
            with self.assertRaisesRegex(ValueError, "mounting"):
                load_demonstrations(directory, replace(self.cfg, lidar_yaw=1))

    def test_recorder_pairs_only_fresh_unique_scans_from_one_controller(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = DemoWriter(directory, self.cfg)
            scan = corridor_scan()
            scan.header = NS(stamp=NS(sec=1, nanosec=0))
            node = NS(writer=writer, cfg=self.cfg, max_scan_age=0.25, cmd_topic="/cmd_vel",
                      count_publishers=Mock(return_value=1), latest_scan=(scan, 10.0), used_stamp=None)
            cmd = NS(linear=NS(x=0.2, y=0, z=0), angular=NS(x=0, y=0, z=0.1))
            with patch("record_demonstrations.time.monotonic", return_value=10.1):
                DemonstrationRecorder._command_cb(node, cmd)
                DemonstrationRecorder._command_cb(node, cmd)
                node.count_publishers.return_value = 2
                DemonstrationRecorder._command_cb(node, cmd)
            node.count_publishers.return_value = 1
            with patch("record_demonstrations.time.monotonic", return_value=11.0):
                DemonstrationRecorder._command_cb(node, cmd)
            self.assertEqual(writer.counts["accepted"], 1)
            self.assertEqual(writer.counts["duplicate_scan"], 1)
            self.assertEqual(writer.counts["multiple_or_unknown_command_sources"], 1)
            self.assertEqual(writer.counts["stale_scan"], 1)


if __name__ == "__main__":
    unittest.main()
