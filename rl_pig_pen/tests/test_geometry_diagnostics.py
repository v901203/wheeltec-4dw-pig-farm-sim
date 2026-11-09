"""Raw scan chunks retain their exact journal mapping, including fault frames."""
import gzip
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from geometry_diagnostics import GeometryDiagnosticsLog
from lidar_patrol_controller import TraditionalPatrolController
from navigation import load_config


class GeometryLogTests(unittest.TestCase):
    def test_raw_mapping_copy_and_final_partial_chunk(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)/"journal"
            writer = GeometryDiagnosticsLog(root, chunk_size=2)
            c = TraditionalPatrolController(load_config())
            c.geometry_diagnostics = [{"rejected": {"occupied_opening": 1}}]
            scan = SimpleNamespace(ranges=np.zeros(8), angle_min=-3., angle_increment=.1,
                                   range_min=.1, range_max=25.)
            for i in range(3):
                scan.ranges[:] = i+1
                writer.record_scan(scan, i*.2, c)
            scan.ranges[:] = 99
            writer.close()
            writer.close()
            rows = [json.loads(line) for line in gzip.open(root/"decisions.jsonl.gz", "rt")]
            self.assertEqual(len(rows), 3)
            for i, r in enumerate(rows):
                with np.load(root/f"scans_{r['raw_chunk']:05d}.npz") as data:
                    np.testing.assert_array_equal(data["ranges"][r["raw_index"]], np.full(8, i+1))
                self.assertEqual(r["detections"][0]["rejected"]["occupied_opening"], 1)

    def test_fault_flushes_raw_scan_before_normal_chunk_limit(self):
        with tempfile.TemporaryDirectory() as temp:
            writer = GeometryDiagnosticsLog(Path(temp)/"journal")
            c = TraditionalPatrolController(load_config())
            c.last_stamp = 1.
            c.fail("junction_geometry_lost")
            scan = SimpleNamespace(ranges=np.ones(8), angle_min=-3., angle_increment=.1,
                                   range_min=.1, range_max=25.)
            writer.record_scan(scan, 1., c)
            self.assertTrue((writer.directory/"scans_00000.npz").exists())
            writer.close()
