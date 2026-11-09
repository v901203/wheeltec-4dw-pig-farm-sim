from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
from types import SimpleNamespace
import numpy as np
from navigation import load_config,SensorFault
from check_motion import inspect_scan,recent_log_lines
import tempfile


class MotionDiagnosticsTests(unittest.TestCase):
    def scan(self):
        return SimpleNamespace(ranges=np.full(720,float('inf')),angle_min=-np.pi,
                               angle_increment=2*np.pi/720,range_min=.05,range_max=25.)
    def test_front_hit_blocks_forward_but_not_reverse(self):
        scan=self.scan();scan.ranges[360]=.35
        result=inspect_scan(scan,load_config(),{'forward':(.05,0.),'reverse':(-.05,0.)})
        self.assertTrue(result['probes']['forward']['blocked'])
        self.assertFalse(result['probes']['reverse']['blocked'])
        self.assertEqual(result['probes']['forward']['hits'][0]['guard'],'travel')
        self.assertAlmostEqual(result['probes']['forward']['hits'][0]['body_angle_deg'],0.)
    def test_post_mask_excluded_but_unmasked_invalid_reported(self):
        scan=self.scan();scan.ranges[436]=.15 # +38 degrees, ignored corner post
        result=inspect_scan(scan,load_config(),{'stop':(0.,0.)})
        self.assertFalse(result['probes']['stop']['blocked'])
        scan.ranges[360]=float('nan')
        result=inspect_scan(scan,load_config(),{'stop':(0.,0.)})
        self.assertEqual(result['invalid_unmasked_rays'],1)
    def test_footprint_hit_blocks_even_zero_command(self):
        scan=self.scan();scan.ranges[360]=.15
        result=inspect_scan(scan,load_config(),{"stop":(0.,0.)})
        self.assertTrue(result["probes"]["stop"]["blocked"])
        self.assertIn("footprint",{h["guard"] for h in result["probes"]["stop"]["hits"]})
    def test_bad_metadata_raises_sensor_fault(self):
        scan=self.scan();scan.angle_increment=0.
        with self.assertRaises(SensorFault):inspect_scan(scan,load_config(),{})
    def test_log_clue_is_read_only(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'patrol.log';p.write_text('[INFO] moving\n[WARN] 原因：obstacle_clearance\n')
            before=p.read_bytes();result=recent_log_lines(Path(folder))
            self.assertEqual(len(result),1);self.assertEqual(p.read_bytes(),before)

if __name__=='__main__':unittest.main()
