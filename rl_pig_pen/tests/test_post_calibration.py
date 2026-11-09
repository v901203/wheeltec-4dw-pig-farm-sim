import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibrate_lidar_posts import PostSurvey

class PostCalibrationTests(unittest.TestCase):
    def test_raw_corner_edges_are_retained_without_existing_mask(self):
        angles = np.arange(-180., 180., .1)
        ranges = np.full(len(angles), float('inf'))
        ranges[(angles >= 144.5) & (angles <= 145.)] = .175
        scan = SimpleNamespace(ranges=ranges, angle_min=-math.pi,
            angle_increment=math.radians(.1), range_min=.05, range_max=10.)
        survey = PostSurvey()
        for _ in range(20):
            survey.add(scan)
        report = survey.report()
        self.assertEqual(report['frames'], 20)
        self.assertEqual(len(report['candidates']), 1)
        candidate = report['candidates'][0]
        self.assertTrue(candidate['corner_candidate'])
        self.assertAlmostEqual(candidate['median_range_m'], .175)
        self.assertLessEqual(candidate['low_deg'], 144.6)
        self.assertGreaterEqual(candidate['high_deg'], 145.)
        self.assertLessEqual(max(survey.hits), survey.frames)

    def test_intermittent_point_is_not_stable_candidate(self):
        scan = SimpleNamespace(ranges=[.2], angle_min=math.pi/4,
            angle_increment=.01, range_min=.05, range_max=10.)
        survey = PostSurvey()
        survey.add(scan)
        scan.ranges = [float('nan')]
        for _ in range(30):
            survey.add(scan)
        self.assertFalse(survey.report()['candidates'])
