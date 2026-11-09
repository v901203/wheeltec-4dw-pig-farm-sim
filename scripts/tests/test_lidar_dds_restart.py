import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('restart_lidar_dds',
    Path(__file__).resolve().parents[1] / 'restart_lidar_dds.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class LidarSelectionTests(unittest.TestCase):
    def test_targets_only_m10p_launcher_or_x10_driver(self):
        self.assertTrue(module.is_lidar([b'/usr/bin/python3', b'/opt/ros/humble/install/bin/ros2',
            b'launch', b'lslidar_driver', b'lslidar_m10p_launch.py']))
        self.assertTrue(module.is_lidar([b'/workspace/install/lib/lslidar_driver_node',
            b'--ros-args', b'-r', b'__ns:=/x10']))
        self.assertFalse(module.is_lidar([b'python3', b'c270_arm_tracker_node.py']))
        self.assertFalse(module.is_lidar([b'/lib/lslidar_driver_node', b'__ns:=/other']))
        self.assertFalse(module.is_lidar([b'ros2', b'topic', b'echo', b'/x10/scan']))
