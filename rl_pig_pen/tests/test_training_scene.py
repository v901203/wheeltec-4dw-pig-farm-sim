from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from prepare_training_scene import prepare

WORLD = ROOT / 'worlds/pig_pen_16units.world'
ROBOT = ROOT / 'turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf'


class TrainingSceneTests(unittest.TestCase):
    def test_fast_copy_preserves_physics_geometry_and_lidar(self):
        original_world, original_robot = WORLD.read_bytes(), ROBOT.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            count = prepare(WORLD, ROBOT, directory, rtf=6, lidar_only=True)
            self.assertEqual(count, 3)
            result = ET.parse(Path(directory) / 'training.world')
            source = ET.parse(WORLD)
            self.assertEqual(result.findtext('./world/physics/real_time_factor'), '6')
            self.assertEqual(result.findtext('./world/physics/max_step_size'),
                             source.findtext('./world/physics/max_step_size'))
            for tag in ('model', 'include', 'plugin'):
                self.assertEqual([ET.tostring(x) for x in result.findall('./world/' + tag)],
                                 [ET.tostring(x) for x in source.findall('./world/' + tag)])
            robot = ET.parse(Path(directory) / 'training.urdf')
            sensors = robot.findall('.//sensor')
            self.assertEqual([s.get('type') for s in sensors], ['gpu_lidar'])
            self.assertEqual(ET.tostring(sensors[0]), ET.tostring(ET.parse(ROBOT).find('.//sensor')))
        self.assertEqual(WORLD.read_bytes(), original_world)
        self.assertEqual(ROBOT.read_bytes(), original_robot)

    def test_camera_and_world_pacing_can_be_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(prepare(WORLD, ROBOT, directory), 0)
            result = ET.parse(Path(directory) / 'training.world')
            self.assertEqual(result.findtext('./world/physics/real_time_factor'),
                             ET.parse(WORLD).findtext('./world/physics/real_time_factor'))
            self.assertEqual(len(ET.parse(Path(directory) / 'training.urdf').findall('.//sensor')), 4)

    def test_invalid_factor_is_rejected_before_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'invalid'
            for value in (0, -1, float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    prepare(WORLD, ROBOT, target, rtf=value)
                self.assertFalse(target.exists())


if __name__ == '__main__':
    unittest.main()
