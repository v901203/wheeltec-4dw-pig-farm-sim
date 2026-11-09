"""Generate training-only assets, including timestamped model ground truth."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from prepare_training_scene import prepare


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--world", required=True)
    p.add_argument("--robot", required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args()
    prepare(args.world, args.robot, args.output_dir, rtf=1., lidar_only=True)
    target = args.output_dir / "training.urdf"
    tree = ET.parse(target)
    plugin = ET.SubElement(ET.SubElement(tree.getroot(), "gazebo"), "plugin",
        filename="libignition-gazebo-pose-publisher-system.so",
        name="ignition::gazebo::systems::PosePublisher")
    for key, value in {
        "publish_model_pose": "true", "publish_link_pose": "false",
        "publish_sensor_pose": "false", "publish_collision_pose": "false",
        "publish_visual_pose": "false", "publish_nested_model_pose": "true",
        "use_pose_vector_msg": "false", "update_frequency": "50",
    }.items():
        ET.SubElement(plugin, key).text = value
    tree.write(target, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
