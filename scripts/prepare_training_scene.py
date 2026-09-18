#!/usr/bin/env python3
"""Prepare temporary RL assets without editing the original world or robot."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(world_file, robot_file, output_dir, *, rtf=None, lidar_only=False):
    if rtf is not None and (not math.isfinite(rtf) or rtf <= 0):
        raise ValueError("--rtf must be a finite positive number")
    world_tree, robot_tree = ET.parse(world_file), ET.parse(robot_file)
    world = world_tree.getroot().find("world")
    if world is None:
        raise ValueError("Expected an SDF file containing a world")
    if rtf is not None:
        physics = next((p for p in world.findall("physics") if p.get("default") in ("1", "true")),
                       world.find("physics"))
        if physics is None:
            physics = ET.SubElement(world, "physics", {"name": "rl", "type": "ignored"})
            ET.SubElement(physics, "max_step_size").text = "0.001"
        step_size = float(physics.findtext("max_step_size", "0.001"))
        if not math.isfinite(step_size) or step_size <= 0:
            raise ValueError("Invalid physics max_step_size")
        # Change pacing, never the integration timestep or sensor update rates.
        for tag, value in (("real_time_factor", rtf), ("real_time_update_rate", rtf / step_size)):
            element = physics.find(tag)
            if element is None:
                element = ET.SubElement(physics, tag)
            element.text = str(value)
    removed = 0
    if lidar_only:
        for parent in robot_tree.getroot().iter():
            for sensor in list(parent.findall("sensor")):
                if sensor.get("type") in ("camera", "depth", "depth_camera", "rgbd_camera"):
                    parent.remove(sensor)
                    removed += 1
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    world_tree.write(output_dir / "training.world", encoding="utf-8", xml_declaration=True)
    robot_tree.write(output_dir / "training.urdf", encoding="utf-8", xml_declaration=True)
    return removed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--robot", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--rtf", type=float)
    parser.add_argument("--lidar-only", action="store_true")
    args = parser.parse_args()
    removed = prepare(args.world, args.robot, args.output_dir, rtf=args.rtf, lidar_only=args.lidar_only)
    print(f"Training scene: target RTF={args.rtf if args.rtf is not None else 'original'}, "
          f"disabled cameras={removed}")


if __name__ == "__main__":
    main()
