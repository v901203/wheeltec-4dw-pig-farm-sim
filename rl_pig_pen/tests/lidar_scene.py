"""Local static collision raycaster for tests, not a physics/Gazebo substitute."""

import math
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np


class StaticLidarScene:
    def __init__(self, rays=1640, radius=6.0):
        self.angles = np.linspace(-math.pi, math.pi, rays)
        self.radius = radius
        boxes, circles = [], []
        world = ET.parse(Path(__file__).resolve().parents[2] / "worlds/pig_pen_16units.world")
        def xyz(element):
            pose = np.fromstring(element.findtext("pose", "0 0 0 0 0 0"), sep=" ")
            if np.any(pose[3:]):
                raise ValueError("Static test raycaster only supports axis-aligned world geometry")
            return pose[:3]
        for model in world.findall("./world/model"):
            for link in model.findall("link"):
                for collision in link.findall("collision"):
                    position = xyz(model)+xyz(link)+xyz(collision)
                    box = collision.find("geometry/box")
                    cylinder = collision.find("geometry/cylinder")
                    if box is not None:
                        size = np.fromstring(box.findtext("size"), sep=" ")
                        if abs(position[2]-.245) <= size[2]/2:
                            boxes.append([*(position[:2]-size[:2]/2), *(position[:2]+size[:2]/2)])
                    if cylinder is not None and abs(position[2]-.245) <= float(cylinder.findtext("length"))/2:
                        circles.append([*position[:2], float(cylinder.findtext("radius"))])
        self.boxes, self.circles = np.array(boxes), np.array(circles)

    def scan(self, pose):
        xy, yaw = np.asarray(pose[:2]), float(pose[2])
        directions = np.column_stack((np.cos(self.angles+yaw), np.sin(self.angles+yaw)))
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (self.boxes[None, :, :2]-xy) / directions[:, None, :]
            t2 = (self.boxes[None, :, 2:]-xy) / directions[:, None, :]
        entry = np.max(np.minimum(t1, t2), axis=2)
        leave = np.min(np.maximum(t1, t2), axis=2)
        ranges = np.min(np.where((leave >= entry) & (entry > 0), entry, np.inf), axis=1)
        nearby = np.linalg.norm(self.circles[:, :2]-xy, axis=1) < self.radius
        relative, radii = self.circles[nearby, :2]-xy, self.circles[nearby, 2]
        if len(relative):
            projection = directions @ relative.T
            cross = directions[:, 0, None]*relative[None, :, 1] - directions[:, 1, None]*relative[None, :, 0]
            discriminant = radii**2-cross**2
            hits = projection-np.sqrt(np.maximum(0, discriminant))
            ranges = np.minimum(ranges, np.min(np.where((discriminant >= 0) & (hits > 0), hits, np.inf), axis=1))
        ranges[ranges > self.radius] = np.inf
        return NS(ranges=ranges, angle_min=self.angles[0], angle_increment=self.angles[1]-self.angles[0],
                  range_min=.1, range_max=25.)
