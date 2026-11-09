"""Task geometry and privileged scoring for the dedicated junction world."""
from dataclasses import dataclass
import math
import numpy as np
from pathlib import Path
from functools import lru_cache
import xml.etree.ElementTree as ET

WORLD_FILE = Path(__file__).resolve().parents[1] / "worlds/pig_pen_16units.world"
WORLD_NAME = "pig_pen_16units_world"
CENTRE_Y = -6.18


@dataclass(frozen=True)
class Task:
    name: str
    start: tuple
    goal: tuple


TASKS = (
    Task("main_to_left", (0., CENTRE_Y-2, math.pi / 2), (-2., CENTRE_Y, math.pi)),
    Task("left_to_right_reverse", (-2., CENTRE_Y, math.pi), (2., CENTRE_Y, math.pi)),
    Task("right_to_main", (2., CENTRE_Y, math.pi), (0., CENTRE_Y+2, math.pi / 2)),
)


def angle_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def walls(width=1.4, extent=4.):
    """Axis-aligned wall bounds shared by scene generation and collision score."""
    h = width / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            yield (sx*(extent+h)/2, sy*(extent+h)/2, (extent-h)/2, (extent-h)/2)
    for x, y, hx, hy in ((0, extent+.1, extent+.2, .1),
                         (0, -extent-.1, extent+.2, .1),
                         (extent+.1, 0, .1, extent+.2),
                         (-extent-.1, 0, .1, extent+.2)):
        yield x, y, hx, hy


@lru_cache(maxsize=1)
def static_geometry():
    """Original map geometry intersecting the 0.245 m LiDAR/body slice.

    Includes are not contact truth (pigs); this is a static geometry proxy only.
    Refuse rotated explicit geometry instead of silently mis-scoring it.
    """
    boxes, circles = [], []
    def xyz(element):
        p = np.fromstring(element.findtext("pose", "0 0 0 0 0 0"), sep=" ")
        if len(p) != 6 or np.any(p[3:]):
            raise ValueError("Unsupported map pose in static collision scoring")
        return p[:3]
    for model in ET.parse(WORLD_FILE).findall("./world/model"):
        for link in model.findall("link"):
            for col in link.findall("collision"):
                p = xyz(model)+xyz(link)+xyz(col)
                box, cylinder = col.find("geometry/box"), col.find("geometry/cylinder")
                if box is not None:
                    size = np.fromstring(box.findtext("size"), sep=" ")
                    if abs(p[2]-.245) <= size[2]/2:
                        boxes.append((*p[:2], *(size[:2]/2)))
                elif cylinder is not None:
                    if abs(p[2]-.245) <= float(cylinder.findtext("length"))/2:
                        circles.append((*p[:2], float(cylinder.findtext("radius"))))
                elif col.find("geometry/plane") is None:
                    raise ValueError("Unsupported explicit collision shape")
    return boxes, circles


def collision(pose, half_length=.23, half_width=.21):
    """Static original-map collision proxy, not a Gazebo contact sensor."""
    x, y, yaw = pose
    u = np.array([math.cos(yaw), math.sin(yaw)])
    v = np.array([-u[1], u[0]])
    boxes, circles = static_geometry()
    for wx, wy, hx, hy in boxes:
        delta = np.array([wx-x, wy-y])
        if all(abs(np.dot(delta, a)) <= half_length*abs(np.dot(u, a)) +
               half_width*abs(np.dot(v, a)) + hx*abs(a[0]) + hy*abs(a[1])
               for a in (u, v, np.array([1., 0.]), np.array([0., 1.]))):
            return True
    for cx, cy, radius in circles:
        delta = np.array([cx-x, cy-y])
        dx = max(abs(np.dot(delta, u))-half_length, 0.)
        dy = max(abs(np.dot(delta, v))-half_width, 0.)
        if dx*dx+dy*dy <= radius*radius:
            return True
    return False


def distance(pose, task):
    return float(np.linalg.norm(np.asarray(pose[:2])-task.goal[:2]))


def goal_reached(pose, task):
    return distance(pose, task) < .25 and abs(angle_error(pose[2], task.goal[2])) < math.radians(10)
