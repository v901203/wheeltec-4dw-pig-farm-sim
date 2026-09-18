"""Local LiDAR junction geometry and feature-to-feature motion tracking.

No map, chassis odometry, IMU, command integration or assumed speed. This is
not general-purpose SLAM: it requires four supported walls around an open
crossing and rejects missing/ambiguous geometry.
"""

from dataclasses import dataclass
import math

import numpy as np

from lidar_geometry import axis_angle, wall_lines


@dataclass(frozen=True)
class JunctionGeometry:
    centre: np.ndarray  # Intersection of corridor centre lines, in base frame.
    axes: tuple        # Two undirected corridor axes, radians in base frame.
    confidence: float


def junction_geometry(features, cfg):
    """Find two perpendicular wall pairs with actual gaps on all four walls.

    Merely fitting a rectangle also describes a closed room. Support on BOTH
    sides of each opening, and absence of a wall across its middle, distinguish
    a crossing. Rail gaps alone cannot form metre-wide four-way openings.
    """
    radius = cfg.junction_fit_radius
    lines = wall_lines(features.ranges, features.angles, radius, max_lines=8)
    valid = (features.ranges > .05) & (features.ranges < radius)
    points = np.column_stack((features.ranges[valid]*np.cos(features.angles[valid]),
                              features.ranges[valid]*np.sin(features.angles[valid])))
    pairs = []
    for i, a in enumerate(lines):
        for j in range(i+1, len(lines)):
            b = lines[j]
            if abs(axis_angle(a.angle-b.angle)) > math.radians(8):
                continue
            angle = a.angle + axis_angle(b.angle-a.angle)/2
            normal = np.array([-math.sin(angle), math.cos(angle)])
            da = a.offset * (1 if a.normal @ normal >= 0 else -1)
            db = b.offset * (1 if b.normal @ normal >= 0 else -1)
            width = abs(da-db)
            if not 2*(cfg.half_width+cfg.collision_margin) < width < 2*cfg.open_distance:
                continue
            pairs.append((i, j, angle, normal, (da+db)/2, width))
    candidates = []
    for index, a in enumerate(pairs):
        for b in pairs[index+1:]:
            if len({a[0], a[1], b[0], b[1]}) != 4:
                continue
            if abs(abs(axis_angle(a[2]-b[2]))-math.pi/2) > math.radians(8):
                continue
            centre = np.linalg.solve(np.stack((a[3], b[3])), [a[4], b[4]])
            if np.linalg.norm(centre) > cfg.junction_acquire_distance:
                continue
            confidence, support = 1.0, 0
            for pair, cross in ((a, b), (b, a)):
                direction = np.array([math.cos(pair[2]), math.sin(pair[2])])
                for line_index in pair[:2]:
                    line = lines[line_index]
                    wall_points = points[np.abs(points @ line.normal-line.offset) < .06]
                    along = (wall_points-centre) @ direction
                    half_gap = cross[5]/2
                    before = np.count_nonzero(along < -half_gap-.04)
                    after = np.count_nonzero(along > half_gap+.04)
                    inside = np.count_nonzero(np.abs(along) < half_gap-.10)
                    if min(before, after) < 3 or inside > max(2, .10*len(along)):
                        confidence = 0.0
                        break
                    support += min(before, after)
                    confidence = min(confidence, max(0.0, 1-line.residual/.06),
                                     min(1.0, line.span/1.0))
                if confidence == 0:
                    break
            if confidence >= cfg.junction_min_confidence:
                candidates.append((confidence*support, JunctionGeometry(
                    centre, (axis_angle(a[2]), axis_angle(b[2])), confidence)))
    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0], reverse=True)
    score, best = candidates[0]
    if any(value >= .65*score and np.linalg.norm(other.centre-best.centre) > .20
           for value, other in candidates[1:]):
        return None
    return best


class JunctionTracker:
    """Match crossing axes between fresh scans without swapping route identity.

    A cross is 90-degree symmetric. Continuity is required: updates with large
    gaps or rotations are rejected rather than choosing a new nearest axis.
    The tracked incoming axis is directed and unwrapped across the whole turn.
    """
    def __init__(self, geometry, stamp, cfg, heading_hint=0.0):
        self.cfg = cfg
        self.geometry = geometry
        self.stamp = float(stamp)
        self.incoming_axis = min((heading_hint+axis_angle(a-heading_hint) for a in geometry.axes),
                                 key=lambda a: abs(a-heading_hint))
        self.delta_pose = np.zeros(3)
        self.failure = None

    def update(self, geometry, stamp):
        dt = float(stamp)-self.stamp
        if dt <= 0 or dt > self.cfg.junction_tracking_max_gap:
            self.failure = "junction_scan_gap"
            return False
        candidates = [self.incoming_axis+axis_angle(a-self.incoming_axis) for a in geometry.axes]
        incoming = min(candidates, key=lambda a: abs(a-self.incoming_axis))
        delta_yaw = self.incoming_axis-incoming
        if abs(delta_yaw) > self.cfg.junction_tracking_max_angle:
            self.failure = "junction_axis_jump"
            return False
        c, s = math.cos(delta_yaw), math.sin(delta_yaw)
        rotation = np.array([[c, -s], [s, c]])
        translation = self.geometry.centre-rotation @ geometry.centre
        if np.linalg.norm(translation) > self.cfg.junction_tracking_max_translation:
            self.failure = "junction_position_jump"
            return False
        self.delta_pose = np.array([*translation, delta_yaw])
        self.incoming_axis = incoming
        self.geometry, self.stamp = geometry, float(stamp)
        self.failure = None
        return True
