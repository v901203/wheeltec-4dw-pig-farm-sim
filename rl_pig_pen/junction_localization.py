"""Local LiDAR junction geometry and feature-to-feature motion tracking.

No map, chassis odometry, IMU, command integration or assumed speed. This is
not general-purpose SLAM: acquisition requires four supported walls around
an open crossing. An acquired crossing can be tracked with three measured
walls on two axes; missing/ambiguous geometry is rejected.
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
    widths: tuple = ()  # Per-axis wall separation, measured at acquisition.


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
                    centre, (axis_angle(a[2]), axis_angle(b[2])), confidence,
                    (a[5], b[5]))))
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
        self.update_diagnostics = {}

    def observe_walls(self, features):
        """Track an acquired crossing with >=3 walls on two independent axes.

        Opening validation remains mandatory for acquisition. After acquisition
        a temporarily hidden opening does not erase the measured road widths.
        Only current wall measurements constrain the centre; no velocity or
        commanded displacement is integrated. Missing both walls of either
        axis, ambiguous matches, or large changes return None.
        """
        for scale in (1.0, .8, .65):
            geometry = self._observe_walls(features, scale * self.cfg.junction_fit_radius)
            if geometry is not None:
                return geometry
        return None

    def _observe_walls(self, features, radius):
        previous = self.geometry
        if len(previous.widths) != 2:
            return None
        lines = wall_lines(features.ranges, features.angles, radius, max_lines=8)
        lines = [line for line in lines if line.span >= .5 and line.residual <= .03]
        proposals = [axis_angle(line.angle - axis) for line in lines for axis in previous.axes
                     if abs(axis_angle(line.angle - axis)) <= self.cfg.junction_tracking_max_angle]
        candidates = []
        translation_limit = min(.12, self.cfg.junction_tracking_max_translation)
        for delta in proposals:
            # Axis change is minus the robot's yaw change.
            c, s = math.cos(delta), math.sin(delta)
            predicted = np.array([[c, -s], [s, c]]) @ previous.centre
            matches = {}
            for group, (axis, width) in enumerate(zip(previous.axes, previous.widths)):
                expected_angle = axis + delta
                for index, line in enumerate(lines):
                    if abs(axis_angle(line.angle - expected_angle)) > math.radians(4):
                        continue
                    angle = expected_angle + axis_angle(line.angle - expected_angle)
                    normal = np.array([-math.sin(angle), math.cos(angle)])
                    offset = line.offset * (1 if line.normal @ normal >= 0 else -1)
                    relative = offset - float(predicted @ normal)
                    side = 1 if relative >= 0 else -1
                    error = abs(relative - side * width / 2)
                    if error > translation_limit:
                        continue
                    quality = 1 - line.residual / .06
                    score = quality * min(line.count, 40) / (1 + error / .04)
                    key = (group, side)
                    value = (score, normal, offset - side * width / 2,
                             axis_angle(angle - axis), quality, index)
                    if key not in matches or score > matches[key][0]:
                        matches[key] = value
            if len(matches) < 3 or len({v[5] for v in matches.values()}) != len(matches):
                continue
            values = list(matches.values())
            weights = np.array([v[4] for v in values])
            normals = np.stack([v[1] for v in values])
            offsets = np.array([v[2] for v in values])
            centre = np.linalg.lstsq(normals * weights[:, None], offsets * weights, rcond=None)[0]
            delta_fit = float(np.average([v[3] for v in values], weights=weights))
            c, s = math.cos(delta_fit), math.sin(delta_fit)
            predicted = np.array([[c, -s], [s, c]]) @ previous.centre
            residual = float(np.max(np.abs(normals @ centre - offsets)))
            if (residual > .04 or np.linalg.norm(centre - predicted) > translation_limit or
                    np.linalg.norm(centre) > self.cfg.junction_acquire_distance):
                continue
            geometry = JunctionGeometry(centre, tuple(axis_angle(a + delta_fit) for a in previous.axes),
                                        float(min(weights)), previous.widths)
            candidates.append((sum(v[0] for v in values), geometry))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        score, best = candidates[0]
        if any(value >= .9 * score and np.linalg.norm(other.centre - best.centre) > .06
               for value, other in candidates[1:]):
            return None
        return best

    def update(self, geometry, stamp):
        dt = float(stamp)-self.stamp
        self.update_diagnostics = {
            "previous_centre": self.geometry.centre.tolist(),
            "candidate_centre": geometry.centre.tolist(),
            "scan_gap_seconds": dt,
            "translation_limit_m": self.cfg.junction_tracking_max_translation,
            "rotation_limit_rad": self.cfg.junction_tracking_max_angle,
        }
        if dt <= 0 or dt > self.cfg.junction_tracking_max_gap:
            self.failure = "junction_scan_gap"
            return False
        candidates = [self.incoming_axis+axis_angle(a-self.incoming_axis) for a in geometry.axes]
        incoming = min(candidates, key=lambda a: abs(a-self.incoming_axis))
        delta_yaw = self.incoming_axis-incoming
        self.update_diagnostics["rotation_delta_rad"] = delta_yaw
        if abs(delta_yaw) > self.cfg.junction_tracking_max_angle:
            self.failure = "junction_axis_jump"
            return False
        c, s = math.cos(delta_yaw), math.sin(delta_yaw)
        rotation = np.array([[c, -s], [s, c]])
        translation = self.geometry.centre-rotation @ geometry.centre
        self.update_diagnostics["translation_delta_m"] = float(np.linalg.norm(translation))
        if np.linalg.norm(translation) > self.cfg.junction_tracking_max_translation:
            self.failure = "junction_position_jump"
            return False
        self.delta_pose = np.array([*translation, delta_yaw])
        self.incoming_axis = incoming
        self.geometry, self.stamp = geometry, float(stamp)
        self.failure = None
        return True
