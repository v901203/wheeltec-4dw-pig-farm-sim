"""Fit separate physical lines before estimating a corridor axis.

All coordinates are in the LiDAR/base frame. No odometry, map or commanded
motion is used. A corridor requires two parallel lines on opposite sides of
the sensor; PCA over disconnected walls is deliberately never used.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class WallLine:
    angle: float
    normal: np.ndarray
    offset: float
    span: float
    residual: float
    count: int


def axis_angle(angle):
    """Undirected line orientation in [-pi/2, pi/2]."""
    return math.atan2(math.sin(2 * angle), math.cos(2 * angle)) / 2


def wall_lines(ranges, angles, radius, *, max_lines=6):
    valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < radius)
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                              ranges[valid] * np.sin(angles[valid])))
    # Bound per-scan cost independently of the sensor's angular resolution.
    if len(points) > 360:
        points = points[np.linspace(0, len(points)-1, 360).astype(int)]
    rng = np.random.default_rng(0)  # Repeatable without changing PPO's RNG.
    lines = []
    tolerance = 0.045
    for _ in range(max_lines):
        if len(points) < 8:
            break
        pairs = rng.integers(0, len(points), (128, 2))
        origins = points[pairs[:, 0]]
        vectors = points[pairs[:, 1]] - origins
        lengths = np.linalg.norm(vectors, axis=1)
        usable = lengths > 0.30
        if not np.any(usable):
            break
        vectors = vectors[usable] / lengths[usable, None]
        origins = origins[usable]
        normals = np.column_stack((-vectors[:, 1], vectors[:, 0]))
        offsets = np.sum(origins * normals, axis=1)
        residuals = np.abs(points @ normals.T - offsets)
        support = residuals <= tolerance
        best = int(np.argmax(np.sum(support, axis=0)))
        inliers = support[:, best]
        if np.count_nonzero(inliers) < 8:
            break
        # Refine only the consensus on ONE wall, never a whole angular sector.
        for _ in range(2):
            selected = points[inliers]
            centre = selected.mean(axis=0)
            _, _, axes = np.linalg.svd(selected - centre, full_matrices=False)
            direction = axes[0]
            normal = np.array([-direction[1], direction[0]])
            offset = float(centre @ normal)
            refined = np.abs(points @ normal - offset) <= tolerance
            if np.count_nonzero(refined) < 8:
                break
            inliers = refined
        selected = points[inliers]
        span = float(np.ptp(selected @ direction))
        residual = float(np.sqrt(np.mean((selected @ normal - offset)**2)))
        if span >= 0.30 and residual <= 0.035:
            angle = axis_angle(math.atan2(direction[1], direction[0]))
            lines.append(WallLine(angle, normal, offset, span, residual, len(selected)))
        points = points[~inliers]
    return lines


def corridor_geometry(ranges, angles, cfg):
    """Return (axis error in radians, confidence, signed wall-distance balance).

    Unpaired walls and competing perpendicular corridor axes are ambiguous,
    hence return confidence zero. A perpendicular *single corridor* still has
    two parallel walls and correctly returns an error close to +/-90 degrees.
    """
    lines = wall_lines(ranges, angles, max(2.5, 2 * cfg.open_distance))
    candidates = []
    for i, left in enumerate(lines):
        for right in lines[i+1:]:
            difference = abs(axis_angle(left.angle - right.angle))
            if difference > math.radians(8):
                continue
            a = left.offset
            b = right.offset * (1 if left.normal @ right.normal >= 0 else -1)
            width = abs(a - b)
            if (a*b >= 0 or min(abs(a), abs(b)) < cfg.half_width or
                    max(abs(a), abs(b)) > cfg.open_distance or
                    not 2*cfg.half_width < width < 2*cfg.open_distance):
                continue
            angle = math.atan2(math.sin(2*left.angle) + math.sin(2*right.angle),
                               math.cos(2*left.angle) + math.cos(2*right.angle)) / 2
            residual_quality = max(0.0, 1 - max(left.residual, right.residual)/0.06)
            confidence = (min(1.0, min(left.span, right.span)/0.8) *
                          residual_quality * math.cos(difference))
            # Quality and extent select the physically supported pair, not the
            # pair whose direction happens to be closest to the vehicle's x-axis.
            score = confidence * min(left.count, right.count)
            normal = np.array([-math.sin(angle), math.cos(angle)])
            da = left.offset * (1 if left.normal @ normal >= 0 else -1)
            db = right.offset * (1 if right.normal @ normal >= 0 else -1)
            candidates.append((score, angle, confidence, da+db))
    if not candidates:
        return 0.0, 0.0, 0.0
    candidates.sort(reverse=True)
    best = candidates[0]
    if any(item[0] >= 0.65*best[0] and
           abs(axis_angle(item[1]-best[1])) > math.radians(25)
           for item in candidates[1:]):
        return 0.0, 0.0, 0.0
    return float(best[1]), float(best[2]), float(best[3])
