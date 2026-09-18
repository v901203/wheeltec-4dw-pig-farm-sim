"""ROS-independent perception, reward and command safety for the 38-D policy."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

CONFIG_FILE = Path(__file__).with_name("patrol_config.json")
N_LIDAR = 36
MAX_RANGE = 10.0
MAX_LIN = 1.0
MAX_ANG = 2.0


@dataclass(frozen=True)
class Config:
    world_name: str = "pig_pen_16units_world"
    model_name: str = "wheeltec_mini"
    start: tuple = (0.0, -14.0, math.pi / 2)
    junctions: int = 3
    lidar_yaw: float = 0.0  # laser_link is at base x=y=0 in the supplied URDF
    half_length: float = 0.23  # includes wheels
    half_width: float = 0.21
    collision_margin: float = 0.02
    stop_margin: float = 0.18
    brake_deceleration: float = 0.8
    control_dt: float = 1.0 / 12.0
    sensor_timeout: float = 3.0  # wall seconds, including a paused simulator
    reset_attempts: int = 5
    reset_validation_frames: int = 24
    open_distance: float = 1.5
    wall_distance: float = 1.0
    confirm_frames: int = 3
    cruise_speed: float = 0.20
    turn_speed: float = 0.60
    yaw_tolerance: float = 0.06
    junction_advance: float = 0.32
    min_main_run: float = 2.5
    min_branch_run: float = 2.5
    max_branch_run: float = 6.0
    max_main_run: float = 10.0
    return_takeover: float = 1.1
    return_tolerance: float = 0.10
    entry_distance: float = 0.9
    stuck_seconds: float = 10.0
    state_timeout: float = 90.0
    train_max_steps: int = 600
    patrol_max_steps: int = 12000


def load_config(path=CONFIG_FILE):
    data = json.loads(Path(path).read_text())
    cfg = Config(**data)
    positive = ("half_length", "half_width", "control_dt", "sensor_timeout",
                "open_distance", "wall_distance", "brake_deceleration",
                "cruise_speed", "turn_speed", "yaw_tolerance", "entry_distance",
                "min_main_run", "min_branch_run", "max_branch_run", "max_main_run",
                "return_takeover", "return_tolerance", "stuck_seconds", "state_timeout")
    if any(not math.isfinite(getattr(cfg, key)) or getattr(cfg, key) <= 0 for key in positive):
        raise ValueError("Patrol distances, speeds and times must be finite and positive")
    if any(type(getattr(cfg, key)) is not int or getattr(cfg, key) < 1 for key in
           ("junctions", "confirm_frames", "train_max_steps", "patrol_max_steps",
            "reset_attempts", "reset_validation_frames")):
        raise ValueError("Patrol counts must be positive integers")
    if cfg.reset_validation_frames < cfg.confirm_frames:
        raise ValueError("reset_validation_frames must be at least confirm_frames")
    if len(cfg.start) != 3 or not all(math.isfinite(v) for v in cfg.start):
        raise ValueError("start must contain finite [world_x, world_y, world_yaw]")
    if not (0 < cfg.wall_distance < cfg.open_distance <= MAX_RANGE):
        raise ValueError("Require wall_distance < open_distance <= 10 m")
    if cfg.min_branch_run >= cfg.max_branch_run or cfg.min_main_run >= cfg.max_main_run:
        raise ValueError("Minimum travel must be less than maximum travel")
    if cfg.cruise_speed > MAX_LIN or cfg.turn_speed > MAX_ANG:
        raise ValueError("FSM speed exceeds action limits")
    if any(not math.isfinite(v) or v < 0 for v in
           (cfg.collision_margin, cfg.stop_margin, cfg.junction_advance)):
        raise ValueError("Margins and junction advance must be finite and nonnegative")
    if not math.isfinite(cfg.lidar_yaw):
        raise ValueError("lidar_yaw must be finite")
    return cfg


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SensorFault(RuntimeError):
    pass


@dataclass
class ScanFeatures:
    observation: np.ndarray
    ranges: np.ndarray
    angles: np.ndarray
    balance_m: float
    front_m: float
    left_edge_m: float
    right_edge_m: float

    def both_open(self, cfg):
        return min(self.left_edge_m, self.right_edge_m) > cfg.open_distance

    def walls_present(self, cfg):
        return max(self.left_edge_m, self.right_edge_m) < cfg.wall_distance


def scan_features(scan, cfg):
    """Use scan metadata, all rays for sectors, and 36 original-order samples.

    +inf is a no-return ray. NaN, -inf and out-of-spec finite values are
    unknown, never silently interpreted as free space. Excess unknown data
    stops the robot. Remaining unknown rays are conservatively represented.
    """
    raw = np.asarray(scan.ranges, dtype=np.float64)
    if (raw.ndim != 1 or raw.size < N_LIDAR or
            not math.isfinite(scan.angle_increment) or scan.angle_increment == 0 or
            not math.isfinite(scan.angle_min) or
            not 0 <= scan.range_min < scan.range_max):
        raise SensorFault("Invalid LaserScan metadata or too few rays")
    valid = np.isfinite(raw) & (raw >= scan.range_min) & (raw <= scan.range_max)
    known = valid | np.isposinf(raw)
    if np.mean(known) < 0.95:
        raise SensorFault("More than 5% of LiDAR rays are invalid")
    arr = np.where(valid, raw, np.where(np.isposinf(raw), MAX_RANGE, 0.0))
    arr = np.clip(arr, 0.0, MAX_RANGE)
    angles = scan.angle_min + np.arange(raw.size) * scan.angle_increment + cfg.lidar_yaw
    angles = np.arctan2(np.sin(angles), np.cos(angles))

    def sector(low, high):
        mask = (angles >= math.radians(low) - 1e-7) & (angles <= math.radians(high) + 1e-7)
        if not np.any(mask) or np.mean(known[mask]) < 0.95:
            raise SensorFault(f"Missing or invalid LiDAR sector {low}..{high}")
        return arr[mask]

    balance = float(np.mean(sector(60, 120)) - np.mean(sector(-120, -60)))
    front = float(np.min(sector(-30, 30)))
    idx = np.round(np.linspace(0, arr.size - 1, N_LIDAR)).astype(int)
    observation = np.concatenate((arr[idx] / MAX_RANGE, [balance / MAX_RANGE, front / MAX_RANGE]))
    # Narrow sectors and a lower quantile resist rays passing through rail gaps.
    # Real rail scans contain only ~13-18% near-wall returns in these sectors.
    left = float(np.quantile(sector(80, 100), 0.10))
    right = float(np.quantile(sector(-100, -80), 0.10))
    return ScanFeatures(observation.astype(np.float32), arr, angles, balance, front, left, right)


def collision_detected(features, cfg):
    """Conservative 2-D footprint intrusion proxy, not a contact sensor."""
    x = features.ranges * np.cos(features.angles)
    y = features.ranges * np.sin(features.angles)
    return bool(np.any((np.abs(x) < cfg.half_length + cfg.collision_margin) &
                       (np.abs(y) < cfg.half_width + cfg.collision_margin)))


def safe_command(action, features, cfg, measured_speed=0.0):
    action = np.asarray(action, dtype=float)
    if action.shape != (2,) or not np.all(np.isfinite(action)):
        raise ValueError("Action must contain two finite values: [linear, angular]")
    vx, wz = np.clip(action, [0.0, -MAX_ANG], [MAX_LIN, MAX_ANG])
    x = features.ranges * np.cos(features.angles)
    y = features.ranges * np.sin(features.angles)
    speed = max(vx, abs(measured_speed))
    braking = speed * cfg.control_dt + speed ** 2 / (2 * cfg.brake_deceleration)
    blocked = bool(np.any((x > 0) & (x < cfg.half_length + cfg.stop_margin + braking) &
                          (np.abs(y) < cfg.half_width + cfg.stop_margin)))
    turning_blocked = bool(np.min(features.ranges) <
                            math.hypot(cfg.half_length, cfg.half_width) + cfg.collision_margin)
    stopped = collision_detected(features, cfg) or (vx > 0 and blocked) or (abs(wz) > 0 and turning_blocked)
    return (np.zeros(2) if stopped else np.array([vx, wz])), stopped


def local_reward(speed, balance_m, collision=False, corridor=True):
    if collision:
        return -100.0
    return 2.0 * float(speed) - (abs(float(balance_m)) if corridor else 0.0) - 0.05
