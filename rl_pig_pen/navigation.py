"""ROS-independent LiDAR perception, training rewards and command safety."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

from lidar_geometry import corridor_geometry

CONFIG_FILE = Path(__file__).with_name("patrol_config.json")
N_LIDAR = 36
MAX_RANGE = 10.0
MAX_LIN = 0.5
MAX_ANG = 0.6
TIME_PENALTY_RATE = 0.6
# Reward per metre of measured displacement along the travel direction.
PROGRESS_REWARD_RATE = 6.0
PROGRESS_CLIP_FACTOR = 1.5
MIN_WALL_CONFIDENCE = 0.5
EVENT_ALIGNMENT_TOLERANCE = math.radians(20)
ALIGNMENT_PENALTY_RATE = 8.0
CENTRING_PENALTY_RATE = 0.8

# === 新增：旋轉與倒退懲罰權重 ===
SPIN_PENALTY_RATE = 2.5      # 旋轉扣分強度，抑制原地轉圈
REVERSE_PENALTY_RATE = 4.0   # 主幹道倒退扣分強度


@dataclass(frozen=True)
class Config:
    world_name: str = "pig_pen_16units_world"
    model_name: str = "wheeltec_mini"
    start: tuple = (0.0, -11.3, math.pi / 2)
    junctions: int = 3
    lidar_yaw: float = 0.0  # laser_link is at base x=y=0 in the supplied URDF
    half_length: float = 0.23  # includes wheels
    half_width: float = 0.21
    collision_margin: float = 0.02
    stop_margin: float = 0.18
    brake_deceleration: float = 1.5
    control_dt: float = 1.0 / 12.0
    sensor_timeout: float = 3.0  # wall seconds, including a paused simulator
    reset_attempts: int = 5
    reset_validation_frames: int = 24
    open_distance: float = 1.5
    wall_distance: float = 1.0
    end_distance: float = 1.2
    confirm_frames: int = 3
    cruise_speed: float = 0.20
    turn_speed: float = 0.60
    yaw_tolerance: float = 0.06
    junction_advance: float = 0.32
    junction_fit_radius: float = 3.0
    junction_acquire_distance: float = 1.5
    junction_centre_tolerance: float = 0.05
    junction_turn_centre_limit: float = 0.15
    junction_min_confidence: float = 0.65
    junction_tracking_max_gap: float = 0.5
    junction_tracking_max_angle: float = math.radians(20)
    junction_tracking_max_translation: float = 0.25
    junction_geometry_timeout: float = 2.0
    turn_kp: float = 1.5
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
                "open_distance", "wall_distance", "end_distance", "brake_deceleration",
                "cruise_speed", "turn_speed", "yaw_tolerance", "entry_distance",
                "min_main_run", "min_branch_run", "max_branch_run", "max_main_run",
                "return_takeover", "return_tolerance", "stuck_seconds", "state_timeout",
                "junction_fit_radius", "junction_acquire_distance", "junction_centre_tolerance",
                "junction_turn_centre_limit",
                "junction_min_confidence", "junction_tracking_max_gap", "junction_tracking_max_angle",
                "junction_tracking_max_translation", "junction_geometry_timeout", "turn_kp")
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
    if not (0 < cfg.junction_min_confidence <= 1 and
            cfg.junction_tracking_max_angle < math.pi/4 and
            cfg.junction_centre_tolerance < cfg.junction_turn_centre_limit and
            cfg.junction_centre_tolerance < cfg.junction_acquire_distance < cfg.junction_fit_radius):
        raise ValueError("Invalid junction confidence, tracking angle or geometry distances")
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
    front_wall_m: float
    rear_wall_m: float
    left_edge_m: float
    right_edge_m: float
    wall_parallel_error: float
    wall_alignment_confidence: float
    wall_balance_m: float = 0.0

    def aligned(self):
        return (self.wall_alignment_confidence >= MIN_WALL_CONFIDENCE and
                abs(self.wall_parallel_error) <= EVENT_ALIGNMENT_TOLERANCE)

    def junction_open(self, cfg, *, both=True):
        lateral = (min if both else max)(self.left_edge_m, self.right_edge_m)
        misaligned = (self.wall_alignment_confidence >= MIN_WALL_CONFIDENCE and
                      abs(self.wall_parallel_error) > EVENT_ALIGNMENT_TOLERANCE)
        return (lateral > cfg.open_distance and not misaligned and
                min(self.front_wall_m, self.rear_wall_m) > cfg.end_distance)

    def both_open(self, cfg):
        return min(self.left_edge_m, self.right_edge_m) > cfg.open_distance

    def walls_present(self, cfg):
        return max(self.left_edge_m, self.right_edge_m) < cfg.wall_distance

    def end_wall_present(self, cfg, reverse=False):
        if not (self.walls_present(cfg) and self.aligned()):
            return False
        target = self.wall_parallel_error + (math.pi if reverse else 0.0)
        delta = np.arctan2(np.sin(self.angles-target), np.cos(self.angles-target))
        rays = self.ranges[np.abs(delta) <= math.radians(15)]
        return bool(rays.size and np.quantile(rays, 0.25) < cfg.end_distance)


def scan_features(scan, cfg):
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
    front_sector = sector(-10, 10)
    front = float(np.min(front_sector))
    front_wall = float(np.quantile(front_sector, 0.25))
    rear_sector = np.concatenate((sector(-180, -150), sector(150, 180)))
    rear_wall = float(np.quantile(rear_sector, 0.25))
    idx = np.round(np.linspace(0, arr.size - 1, N_LIDAR)).astype(int)
    observation = np.concatenate((arr[idx] / MAX_RANGE, [balance / MAX_RANGE, front / MAX_RANGE]))
    left = float(np.quantile(sector(80, 100), 0.10))
    right = float(np.quantile(sector(-100, -80), 0.10))
    wall_error, wall_confidence, wall_balance = corridor_geometry(arr, angles, cfg)
    return ScanFeatures(observation.astype(np.float32), arr, angles, balance, front,
                        front_wall, rear_wall, left, right, wall_error, wall_confidence, wall_balance)


def wall_alignment(ranges, angles, cfg):
    return corridor_geometry(ranges, angles, cfg)[:2]


def collision_detected(features, cfg):
    x = features.ranges * np.cos(features.angles)
    y = features.ranges * np.sin(features.angles)
    return bool(np.any((np.abs(x) < cfg.half_length + cfg.collision_margin) &
                       (np.abs(y) < cfg.half_width + cfg.collision_margin)))


def safe_command(action, features, cfg, measured_speed=0.0, allow_reverse=False,
                 check_turn_clearance=True, enable_safety=True):
    """根據雷達障礙物距離評估動作安全性並輸出安全控制量。"""
    action = np.asarray(action, dtype=float)
    if action.shape != (2,) or not np.all(np.isfinite(action)):
        raise ValueError("Action must contain two finite values: [linear, angular]")

    lower_vx = -MAX_LIN if allow_reverse else 0.0
    vx, wz = np.clip(action, [lower_vx, -MAX_ANG], [MAX_LIN, MAX_ANG])

    # 訓練階段如果關閉安全限制，直接放行
    if not enable_safety:
        return np.array([vx, wz]), False

    x = features.ranges * np.cos(features.angles)
    y = features.ranges * np.sin(features.angles)
    speed = max(abs(vx), abs(measured_speed))
    braking = speed * cfg.control_dt + speed ** 2 / (2 * cfg.brake_deceleration)
    direction = 1.0 if vx >= 0 else -1.0
    longitudinal = direction * x

    # 橫向只取車寬 + 3cm (0.21 + 0.03 = 0.24m)，防止窄走道兩側欄杆誤觸前方煞停
    lateral_limit = cfg.half_width + 0.03
    blocked = bool(np.any((longitudinal > 0) &
                          (longitudinal < cfg.half_length + cfg.stop_margin + braking) &
                          (np.abs(y) < lateral_limit)))

    turning_blocked = (check_turn_clearance and
                       bool(np.min(features.ranges) <
                            math.hypot(cfg.half_length, cfg.half_width) + cfg.collision_margin))

    stopped = (collision_detected(features, cfg) or (abs(vx) > 0 and blocked) or
               (abs(wz) > 0 and turning_blocked))

    return (np.zeros(2) if stopped else np.array([vx, wz])), stopped

def travelled_distance(start_pose, end_pose, direction=1.0, actual_dt=1.0 / 12.0):
    dx = float(end_pose[0]) - float(start_pose[0])
    dy = float(end_pose[1]) - float(start_pose[1])
    progress = dx * math.cos(float(start_pose[2])) + dy * math.sin(float(start_pose[2]))
    progress *= -1.0 if direction < 0 else 1.0
    limit = PROGRESS_CLIP_FACTOR * MAX_LIN * max(0.0, float(actual_dt))
    return float(np.clip(progress, -limit, limit))


def local_reward(progress_m, balance_m, collision=False, corridor=True, actual_dt=1.0 / 12.0,
                 commanded_vx=0.0, commanded_wz=0.0, wall_parallel_error=0.0):
    """只獎勵真實走道軸向位移，嚴懲空踩油門與原地發呆。"""
    if collision:
        return -100.0
    elapsed = max(0.0, float(actual_dt))

    # 1. 軸向平行因子
    parallel_cos = max(0.0, math.cos(float(wall_parallel_error)))

    # 2. 真實走道軸向進展獎勵 (只有車子真正往前跑才給分)
    # 算出行駛的真實線速度
    measured_vx = float(progress_m) / elapsed if elapsed > 0 else 0.0
    directed_progress_reward = PROGRESS_REWARD_RATE * float(progress_m) * parallel_cos

    # 3. 基礎時間與置中懲罰
    reward = (directed_progress_reward
              - (CENTRING_PENALTY_RATE * abs(float(balance_m)) * elapsed if corridor else 0.0)
              - TIME_PENALTY_RATE * elapsed)

    # 4. 速度巡航激勵：必須「真正跑出速度」才發放獎勵，徹底杜絕原地踩油門刷分
    target_vx = 0.25
    if measured_vx > 0.05:
        effective_vx = min(measured_vx, target_vx)
        reward += 4.0 * effective_vx * parallel_cos * elapsed
    else:
        # 原地發呆或卡死空轉：每秒罰 1.5 分，逼它必須動起來
        reward -= 1.5 * elapsed

    # 5. 角度線性懲罰 (解決 90 度梯度平坦問題)
    reward -= ALIGNMENT_PENALTY_RATE * abs(float(wall_parallel_error)) * elapsed

    # 6. 旋轉與倒車懲罰
    effective_wz = max(0.0, abs(commanded_wz) - 0.05)
    reward -= SPIN_PENALTY_RATE * effective_wz * elapsed

    if commanded_vx < 0.0:
        reward -= REVERSE_PENALTY_RATE * abs(commanded_vx) * elapsed

    return reward

def corridor_progress(start_pose, end_pose, features, *, reverse=False, actual_dt=1.0 / 12.0):
    if features.wall_alignment_confidence < MIN_WALL_CONFIDENCE:
        return 0.0
    corridor_pose = (start_pose[0], start_pose[1],
                     start_pose[2] + features.wall_parallel_error)
    return travelled_distance(corridor_pose, end_pose, -1.0 if reverse else 1.0, actual_dt)


def patrol_reward(progress_m, features, *, collision=False, actual_dt=1.0 / 12.0,
                  commanded_vx=0.0, commanded_wz=0.0):
    """Return full-route reward and its logged components with spin and reverse penalties."""
    elapsed = max(0.0, float(actual_dt))
    reliable = features.wall_alignment_confidence >= MIN_WALL_CONFIDENCE
    terms = {
        "reward_progress": PROGRESS_REWARD_RATE * float(progress_m),
        "reward_alignment": (-ALIGNMENT_PENALTY_RATE *
                             math.sin(features.wall_parallel_error)**2 * elapsed if reliable else 0.0),
        "reward_centring": (-CENTRING_PENALTY_RATE * abs(features.wall_balance_m) * elapsed
                            if reliable else 0.0),
        "reward_time": -TIME_PENALTY_RATE * elapsed,
        "reward_spin": -SPIN_PENALTY_RATE * abs(commanded_wz) * elapsed,
    }
    # 新增：當前方距離小於 0.8m（面壁）且偏離走道時，給予嚴重排斥懲罰
    if features.front_m < 0.8 and abs(features.wall_parallel_error) > math.radians(30):
        terms["reward_wall_stare"] = -10.0 * (0.8 - features.front_m) * elapsed

    if commanded_vx < 0.0:
        terms["reward_reverse"] = -REVERSE_PENALTY_RATE * abs(commanded_vx) * elapsed
    else:
        terms["reward_reverse"] = 0.0

    if collision:
        terms = dict.fromkeys(terms, 0.0)
        terms["reward_collision"] = -100.0
    return sum(terms.values()), terms