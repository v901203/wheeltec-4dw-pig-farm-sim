"""
pig_pen_env.py — Gymnasium 環境，對接 Gazebo Fortress 豬舍模擬場景

觀察空間（39 維 float32）：
  [0:36]  36 條 LiDAR 射線（從 /scan 均勻取樣），正規化到 [0, 1]
  [36]    到下一個路徑點的距離 / 30（正規化）
  [37]    到下一個路徑點的方向角 sin 值（機器人座標系）
  [38]    到下一個路徑點的方向角 cos 值（機器人座標系）

動作空間（2 維 float32）：
  [0]  線速度 ∈ [0,  0.5] m/s
  [1]  角速度 ∈ [-1.5, 1.5] rad/s

獎勵：
  +5 × 本步靠近路徑點的距離（公尺）
  +50  到達一個路徑點（距離 < REACH_DIST）
  +200 完成整條巡邏路線
  -100 碰撞（任一 LiDAR 射線 < COLL_DIST）
  -0.01 每步的時間懲罰
"""

import math
import json
import time
import threading
import subprocess
from pathlib import Path

import numpy as np
import gymnasium as gym

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# ── 環境參數（可依現場調整） ────────────────────────────────────────────
N_LIDAR    = 36       # 取樣的 LiDAR 射線數量
MAX_RANGE  = 10.0     # LiDAR 最大量測距離 (m)，超過就截斷
MAX_LIN    = 0.5      # 最大線速度 (m/s)
MAX_ANG    = 1.5      # 最大角速度 (rad/s)
REACH_DIST = 0.8      # 距路徑點此距離內算「到達」(m)
COLL_DIST  = 0.35     # LiDAR 射線小於此值視為碰撞 (m)
MAX_STEPS  = 3000     # 每個 episode 最多步數

WORLD_NAME = "pig_pen_16units_world"   # 對應 pig_pen_16units.world 的 world name
MODEL_NAME = "wheeltec_mini"           # 對應 launch_clean.sh 裡的 MODEL


# ── 內部 ROS2 節點 ──────────────────────────────────────────────────────
class _RosNode(Node):
    """只負責收發資料的輕量節點，由獨立 thread 執行 spin。"""

    def __init__(self):
        super().__init__("pig_pen_rl_env_node")
        self._lock   = threading.Lock()
        self._scan: LaserScan | None = None
        self._odom: Odometry  | None = None

        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Odometry,  "/odom", self._odom_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

    def get_data(self):
        with self._lock:
            return self._scan, self._odom

    def publish_cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x  = float(vx)
        t.angular.z = float(wz)
        self._cmd_pub.publish(t)


# ── 工具函式 ────────────────────────────────────────────────────────────
def _teleport(x: float, y: float, yaw: float = 0.0):
    """
    透過 ign service 把模型 teleport 到指定世界座標。
    適用於 Gazebo Fortress（ign gazebo 6.x）。
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = (
        f'name: "{MODEL_NAME}" '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: 0.05}} '
        f'orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}'
    )
    result = subprocess.run(
        [
            "ign", "service", "-s",
            f"/world/{WORLD_NAME}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req", req,
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"[WARN] teleport 失敗: {result.stderr.strip()}")


# ── Gymnasium 環境 ──────────────────────────────────────────────────────
class PigPenEnv(gym.Env):
    """
    Gymnasium-v0.26+ 相容環境。
    需要先執行 calibrate_waypoints.py 產生 waypoints.json，
    以及先啟動 launch_clean.sh。
    """

    metadata = {"render_modes": []}

    def __init__(self, waypoints_file: str = "waypoints.json"):
        super().__init__()

        # 讀取路徑點
        data = json.loads(Path(waypoints_file).read_text())
        self._start      = data["start"]   # {world_x, world_y, world_yaw}
        self._waypoints  = [(wp["x"], wp["y"]) for wp in data["patrol_waypoints"]]
        self._n_wp       = len(self._waypoints)

        # ── 觀察空間 ─────────────────────────────────────────────────────
        # [LiDAR × N_LIDAR] + [dist_norm, sin(angle), cos(angle)]
        obs_low  = np.zeros(N_LIDAR + 3, dtype=np.float32)
        obs_high = np.ones (N_LIDAR + 3, dtype=np.float32)
        obs_low [N_LIDAR + 1] = -1.0   # sin 可以是負數
        self.observation_space = gym.spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float32
        )

        # ── 動作空間 ─────────────────────────────────────────────────────
        self.action_space = gym.spaces.Box(
            low =np.array([0.0,    -MAX_ANG], dtype=np.float32),
            high=np.array([MAX_LIN, MAX_ANG], dtype=np.float32),
        )

        # ── ROS2 ─────────────────────────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()
        self._ros = _RosNode()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()

        # ── 回合狀態 ─────────────────────────────────────────────────────
        self._wp_idx    = 0
        self._prev_dist = 0.0
        self._steps     = 0

    # ── 內部工具 ─────────────────────────────────────────────────────────

    def _wait_sensor(self, timeout: float = 8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            scan, odom = self._ros.get_data()
            if scan is not None and odom is not None:
                return
            time.sleep(0.05)
        raise RuntimeError("等待 /scan 或 /odom 超時，請確認 launch_clean.sh 已啟動。")

    def _robot_pose(self):
        """回傳機器人在 odom 座標系的 (x, y, yaw)。"""
        _, odom = self._ros.get_data()
        p   = odom.pose.pose.position
        q   = odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )
        return p.x, p.y, yaw

    def _dist_and_bearing(self):
        """到當前目標路徑點的距離與相對方位角（機器人座標系）。"""
        rx, ry, ryaw = self._robot_pose()
        wx, wy = self._waypoints[self._wp_idx]
        dx, dy = wx - rx, wy - ry
        dist   = math.hypot(dx, dy)
        angle  = math.atan2(dy, dx) - ryaw
        # 正規化到 [-π, π]
        angle  = math.atan2(math.sin(angle), math.cos(angle))
        return dist, angle

    def _lidar_obs(self) -> np.ndarray:
        """取樣 N_LIDAR 條射線，正規化到 [0, 1]。"""
        scan, _ = self._ros.get_data()
        arr = np.asarray(scan.ranges, dtype=np.float32)
        arr = np.nan_to_num(arr, nan=MAX_RANGE, posinf=MAX_RANGE, neginf=0.0)
        arr = np.clip(arr, 0.0, MAX_RANGE)
        idx = np.round(np.linspace(0, len(arr) - 1, N_LIDAR)).astype(int)
        return arr[idx] / MAX_RANGE

    def _build_obs(self) -> np.ndarray:
        lidar       = self._lidar_obs()
        dist, angle = self._dist_and_bearing()
        extra = np.array(
            [min(dist / 30.0, 1.0), math.sin(angle), math.cos(angle)],
            dtype=np.float32,
        )
        return np.concatenate([lidar, extra])

    def _is_collision(self) -> bool:
        scan, _ = self._ros.get_data()
        arr = np.asarray(scan.ranges, dtype=np.float32)
        arr = np.nan_to_num(arr, nan=MAX_RANGE, posinf=MAX_RANGE, neginf=MAX_RANGE)
        return bool(np.any(arr < COLL_DIST))

    # ── Gymnasium API ─────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # 停車 → teleport 回起點 → 等穩定
        self._ros.publish_cmd(0.0, 0.0)
        _teleport(
            self._start["world_x"],
            self._start["world_y"],
            self._start["world_yaw"],
        )
        time.sleep(0.8)   # 等 Gazebo physics 穩定

        self._wait_sensor()
        self._wp_idx    = 0
        self._steps     = 0
        self._prev_dist, _ = self._dist_and_bearing()

        return self._build_obs(), {}

    def step(self, action):
        # 執行動作
        vx = float(np.clip(action[0], 0.0,    MAX_LIN))
        wz = float(np.clip(action[1], -MAX_ANG, MAX_ANG))
        self._ros.publish_cmd(vx, wz)
        time.sleep(0.1)   # 10 Hz 控制頻率

        self._steps += 1
        obs           = self._build_obs()
        curr_dist, _  = self._dist_and_bearing()
        terminated    = False
        truncated     = False
        info: dict    = {}

        # ── 獎勵計算 ─────────────────────────────────────────────────────
        # 1. 靠近路徑點的進度獎勵
        reward = (self._prev_dist - curr_dist) * 5.0
        self._prev_dist = curr_dist

        # 2. 到達路徑點
        if curr_dist < REACH_DIST:
            reward += 50.0
            self._wp_idx += 1
            info["waypoints_done"] = self._wp_idx

            if self._wp_idx >= self._n_wp:
                # 完成整條巡邏路線
                reward    += 200.0
                terminated = True
                info["success"] = True
            else:
                # 更新下一個路徑點的距離基準
                self._prev_dist, _ = self._dist_and_bearing()

        # 3. 碰撞
        if not terminated and self._is_collision():
            reward    -= 100.0
            terminated = True
            info["collision"] = True

        # 4. 時間懲罰
        reward -= 0.01

        # 5. 達到最大步數
        if self._steps >= MAX_STEPS:
            truncated = True

        return obs, float(reward), terminated, truncated, info

    def close(self):
        self._ros.publish_cmd(0.0, 0.0)
        self._ros.destroy_node()
