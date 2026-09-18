"""Human-readable progress and reset reasons for corridor and patrol training."""

from collections import Counter, deque
import math
import time

from stable_baselines3.common.monitor import Monitor


REASONS = {
    "collision": "碰撞（LiDAR 車身範圍判定）",
    "stuck": "無有效位移或旋轉",
    "time_limit": "回合控制步數達上限",
    "state_timeout": "巡邏狀態逾時（不代表靜止）",
    "sensor_fault": "雷達或里程計資料逾時／無效",
    "segment_complete": "走道片段完成（抵達開口）",
    "success": "完整巡邏成功",
    "main_corridor_not_found": "未找到主幹道",
    "junction_or_main_end_not_found": "未找到下一個路口或主幹道末端",
    "branch_corridor_not_found": "未找到支線走道",
    "branch_end_not_found": "未找到支線末端",
    "return_junction_not_confirmed": "返回時未確認路口",
}


def episode_reason(info):
    if info.get("collision"):
        return "collision"
    if info.get("success"):
        return "success"
    if info.get("failure_reason"):
        return info["failure_reason"]
    if info.get("segment_complete"):
        return "segment_complete"
    return "unknown"


class TrainingMonitor(Monitor):
    def __init__(self, env, *args, worker_id=None, **kwargs):
        super().__init__(env, *args, **kwargs)
        self.prefix = f"[環境 {worker_id}] " if worker_id is not None else ""
        self.env.unwrapped.log_prefix = self.prefix
        self.training = False
        self.pending_reason = None
        self.progress_interval = 15.0
        self.phase = "啟動環境檢查"
        self.reason_counts = Counter()
        self.recent_rewards = deque(maxlen=100)
        self.completed_steps = 0
        self.completed_episodes = 0
        self.latest_info = {}
        # FSM maneuvers and reset approach can last many wall-clock seconds.
        # Report there too, without counting their frames as PPO decisions.
        self.env.unwrapped.progress_hook = self.heartbeat

    def _emit(self, message, **kwargs):
        print(self.prefix + message, **kwargs)

    def start_training(self, initial_steps, total_steps):
        self.training = True
        self.initial_steps = initial_steps
        self.target_steps = total_steps
        self.completed_steps = self.completed_episodes = 0
        self.pending_reason = None
        self.reason_counts.clear()
        self.recent_rewards.clear()
        self.started = self.last_report = time.monotonic()
        self._emit(f"[訓練開始] 本次目標 {total_steps:,} 次 PPO 決策；既有累積 {initial_steps:,} 步", flush=True)
        self._emit("[說明] FSM 轉向／掉頭不計入 PPO 步數；每 15 秒顯示進度，每回合列出重生原因。", flush=True)

    def heartbeat(self):
        if not self.training or time.monotonic()-self.last_report < self.progress_interval:
            return
        self.last_report = time.monotonic()
        elapsed = self.last_report-self.started
        fps = self.completed_steps/max(elapsed, 1e-9)
        mean = sum(self.recent_rewards)/len(self.recent_rewards) if self.recent_rewards else None
        info = self.latest_info
        raw = self.env.unwrapped
        if getattr(raw, "mode", None) == "patrol":
            info = raw.controller.info()
        route = (f" | 狀態 {info['state']} | 返回 {info['branches_done']}/{info['branches_total']}"
                 if "state" in info else "")
        mean_text = f"{mean:.2f}" if mean is not None else "尚無完整回合"
        self._emit(f"[訓練進度] {self.completed_steps:,}/{self.target_steps:,} "
              f"({100*self.completed_steps/self.target_steps:.2f}%) | 累積 {self.initial_steps+self.completed_steps:,} 步 "
              f"| {fps:.2f} 決策/秒 | 經過 {elapsed/60:.1f} 分鐘 | {self.phase}"
              f" | 最近回合平均獎勵 {mean_text}{route}", flush=True)

    def reset(self, **kwargs):
        reason = self.pending_reason
        label = (REASONS.get(reason, reason) if reason else
                 ("開始訓練" if self.training else "啟動環境檢查（非碰撞）"))
        self._emit(f"[重生] 原因：{label}", flush=True)
        self.phase = "重生／接近第一段走道" if self.training else "啟動環境檢查"
        try:
            obs, info = super().reset(**kwargs)
        except Exception as exc:
            self._emit(f"[重生失敗] {type(exc).__name__}: {exc}", flush=True)
            raise
        self.pending_reason = None
        self.latest_info = info
        self.phase = "收集巡邏經驗"
        if "spawn" in info:
            x, y, yaw = info["spawn"]
            self._emit(f"[重生完成] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad", flush=True)
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        self.latest_info = info
        if self.training:
            self.completed_steps += 1
        if terminated or truncated:
            reason = episode_reason(info)
            self.pending_reason = reason
            episode = info["episode"]
            if self.training:
                self.completed_episodes += 1
                self.reason_counts[reason] += 1
                self.recent_rewards.append(episode["r"])
            phase = f"第 {self.completed_episodes} 回合" if self.training else "環境檢查回合"
            route = (f" | 端點 {info['endpoints_reached']}/{info['branches_total']}"
                     f" | 返回 {info['branches_done']}/{info['branches_total']}" if "branches_done" in info else "")
            self._emit(f"[回合結束] {phase} | 重生原因：{REASONS.get(reason, reason)} "
                  f"| 獎勵 {episode['r']:.2f} | PPO 步數 {episode['l']}{route}", flush=True)
            if reason in ("stuck", "state_timeout") and "no_motion_sim_seconds" in info:
                self._emit(f"[判定依據] 相對上次有效移動：位移 {info['motion_distance_m']:.3f} m "
                      f"| 轉角 {math.degrees(info['motion_angle_rad']):.1f}° "
                      f"| 無有效移動 {info['no_motion_sim_seconds']:.2f} 模擬秒 "
                      f"| 狀態持續 {info.get('state_sim_seconds', 0):.2f} 模擬秒", flush=True)
            if self.training:
                counts = "、".join(f"{REASONS.get(key, key)}={value}" for key, value in self.reason_counts.items())
                self._emit(f"[重生統計] {counts}", flush=True)
        self.heartbeat()
        return obs, reward, terminated, truncated, info
