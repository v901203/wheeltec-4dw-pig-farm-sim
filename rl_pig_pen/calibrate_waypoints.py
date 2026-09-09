#!/usr/bin/env python3
"""
路徑點校準工具（支援中途存檔與續點）

用法：
  Terminal 1: ./launch_clean.sh 或 ./launch_rl_training.sh
  Terminal 2: ros2 run teleop_twist_keyboard teleop_twist_keyboard
  Terminal 3: python3 calibrate_waypoints.py

特性：
  - 每記錄一個點就立刻寫入 waypoints_partial.json
  - 若中途崩潰，重新執行會問你要不要從斷點繼續
  - 全部完成後覆寫為正式的 waypoints.json
"""

import json
import math
import os
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ── 路徑點定義（依巡邏順序） ────────────────────────────────────────────
# 第一個固定是 start（車子已在起點，直接按 Enter）
WAYPOINT_PROMPTS = [
    ("start",            "走道最右端起點 — 車子已在這裡，直接按 Enter"),
    ("junction1_entry",  "主幹道第 1 個路口中心點"),
    ("junction1_top",    "第 1 條支線 上端（北端）"),
    ("junction1_center", "回到第 1 個路口中心點"),
    ("junction1_bottom", "第 1 條支線 下端（南端）"),
    ("junction1_exit",   "再次回到第 1 個路口，準備繼續往左"),
    ("junction2_entry",  "主幹道第 2 個路口中心點"),
    ("junction2_top",    "第 2 條支線 上端（北端）"),
    ("junction2_center", "回到第 2 個路口中心點"),
    ("junction2_bottom", "第 2 條支線 下端（南端）"),
    ("junction2_exit",   "再次回到第 2 個路口，準備繼續往左"),
    ("junction3_entry",  "主幹道第 3 個路口中心點"),
    ("junction3_top",    "第 3 條支線 上端（北端）"),
    ("junction3_center", "回到第 3 個路口中心點"),
    ("junction3_bottom", "第 3 條支線 下端（南端）"),
    ("junction3_exit",   "再次回到第 3 個路口，準備繼續往左"),
    ("goal",             "走道最左端 — 巡邏終點"),
]

PARTIAL_FILE  = Path("waypoints_partial.json")
FINAL_FILE    = Path("waypoints.json")
WORLD_X_START = 0.0
WORLD_Y_START = -14.0
WORLD_YAW_START = 1.5708   # 對應 launch_clean.sh 的 spawn yaw


class OdomReader(Node):
    def __init__(self):
        super().__init__("calibrate_wp_reader")
        self._odom = None
        self._lock = threading.Lock()
        self.create_subscription(Odometry, "/odom", self._cb, 10)

    def _cb(self, msg):
        with self._lock:
            self._odom = msg

    def get_pose(self):
        with self._lock:
            if self._odom is None:
                return None
            p   = self._odom.pose.pose.position
            q   = self._odom.pose.pose.orientation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y ** 2 + q.z ** 2),
            )
            return p.x, p.y, yaw


def save_partial(start_info, recorded):
    """每次記錄後立刻存成暫存檔，防止崩潰遺失。"""
    data = {
        "_status": "partial",
        "start": start_info,
        "patrol_waypoints": recorded,
    }
    PARTIAL_FILE.write_text(json.dumps(data, indent=2, ensure_ascii=False))


def load_partial():
    """讀取上次未完成的校準進度。"""
    if not PARTIAL_FILE.exists():
        return None
    try:
        data = json.loads(PARTIAL_FILE.read_text())
        if data.get("_status") == "partial":
            return data
    except Exception:
        pass
    return None


def main():
    # ── 啟動前稍等，讓 Gazebo ZMQ 連線穩定 ────────────────────────────────
    print("\n等待 ROS2 環境穩定（3 秒）...", flush=True)
    time.sleep(3)

    rclpy.init()
    node = OdomReader()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "=" * 60)
    print("  豬舍巡邏路徑點校準工具（支援續點）")
    print("=" * 60)

    # ── 等待 /odom 上線 ──────────────────────────────────────────────────
    print("等待 /odom 資料...", end="", flush=True)
    timeout = 20
    t0 = time.time()
    while node.get_pose() is None:
        if time.time() - t0 > timeout:
            print("\n[錯誤] 等待 /odom 超時，請確認 launch_clean.sh 有在執行。")
            node.destroy_node()
            rclpy.shutdown()
            return
        time.sleep(0.3)
        print(".", end="", flush=True)
    print(" ✓\n")

    # ── 檢查是否有未完成的校準可以續點 ─────────────────────────────────────
    start_info  = None
    recorded    = []   # list of {name, x, y}
    start_index = 0    # 從哪個 prompt 開始（0 = 從頭）

    partial = load_partial()
    if partial:
        done_names = {wp["name"] for wp in partial["patrol_waypoints"]}
        done_names.add("start")   # start 存在 partial["start"] 裡，算已完成
        n_done = 1 + len(partial["patrol_waypoints"])   # start + 已記錄的路徑點

        print(f"⚠️  發現上次未完成的校準進度（已完成 {n_done}/{len(WAYPOINT_PROMPTS)} 個點）。")
        ans = input("   要從斷點繼續嗎？[Y/n] ").strip().lower()
        if ans in ("", "y", "yes"):
            start_info  = partial["start"]
            recorded    = partial["patrol_waypoints"]
            start_index = n_done   # 從第 n_done 個 prompt 繼續
            print(f"   → 從第 {start_index + 1} 個點繼續。\n")
        else:
            print("   → 從頭開始。\n")
            PARTIAL_FILE.unlink(missing_ok=True)

    # ── 校準主迴圈 ────────────────────────────────────────────────────────
    print("在另一個 Terminal 開啟 teleop 遙控車子：")
    print("  ros2 run teleop_twist_keyboard teleop_twist_keyboard\n")

    for i, (key, desc) in enumerate(WAYPOINT_PROMPTS):
        if i < start_index:
            continue   # 跳過已完成的點

        total = len(WAYPOINT_PROMPTS)
        print(f"[{i+1:02d}/{total}] 目標：{desc}")

        if i == 0:
            print("        車子已在起點，直接按 Enter 記錄...", end="", flush=True)
        else:
            print("        把車開到位後按 Enter 記錄...", end="", flush=True)
        input()

        # 讀取當前位置
        pose = node.get_pose()
        while pose is None:
            print("        等待 /odom...", flush=True)
            time.sleep(0.5)
            pose = node.get_pose()

        x, y, yaw = pose
        print(f"        ✓ x={x:.3f}  y={y:.3f}  yaw={yaw:.3f}\n")

        if i == 0:
            # start：存世界座標（給 teleport 用）+ 記錄 odom 當基準
            start_info = {
                "world_x":   WORLD_X_START,
                "world_y":   WORLD_Y_START,
                "world_yaw": WORLD_YAW_START,
            }
            # 同時存下校準時的 odom 基準，方便 debug
            start_info["_calib_odom_x"] = x
            start_info["_calib_odom_y"] = y
        else:
            # 路徑點：存相對於起點的 odom 位移
            base_x = start_info.get("_calib_odom_x", 0.0)
            base_y = start_info.get("_calib_odom_y", 0.0)
            recorded.append({
                "name": key,
                "x": round(x - base_x, 4),
                "y": round(y - base_y, 4),
            })

        # ★ 每記錄一個點立刻寫暫存檔（防崩潰遺失）
        save_partial(start_info, recorded)
        print(f"        [存檔 ✓] waypoints_partial.json 已更新")

    # ── 校準完成，輸出正式檔案 ────────────────────────────────────────────
    final = {
        "start": {k: v for k, v in start_info.items() if not k.startswith("_")},
        "patrol_waypoints": recorded,
    }
    FINAL_FILE.write_text(json.dumps(final, indent=2, ensure_ascii=False))
    PARTIAL_FILE.unlink(missing_ok=True)   # 刪除暫存檔

    print("=" * 60)
    print(f"✓ 校準完成！已儲存 {FINAL_FILE}（共 {len(recorded)} 個巡邏路徑點）。")
    print("  下一步：python3 train_ppo.py")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
