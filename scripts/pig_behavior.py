#!/usr/bin/env python3
"""
pig_behavior.py
================
讓豬舍地圖裡每一隻 farm_pig 假豬，每隔一段隨機時間就隨機切換行為：
    - 20% 機率：前進
    - 30% 機率：旋轉
    - 50% 機率：停止

作法：直接對每隻豬的 Gazebo VelocityControl 外掛送 Twist 指令，
      topic 格式為 /model/<豬的名字>/cmd_vel (message: gz.msgs.Twist)。
      這跟 model.sdf 裡宣告的
          <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
            <link_name>link</link_name>
          </plugin>
      是相對應的，不需要額外透過 ros_gz_bridge。

用法：
    python3 pig_behavior.py --world /path/to/pig_pen_16units_lv4.world

    # 想自行指定豬的名字列表 (不從 world 檔解析)：
    python3 pig_behavior.py --pigs pig1_a pig1_b pig2_a pig2_b

    # 先在沒有安裝 gz 的機器上測試邏輯是否正確：
    python3 pig_behavior.py --world ./pig_pen_16units_lv4.world --dry-run
"""

import argparse
import random
import re
import shutil
import subprocess
import sys
import threading
import time

# ---- 行為機率設定 (需總和為1.0) ----
ACTIONS = ["forward", "rotate", "stop"]
WEIGHTS = [0.20, 0.30, 0.50]


def parse_pig_names_from_world(world_path: str) -> list[str]:
    """從 .world 檔案裡解析出所有 farm_pig 的 include name"""
    with open(world_path, "r", encoding="utf-8") as f:
        content = f.read()

    names = []
    # 逐一比對每個 <include>...</include> 區塊，確認裡面是 farm_pig 才取 name
    for block in re.findall(r"<include>.*?</include>", content, flags=re.S):
        if "model://farm_pig" not in block:
            continue
        m = re.search(r"<name>(.*?)</name>", block)
        if m:
            names.append(m.group(1))
    return names


def build_twist_proto(linear_x: float, angular_z: float) -> str:
    return f"linear: {{x: {linear_x}}} angular: {{z: {angular_z}}}"


class PigController(threading.Thread):
    def __init__(self, name, args, stop_event):
        super().__init__(daemon=True)
        self.name = name
        self.args = args
        self.stop_event = stop_event
        self.topic = f"/model/{name}/cmd_vel"

    def publish(self, linear_x: float, angular_z: float):
        proto = build_twist_proto(linear_x, angular_z)
        if self.args.dry_run:
            print(f"[dry-run] {self.topic}  ->  {proto}")
            return
        try:
            subprocess.run(
                ["gz", "topic", "-t", self.topic, "-m", "gz.msgs.Twist", "-p", proto],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0,
            )
        except Exception as e:  # noqa: BLE001
            print(f"[警告] 發布給 {self.name} 失敗: {e}", file=sys.stderr)

    def run(self):
        rng = random.Random()
        while not self.stop_event.is_set():
            action = rng.choices(ACTIONS, weights=WEIGHTS, k=1)[0]

            if action == "forward":
                lin, ang = self.args.forward_speed, 0.0
            elif action == "rotate":
                lin = 0.0
                ang = self.args.rotate_speed * rng.choice([-1, 1])
            else:  # stop
                lin, ang = 0.0, 0.0

            duration = rng.uniform(self.args.min_duration, self.args.max_duration)

            # 只在行為切換時發布一次指令。VelocityControl 外掛會持續套用最後
            # 收到的速度，不需要一直重發；過去每0.3秒重發一次的做法，在32隻豬
            # 同時運作時會產生大量短命的 gz-transport CLI 子程序，每個都要重新
            # 做一次 discovery 廣播，量大時會干擾 Gazebo GUI 與 server 之間的
            # 場景同步(SceneBroadcaster)，導致 GUI 的 Entity Tree 漏更新。
            self.publish(lin, ang)

            if self.args.republish_interval > 0:
                # 仍保留一個低頻率的「保險重發」，避免萬一漏封包導致豬卡住不動，
                # 但預設間隔拉長很多，大幅降低 subprocess 呼叫頻率。
                end_time = time.time() + duration
                while not self.stop_event.is_set() and time.time() < end_time:
                    self.stop_event.wait(self.args.republish_interval)
                    if not self.stop_event.is_set() and time.time() < end_time:
                        self.publish(lin, ang)
            else:
                self.stop_event.wait(duration)

        # 收到停止訊號時，確保豬最後是靜止的
        self.publish(0.0, 0.0)


def main():
    parser = argparse.ArgumentParser(description="豬隻隨機行為控制 (前進20% / 旋轉30% / 停止50%)")
    parser.add_argument("--world", type=str, default=None, help="世界檔路徑，會自動解析所有 farm_pig 的名字")
    parser.add_argument("--pigs", nargs="*", default=None, help="直接指定豬的名字列表（不從world檔解析）")
    parser.add_argument("--forward-speed", type=float, default=0.3, help="前進速度 m/s (預設0.3)")
    parser.add_argument("--rotate-speed", type=float, default=0.6, help="旋轉角速度 rad/s (預設0.6，正負隨機)")
    parser.add_argument("--min-duration", type=float, default=2.0, help="每個行為最短持續秒數")
    parser.add_argument("--max-duration", type=float, default=5.0, help="每個行為最長持續秒數")
    parser.add_argument("--republish-interval", type=float, default=3.0,
                         help="保險重發的間隔秒數，設為0代表切換行為時只發一次、完全不重發 (預設3.0)")
    parser.add_argument("--dry-run", action="store_true", help="只印出會發布的指令，不實際呼叫 gz topic（測試用）")
    args = parser.parse_args()

    if args.pigs:
        pig_names = args.pigs
    elif args.world:
        pig_names = parse_pig_names_from_world(args.world)
    else:
        parser.error("請用 --world 指定世界檔路徑，或用 --pigs 直接指定豬的名字列表")

    if not pig_names:
        print("[錯誤] 沒有找到任何 farm_pig，請確認 --world 路徑或 --pigs 內容是否正確。", file=sys.stderr)
        sys.exit(1)

    if not args.dry_run and shutil.which("gz") is None:
        print("[錯誤] 找不到 gz 指令，請確認已安裝 Gazebo (gz-tools) 並在 PATH 中。"
              "也可以先加 --dry-run 測試邏輯。", file=sys.stderr)
        sys.exit(1)

    print(f"控制豬隻數量: {len(pig_names)}")
    print(f"行為機率設定: 前進={WEIGHTS[0]*100:.0f}%  旋轉={WEIGHTS[1]*100:.0f}%  停止={WEIGHTS[2]*100:.0f}%")
    print("按 Ctrl-C 結束並讓所有豬停止。\n")

    stop_event = threading.Event()
    threads = [PigController(name, args, stop_event) for name in pig_names]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n收到停止訊號，正在讓所有豬停止...")
        stop_event.set()
        for t in threads:
            t.join(timeout=3.0)
        print("已結束。")


if __name__ == "__main__":
    main()
