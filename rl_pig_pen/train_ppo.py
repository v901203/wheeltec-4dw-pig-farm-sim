#!/usr/bin/env python3
"""
PPO 訓練腳本 — 豬舍巡邏任務

使用前：
  1. ./launch_clean.sh            （另一個 Terminal）
  2. python3 calibrate_waypoints.py  （先校準路徑點）
  3. python3 train_ppo.py            （開始訓練）

監控訓練：
  tensorboard --logdir logs/
  → 開瀏覽器看 http://localhost:6006
"""

import os
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
)
from stable_baselines3.common.env_checker import check_env

from pig_pen_env import PigPenEnv

# ── 訓練參數 ────────────────────────────────────────────────────────────
TOTAL_TIMESTEPS  = 3_000_000   # 總訓練步數；豬舍巡邏任務建議至少 2M
CHECKPOINT_FREQ  = 20_000      # 每隔幾步存一次 checkpoint
CHECKPOINT_DIR   = "checkpoints"
LOG_DIR          = "logs"
WAYPOINTS_FILE   = "waypoints.json"
EVAL_FREQ        = 50_000      # 每隔幾步做一次評估（獨立 episode）

# ── PPO 超參數（基於 MlpPolicy + 連續動作的常用設定） ────────────────
PPO_PARAMS = dict(
    n_steps    = 2048,    # 每次更新前蒐集的步數
    batch_size = 512,     # mini-batch 大小；RTX 5090 可以設更大
    n_epochs   = 10,      # 每次更新重複迭代幾次
    gamma      = 0.99,    # 折扣因子
    gae_lambda = 0.95,    # GAE lambda
    clip_range = 0.2,     # PPO clip 範圍
    ent_coef   = 0.01,    # 熵正則係數（鼓勵探索）
    learning_rate = 3e-4,
    verbose    = 1,
    policy_kwargs = dict(
        net_arch = [256, 256],   # 兩層 256 neuron 的全連接網路
    ),
)


def main():
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    os.makedirs(LOG_DIR,        exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"\n{'='*50}")
    print(f"  訓練裝置: {device}")
    if device == "cuda":
        print(f"  GPU: {torch.cuda.get_device_name(0)}")
        print(f"  VRAM: {torch.cuda.get_device_properties(0).total_memory // 1024**3} GB")
    print(f"{'='*50}\n")

    # ── 建立環境 ──────────────────────────────────────────────────────────
    print("建立訓練環境...")
    train_env = Monitor(PigPenEnv(WAYPOINTS_FILE), filename=os.path.join(LOG_DIR, "monitor"))

    # 快速檢查觀察/動作空間定義有沒有問題
    print("檢查環境介面...")
    check_env(train_env, warn=True)
    print("環境介面 ✓\n")

    # ── Callbacks ─────────────────────────────────────────────────────────
    checkpoint_cb = CheckpointCallback(
        save_freq  = CHECKPOINT_FREQ,
        save_path  = CHECKPOINT_DIR,
        name_prefix= "ppo_pig_pen",
        verbose    = 1,
    )

    # EvalCallback 已移除（會建立第二個環境與 Gazebo 衝突）

    # ── 建立模型 ──────────────────────────────────────────────────────────
    model = PPO(
        "MlpPolicy",
        train_env,
        device         = "cpu",  # MlpPolicy 用 CPU 比 GPU 快
        tensorboard_log= LOG_DIR,
        **PPO_PARAMS,
    )

    print(f"模型架構：{PPO_PARAMS['policy_kwargs']['net_arch']}")
    print(f"總訓練步數：{TOTAL_TIMESTEPS:,}")
    print(f"Checkpoint 每 {CHECKPOINT_FREQ:,} 步儲存一次\n")
    print("開始訓練！用 Ctrl-C 可中斷（最後一個 checkpoint 不會遺失）。")
    print("TensorBoard：tensorboard --logdir logs/\n")

    try:
        model.learn(
            total_timesteps = TOTAL_TIMESTEPS,
            callback        = [checkpoint_cb],
            progress_bar    = True,
            reset_num_timesteps = True,
        )
    except KeyboardInterrupt:
        print("\n訓練中斷，儲存當前模型...")

    # 儲存最終模型
    final_path = os.path.join(CHECKPOINT_DIR, "ppo_pig_pen_final")
    model.save(final_path)
    print(f"\n✓ 最終模型已儲存：{final_path}.zip")

    train_env.close()


if __name__ == "__main__":
    main()
