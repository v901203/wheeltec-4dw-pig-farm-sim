#!/usr/bin/env python3
"""Train a 38-D policy on corridor segments or complete hybrid patrols."""

import argparse
from pathlib import Path
import re
import signal
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from training_monitor import TrainingMonitor as Monitor

from navigation import CONFIG_FILE
from pig_pen_env import PigPenEnv
from patrol_training_env import PatrolTrainingEnv
from model_utils import CHECKPOINT_DIR, LOG_DIR, latest_checkpoint, resolve_checkpoint, validate_model
from policy_config import PPO_PARAMS
from parallel_training import WorkerFactory, TrainingVecEnv, ParallelProgress

TOTAL_TIMESTEPS = 3_000_000
CHECKPOINT_FREQ = 20_000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timesteps", type=int, default=TOTAL_TIMESTEPS, help="Additional training steps")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--mode", choices=("corridor", "patrol"), default="corridor",
                        help="patrol trains across all six branches; FSM maneuvers run between PPO decisions")
    parser.add_argument("--checkpoint-dir", type=Path, help="Default: checkpoints_corridor38 or checkpoints_patrol38")
    parser.add_argument("--log-dir", type=Path, help="Default: logs_corridor38 or logs_patrol38")
    parser.add_argument("--resume", type=Path, help="Specific compatible 38-D model; default: latest v2 checkpoint")
    parser.add_argument("--torch-threads", type=int, default=1, help="CPU threads for the small MLP (default: 1)")
    parser.add_argument("--num-envs", type=int, default=1, help="Number of isolated Gazebo workers sharing this PPO (1..8)")
    parser.add_argument("--domain-base", type=int, default=40)
    parser.add_argument("--partition-prefix", default="4wd_parallel")
    args = parser.parse_args()
    if args.timesteps <= 0:
        parser.error("--timesteps must be positive")
    if args.torch_threads <= 0:
        parser.error("--torch-threads must be positive")
    if not 1 <= args.num_envs <= 8 or not 0 <= args.domain_base <= 101-args.num_envs:
        parser.error("Use 1..8 environments and ROS domains in 0..100")
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", args.partition_prefix):
        parser.error("Invalid partition prefix")
    args.checkpoint_dir = args.checkpoint_dir or (CHECKPOINT_DIR if args.mode == "corridor" else CHECKPOINT_DIR.with_name("checkpoints_patrol38"))
    args.log_dir = args.log_dir or (LOG_DIR if args.mode == "corridor" else LOG_DIR.with_name("logs_patrol38"))
    resume = args.resume or latest_checkpoint(args.checkpoint_dir)
    if resume is not None:
        try:
            resume = resolve_checkpoint(resume)
        except FileNotFoundError as exc:
            parser.error(f"{exc}\nFor a BC model, first run: python3 {Path(__file__).with_name('train_bc.py').resolve()}\n"
                         "Recording demonstrations alone does not create a checkpoint; "
                         "BC training must finish successfully before --resume.")
    torch.set_num_threads(args.torch_threads)
    print(f"PPO CPU threads: {args.torch_threads}")
    args.checkpoint_dir.mkdir(parents=True, exist_ok=True)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    print(f"Training mode: {args.mode}; timesteps count PPO decisions, not FSM frames.", flush=True)
    if args.num_envs > 1:
        print(f"Parallel PPO: {args.num_envs} workers; ROS domains "
              f"{args.domain_base}..{args.domain_base+args.num_envs-1}; partition prefix={args.partition_prefix}", flush=True)
        env = TrainingVecEnv([WorkerFactory(i, args.domain_base, args.partition_prefix,
                                           args.mode, args.config, args.log_dir) for i in range(args.num_envs)])
        env.seed(0)
    else:
        raw_env = (PatrolTrainingEnv(config_file=args.config) if args.mode == "patrol"
                   else PigPenEnv(mode="train", config_file=args.config))
        metrics = ("success", "coverage", "return_rate", "failure_reason") if args.mode == "patrol" else ()
        env = Monitor(raw_env, filename=str(args.log_dir / "monitor"), info_keywords=metrics)
    rollout_steps = max(128, PPO_PARAMS["n_steps"] // args.num_envs)
    model = None
    try:
        if resume:
            print(f"Resume 38-D policy: {resume}")
            model = PPO.load(str(resume), env=env, device="cpu", tensorboard_log=str(args.log_dir), n_steps=rollout_steps)
            validate_model(model, env.observation_space, env.action_space)
        else:
            print("Start a new 38-D policy from scratch.")
            model = PPO("MlpPolicy", env, device="cpu", tensorboard_log=str(args.log_dir),
                        **dict(PPO_PARAMS, n_steps=rollout_steps))
        if args.num_envs == 1:
            check_env(env, warn=True)
            env.start_training(model.num_timesteps, args.timesteps)
        else:
            env.env_method("start_training", 0,
                           (args.timesteps + args.num_envs-1) // args.num_envs)
        callback = CheckpointCallback(save_freq=max(1, CHECKPOINT_FREQ // args.num_envs), save_path=str(args.checkpoint_dir),
                                      name_prefix="ppo_corridor38", verbose=1)
        if args.num_envs > 1:
            callback = [callback, ParallelProgress(model.num_timesteps, args.timesteps)]
        try:
            model.learn(total_timesteps=args.timesteps, callback=callback,
                        progress_bar=True, reset_num_timesteps=resume is None)
        except KeyboardInterrupt:
            print("Training interrupted; saving the current policy.")
        except Exception:
            emergency = args.checkpoint_dir / "ppo_corridor38_interrupted"
            model.save(str(emergency))
            print(f"Training failed; saved the current policy to {emergency}.zip", flush=True)
            raise
        model.save(str(args.checkpoint_dir / "ppo_corridor38_final"))
    finally:
        env.close()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    main()
