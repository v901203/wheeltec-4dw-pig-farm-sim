#!/usr/bin/env python3
"""Evaluate the 49-D v2 policy plus patrol FSM in an already-running Gazebo world."""

import argparse
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from model_utils import (PATROL_CHECKPOINT_DIR, PATROL_MODEL_NAME, latest_checkpoint,
                         validate_model, validate_patrol_version)
from navigation import CONFIG_FILE
from patrol_training_env import PatrolTrainingEnv


def run_episode(model, env, episode_num):
    obs, _ = env.reset()
    total_reward, steps = 0.0, 0
    previous_status = None
    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        steps += 1
        status = (info.get("state"), info.get("junction"), info.get("branches_done"))
        if status != previous_status:
            print(f"Episode {episode_num} step={steps}: state={status[0]} "
                  f"junction={status[1]} returned={status[2]}/{info.get('branches_total', 6)}")
            previous_status = status
        if terminated or truncated:
            break
    result = dict(steps=steps, reward=total_reward, success=bool(info.get("success")),
                  collision=bool(info.get("collision")), coverage=info.get("coverage", 0.0),
                  return_rate=info.get("return_rate", 0.0),
                  failure_reason=info.get("failure_reason"))
    print(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--checkpoint-dir", type=Path, default=PATROL_CHECKPOINT_DIR)
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--episodes", type=int, default=3)
    args = parser.parse_args()
    if args.episodes <= 0:
        parser.error("--episodes must be positive")
    path = args.model or latest_checkpoint(args.checkpoint_dir, prefix=PATROL_MODEL_NAME)
    if path is None:
        parser.error(f"No patrol49_v2 checkpoint in {args.checkpoint_dir}; run train_ppo.py --mode patrol first")
    model = PPO.load(str(path), device="cpu")
    validate_patrol_version(model)
    env = PatrolTrainingEnv(config_file=args.config)
    try:
        validate_model(model, env.observation_space, env.action_space)
        results = [run_episode(model, env, episode) for episode in range(1, args.episodes + 1)]
    finally:
        env.close()
    print(f"Complete patrols: {sum(r['success'] for r in results)}/{len(results)}")
    print(f"Mean endpoint coverage: {np.mean([r['coverage'] for r in results]):.1%}")
    print(f"Mean completed returns: {np.mean([r['return_rate'] for r in results]):.1%}")
    print(f"Collision episode rate: {np.mean([r['collision'] for r in results]):.1%}")
    print(f"Mean steps: {np.mean([r['steps'] for r in results]):.0f}")


if __name__ == "__main__":
    main()
