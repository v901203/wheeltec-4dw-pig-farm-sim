#!/usr/bin/env python3
"""Load a Stable-Baselines3 model and run inference with real-time rendering.

Usage:
  PYTHONPATH=. python3 examples/run_inference_sb3.py --model models/best_model.zip --episodes 5

Note: Requires `stable_baselines3` and a display (or X11 forwarding). If SB3 is not installed,
the script will print an instruction and exit.
"""
import argparse
import time
from pathlib import Path

try:
    from stable_baselines3 import PPO
except Exception:
    PPO = None

from rl_envs.rect_world_env import RectWorldEnv


def run(model_path: str, episodes: int = 5):
    if PPO is None:
        print('stable_baselines3 not installed. Install with: pip install stable-baselines3')
        return

    p = Path(model_path)
    if not p.exists():
        print(f'Model not found: {model_path}')
        return

    env = RectWorldEnv()
    model = PPO.load(str(p))

    for ep in range(episodes):
        obs = env.reset()
        done = False
        total = 0.0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, done, info = env.step(int(action))
            total += r
            env.render(show=True)
            time.sleep(0.02)
        print(f'Episode {ep+1} return {total:.1f}')
        time.sleep(0.5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--episodes', type=int, default=5)
    args = parser.parse_args()
    run(args.model, args.episodes)
