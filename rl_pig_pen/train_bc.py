#!/usr/bin/env python3
"""Offline behavior cloning of the PPO actor; no Gazebo or ROS node required."""

import argparse
import copy
import json
from pathlib import Path

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
import torch

from demonstrations import DEMO_DIR, FORMAT_VERSION, load_demonstrations, split_demonstrations
from navigation import CONFIG_FILE, MAX_ANG, MAX_LIN, load_config
from policy_config import PPO_PARAMS, policy_spaces

DEFAULT_OUTPUT = Path(__file__).with_name("checkpoints_bc") / "ppo_corridor38_bc.zip"


class OfflineSpaces(gym.Env):
    """Used only to initialize a standard SB3 model for supervised training."""
    def __init__(self):
        self.observation_space, self.action_space = policy_spaces()

    def reset(self, **kwargs):
        raise RuntimeError("OfflineSpaces cannot collect rollouts; use PigPenEnv for PPO")

    def step(self, action):
        raise RuntimeError("OfflineSpaces cannot collect rollouts; use PigPenEnv for PPO")


def fit_actor(model, observations, actions, train_indices, val_indices, *, epochs=30,
              batch_size=256, learning_rate=3e-4, patience=5, seed=0):
    """Fit only actor weights, selecting the best epoch on held-out data."""
    policy = model.policy
    policy.set_training_mode(True)
    parameters = (list(policy.mlp_extractor.policy_net.parameters()) +
                  list(policy.action_net.parameters()) + list(policy.pi_features_extractor.parameters()))
    optimizer = torch.optim.Adam(parameters, lr=learning_rate)
    rng = np.random.default_rng(seed)
    scale = torch.tensor([MAX_LIN, MAX_ANG], device=model.device)
    # Keep the full recording on CPU; transfer one minibatch at a time.
    def batch(indices):
        return (torch.as_tensor(observations[indices], device=model.device),
                torch.as_tensor(actions[indices], device=model.device))

    def validation():
        squared, absolute = 0.0, np.zeros(2)
        with torch.no_grad():
            for offset in range(0, len(val_indices), batch_size):
                indices = val_indices[offset:offset+batch_size]
                obs, targets = batch(indices)
                means = policy.get_distribution(obs).mode()
                squared += (((means-targets)/scale)**2).sum().item()
                clipped = torch.maximum(torch.minimum(means, torch.tensor([MAX_LIN, MAX_ANG], device=model.device)),
                                        torch.tensor([0, -MAX_ANG], device=model.device))
                absolute += (clipped-targets).abs().sum(dim=0).cpu().numpy()
        return squared/(2*len(val_indices)), (absolute/len(val_indices)).tolist()

    baseline, baseline_mae = validation()
    best, best_epoch, bad_epochs = baseline, 0, 0
    best_state = copy.deepcopy(policy.state_dict())
    history = []
    for epoch in range(1, epochs+1):
        indices = rng.permutation(train_indices)
        total = 0.0
        for offset in range(0, len(indices), batch_size):
            selection = indices[offset:offset+batch_size]
            obs, targets = batch(selection)
            means = policy.get_distribution(obs).mode()
            loss = (((means-targets)/scale)**2).mean()
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(parameters, 1.0)
            optimizer.step()
            total += loss.item()*len(selection)
        val_loss, mae = validation()
        history.append(dict(epoch=epoch, train_loss=total/len(indices), validation_loss=val_loss,
                            velocity_mae=mae[0], angular_mae=mae[1]))
        print(f"epoch={epoch:02d} train={total/len(indices):.5f} val={val_loss:.5f} "
              f"MAE: v={mae[0]:.3f} m/s, w={mae[1]:.3f} rad/s", flush=True)
        if val_loss < best:
            best, best_epoch, bad_epochs = val_loss, epoch, 0
            best_state = copy.deepcopy(policy.state_dict())
        else:
            bad_epochs += 1
            if bad_epochs >= patience:
                break
    policy.load_state_dict(best_state)
    # Preserve modest exploration for subsequent PPO; critic remains untrained.
    with torch.no_grad():
        policy.log_std.copy_(torch.log(torch.tensor([0.1, 0.2], device=model.device)))
    policy.set_training_mode(False)
    return dict(baseline_validation_loss=baseline, baseline_mae=baseline_mae,
                best_validation_loss=best, best_epoch=best_epoch, history=history)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEMO_DIR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--patience", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    args = parser.parse_args()
    if min(args.epochs, args.batch_size, args.patience) < 1 or not 0 < args.learning_rate < float("inf"):
        parser.error("Epochs, batch size, patience and learning rate must be positive")
    output = args.output.with_suffix(".zip")
    if output.exists() or output.with_suffix(".json").exists():
        parser.error(f"Output already exists: {output}; choose another --output")
    torch.set_num_threads(1)
    if args.device == "cuda" and not torch.cuda.is_available():
        parser.error("CUDA is unavailable; use --device cpu")
    cfg = load_config(args.config)
    try:
        observations, actions, groups = load_demonstrations(args.data, cfg)
        train, val, split = split_demonstrations(groups, args.seed)
    except ValueError as exc:
        parser.error(f"{exc}\nDemonstration directory: {args.data.resolve()}\n"
                     "No BC checkpoint was created. Correct or extend the recordings, then rerun train_bc.py.")
    print(f"Samples={len(actions)}, sessions={len(np.unique(groups))}; "
          f"train={len(train)}, validation={len(val)} ({split})")
    model = PPO("MlpPolicy", OfflineSpaces(), device=args.device, seed=args.seed, **PPO_PARAMS)
    report = fit_actor(model, observations, actions, train, val, epochs=args.epochs,
                       batch_size=args.batch_size, learning_rate=args.learning_rate,
                       patience=args.patience, seed=args.seed)
    if report["best_epoch"] == 0:
        raise RuntimeError("Validation did not improve; record more consistent demonstrations before saving a BC policy")
    output.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(output))
    report.update(format=FORMAT_VERSION, samples=len(actions), sessions=len(np.unique(groups)),
                  split=split, train_samples=len(train), validation_samples=len(val),
                  source=str(args.data.resolve()), seed=args.seed,
                  note="Actor pretraining only; validate navigation and continue PPO in Gazebo")
    output.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Saved best BC actor as a PPO checkpoint: {output}")
    print("Critic is not pretrained. Use train_ppo.py --resume with this checkpoint to continue RL.")


if __name__ == "__main__":
    main()
