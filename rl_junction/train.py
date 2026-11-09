#!/usr/bin/env python3
"""BC-initialized PPO and continuation for isolated junction tasks; no hardware."""
import argparse
import json
from datetime import datetime
from pathlib import Path
import signal
import hashlib
import gymnasium as gym
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from junction_env import HybridJunctionEnv as JunctionEnv, HANDOFF_STATES
from tasks import TASKS
from learning_common import check_model, save_model, contract, model_spec, HistoryObservation
from geometry_diagnostics import GeometryDiagnosticsLog, encode
from training_checkpoints import ValidationCheckpoints
from phase_training import PhaseTrainingReward, AnchoredPPO


class TrainingDiagnostics(gym.Wrapper):
    """Record terminal causes before VecEnv resets; never change observations/rewards."""
    def __init__(self, env, run):
        super().__init__(env)
        self.geometry = GeometryDiagnosticsLog(run/"failure_geometry")
        self.episodes = (run/"episode_details.jsonl").open("w")
        self.episode = 0

    def reset(self, *, seed=None, options=None):
        result = self.env.reset(seed=seed, options=options)
        self.episode += 1
        return result

    def step(self, action):
        observation, reward, terminated, truncated, info = self.env.step(action)
        raw = self.env.unwrapped
        if terminated or truncated:
            row = {"episode": self.episode, "terminated": terminated, "truncated": truncated,
                   "outcome": info, "fsm_status": raw.teacher.status()}
            self.episodes.write(json.dumps(row, default=encode)+"\n")
            self.episodes.flush()
            if not info.get("success", False):
                self.geometry.record_scan(raw.scan, raw.scan_stamp, raw.teacher.controller, {
                    "episode": self.episode, "task": raw.task.name,
                    "learner_action_on_previous_scan": action,
                    "snapshot_timing": "after_step", "context": raw.cached_context.copy(),
                    "diagnostic_pose_only": raw.pose.copy(), "outcome": info})
                self.geometry.flush()
        return observation, reward, terminated, truncated, info

    def close(self):
        try:
            try:
                self.geometry.close()
            finally:
                self.episodes.close()
        finally:
            self.env.close()


class TaskRoundRobin(gym.Wrapper):
    """Short runs must see every allowed skill, not repeat one random task."""
    def __init__(self, env, tasks):
        super().__init__(env)
        self.tasks, self.next_task = list(tasks), 0
        if not self.tasks:
            raise ValueError("No curriculum tasks")

    def reset(self, *, seed=None, options=None):
        self.env.fixed_task = self.tasks[self.next_task % len(self.tasks)]
        observation, info = self.env.reset(seed=seed, options=options)
        self.next_task += 1
        return observation, info


class CriticWarmup(BaseCallback):
    """Fit value estimates before allowing small PPO actor updates."""
    def __init__(self, rollouts, run=None, spec=None, critic_learning_rate=None, critic_epochs=None):
        super().__init__()
        self.warmup, self.rollouts = rollouts, 0
        self.run, self.spec = run, spec
        self.critic_learning_rate, self.critic_epochs = critic_learning_rate, critic_epochs

    def _on_training_start(self):
        self.actor_schedule = self.model.lr_schedule
        self.actor_epochs = self.model.n_epochs
        self.starting_steps = self.model.num_timesteps

    def _progress(self):
        if self.run is not None:
            (self.run/"progress.json").write_text(json.dumps({
                "timesteps": self.num_timesteps,
                "additional_timesteps": self.num_timesteps-self.starting_steps,
                "completed_rollouts": self.rollouts,
                "actor_update_rollouts": max(0, self.rollouts-self.warmup)}, indent=2))

    def _actor(self, enabled):
        for module in (self.model.policy.mlp_extractor.policy_net, self.model.policy.action_net):
            for parameter in module.parameters():
                parameter.requires_grad_(enabled)
        self.model.policy.log_std.requires_grad_(enabled)

    def _on_rollout_start(self):
        if self.run is not None and self.rollouts:
            save_model(self.model, self.run/"latest", self.spec)
            self._progress()
            # Flush the PREVIOUS completed update at its actual step count;
            # SB3's regular rollout log otherwise carries update statistics
            # forward to the next rollout's timestamp.
            self.model.logger.record("time/total_timesteps", self.num_timesteps)
            self.model.logger.record("time/run_timesteps", self.num_timesteps-self.starting_steps)
            self.model.logger.record("training/completed_rollouts", self.rollouts)
            self.model.logger.record("training/actor_update_rollouts", max(0, self.rollouts-self.warmup))
            self.model.logger.dump(step=self.num_timesteps)
        warming = self.rollouts < self.warmup
        self._actor(not warming)
        warmup_rate = self.critic_learning_rate
        self.model.lr_schedule = ((lambda _, rate=warmup_rate: rate)
            if warming and self.critic_learning_rate is not None else self.actor_schedule)
        self.model.n_epochs = self.critic_epochs if warming and self.critic_epochs is not None else self.actor_epochs
        self.rollouts += 1

    def _on_step(self):
        return True

    def _on_training_end(self):
        self._actor(True)
        self.model.lr_schedule = self.actor_schedule
        self.model.n_epochs = self.actor_epochs
        self._progress()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase-reward", action="store_true", help="Training-only phase progress and measured stagnation")
    parser.add_argument("--bc-reference", type=Path, help="Frozen compatible BC policy for auxiliary actor rehearsal")
    parser.add_argument("--bc-anchor-coef", type=float, default=.1)
    parser.add_argument("--eval-every", type=int, default=5120, help="Validate/save at rollout boundaries; 0 disables")
    parser.add_argument("--eval-episodes", type=int, default=3)
    parser.add_argument("--eval-seed", type=int, default=810000)
    parser.add_argument("--task", choices=list(HANDOFF_STATES), default=None,
                        help="Omit to sample all three tasks at reset")
    parser.add_argument("--timesteps", type=int, default=1024)
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--n-epochs", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=1e-5)
    parser.add_argument("--critic-warmup-rollouts", type=int, default=1)
    parser.add_argument("--critic-learning-rate", type=float, default=3e-4,
                        help="Value-only warmup rate; actor remains frozen during warmup")
    parser.add_argument("--critic-warmup-epochs", type=int, default=16)
    parser.add_argument("--max-steps", type=int, default=300,
                        help="Training episode cap: 60 simulated seconds by default")
    parser.add_argument("--task-sampling", choices=["round-robin", "random"], default="round-robin")
    parser.add_argument("--torch-threads", type=int, default=1)
    parser.add_argument("--geometry-gate", type=Path, required=True,
                        help="Passed localization audit of this exact checkpoint and current environment")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--init-model", type=Path, required=True,
                        help="BC/PPO checkpoint (.zip plus matching .json); no training from scratch")
    parser.add_argument("--output", type=Path, default=Path("logs/rl_junction_training"))
    args = parser.parse_args()
    if (args.timesteps <= 0 or min(args.n_epochs, args.max_steps, args.torch_threads, args.critic_warmup_epochs) < 1 or
            min(args.n_steps, args.batch_size) < 2):
        parser.error("timesteps must be positive")
    if args.eval_every < 0 or args.eval_episodes < 1:
        parser.error("Invalid validation interval/episode count")
    checkpoint = check_model(args.init_model)
    if (not 0 < args.learning_rate <= 3e-4 or not 0 < args.critic_learning_rate <= 1e-3 or args.n_steps % args.batch_size or
            args.critic_warmup_rollouts < 0 or
            args.timesteps <= args.critic_warmup_rollouts*args.n_steps):
        parser.error("Invalid PPO sizes/rate, or no actor-update rollout remains after critic warmup")
    gate = json.loads(args.geometry_gate.read_text())
    tasks_audited = gate.get("tasks", {})
    criteria_ok = (gate.get("minimum_episodes_per_task", 0) >= 2 and
                   0 < gate.get("centre_error_limit_m", float("inf")) <= .1 and
                   not gate.get("unfinished_geometry_recoveries", ["missing"]) and
                   all(t.name in tasks_audited and tasks_audited[t.name].get("episodes", 0) >= 2 and
                       tasks_audited[t.name].get("geometry_frames", 0) > 0 and
                       tasks_audited[t.name].get("wrong_centre_frames", -1) == 0 and
                       tasks_audited[t.name].get("max_centre_error_m", float("inf")) <= .1
                       for t in TASKS))
    if (not gate.get("passed") or not criteria_ok or gate.get("contract") != contract() or
            gate.get("model_sha256") != hashlib.sha256(checkpoint.read_bytes()).hexdigest()):
        parser.error("Localization gate did not pass for this exact model/current environment")
    spec = model_spec(checkpoint)
    if not 0 < args.bc_anchor_coef <= 1:
        parser.error("BC anchor coefficient must be in (0,1]")
    if args.bc_reference and model_spec(check_model(args.bc_reference)) != spec:
        parser.error("BC reference architecture/tasks must match checkpoint")
    if args.task and args.task not in spec["tasks"]:
        parser.error("Task was not included in the BC checkpoint")
    torch.set_num_threads(args.torch_threads)
    run = args.output / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True)
    (run / "config.json").write_text(json.dumps({
        "task": args.task, "timesteps": args.timesteps, "seed": args.seed,
        "observation_size": spec["observation_size"], "control_dt": .2,
        "max_linear": .2, "max_angular": .6, "safety_enabled": True,
        "max_steps": args.max_steps, "task_sampling": args.task_sampling,
        "init_model": str(checkpoint), "contract": contract()}, indent=2))
    (run/"ppo_settings.json").write_text(json.dumps({**vars(args),
        "clip_range": .1, "target_kl": .01, "geometry_gate": gate}, default=str, indent=2))
    raw_env = JunctionEnv(task=args.task, available_tasks=spec["tasks"], max_steps=args.max_steps)
    simulator_env = raw_env
    if args.task is None and args.task_sampling == "round-robin":
        raw_env = TaskRoundRobin(raw_env, [t.name for t in TASKS if t.name in spec["tasks"]])
    if args.phase_reward:
        raw_env = PhaseTrainingReward(raw_env, trace_path=run/"reward_steps.jsonl")
    raw_env = TrainingDiagnostics(raw_env, run)
    env = Monitor(HistoryObservation(raw_env, spec["history"]), str(run / "episodes.csv"),
                  info_keywords=("task", "success", "collision", "off_route", "no_progress"))
    model = None
    def stop(*_):
        raise KeyboardInterrupt
    old_term = signal.signal(signal.SIGTERM, stop)
    try:
        model = AnchoredPPO.load(checkpoint, env=env, seed=args.seed, verbose=1, device="cpu",
                         n_steps=args.n_steps, batch_size=args.batch_size, n_epochs=args.n_epochs,
                         learning_rate=args.learning_rate, clip_range=.1, target_kl=.01)
        if args.bc_reference:
            reference = PPO.load(check_model(args.bc_reference), device="cpu")
            model.attach_bc(reference.policy, args.learning_rate, args.bc_anchor_coef)
        starting_steps, starting_updates = model.num_timesteps, model._n_updates
        (run/"continuation.json").write_text(json.dumps({
            "source_checkpoint": str(checkpoint), "starting_timesteps": starting_steps,
            "starting_ppo_epoch_updates": starting_updates, "requested_additional_timesteps": args.timesteps}, indent=2))
        model.set_logger(configure(str(run/"metrics"), ["stdout", "csv", "json"]))
        callbacks = [CriticWarmup(args.critic_warmup_rollouts, run, spec,
                                  args.critic_learning_rate, args.critic_warmup_epochs)]
        if args.eval_every:
            eval_spec = dict(spec, tasks=[args.task] if args.task else spec["tasks"])
            validation = ValidationCheckpoints(run, spec, simulator_env, args.eval_every,
                                               args.eval_episodes, args.eval_seed)
            validation.eval_tasks = eval_spec["tasks"]
            callbacks.append(validation)
        model.learn(total_timesteps=args.timesteps,
                    reset_num_timesteps=False,
                    callback=callbacks)
        model.logger.record("time/total_timesteps", model.num_timesteps)
        model.logger.record("time/run_timesteps", model.num_timesteps-starting_steps)
        model.logger.record("training/final_dump", 1)
        model.logger.dump(step=model.num_timesteps)
        save_model(model, run / "model", spec)
        (run/"completion.json").write_text(json.dumps({"complete": True,
            "actual_timesteps": model.num_timesteps, "requested_timesteps": args.timesteps,
            "starting_timesteps": starting_steps,
            "additional_timesteps": model.num_timesteps-starting_steps,
            "additional_ppo_epoch_updates": model._n_updates-starting_updates,
            "note": "Initial PPO fine-tuning only; requires independent closed-loop evaluation."}, indent=2))
    except KeyboardInterrupt:
        if model is not None:
            save_model(model, run / "interrupted_model", spec)
            (run/"interrupted.json").write_text(json.dumps({"complete": False,
                "actual_timesteps": model.num_timesteps, "reason": "signal_or_keyboard_interrupt"}, indent=2))
    except Exception as exc:
        if model is not None:
            save_model(model, run/"failed_run_checkpoint", spec)
        (run/"infrastructure_error.json").write_text(json.dumps({"type": type(exc).__name__, "error": str(exc)}, indent=2))
        raise
    finally:
        try:
            env.close()
        finally:
            try:
                if model is not None:
                    model.logger.close()
            finally:
                signal.signal(signal.SIGTERM, old_term)


if __name__ == "__main__":
    main()
