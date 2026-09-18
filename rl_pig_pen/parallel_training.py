"""Independent ROS/Gazebo workers feeding one shared PPO learner."""

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import time

from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import SubprocVecEnv


@dataclass
class WorkerFactory:
    index: int
    domain_base: int
    partition_prefix: str
    mode: str
    config: Path
    log_dir: Path

    def __call__(self):
        # Set transport isolation before creating any ROS context or subprocess.
        os.environ["ROS_DOMAIN_ID"] = str(self.domain_base + self.index)
        os.environ["IGN_PARTITION"] = f"{self.partition_prefix}_{self.index}"
        os.environ["GZ_PARTITION"] = os.environ["IGN_PARTITION"]
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.default_int_handler)
        import torch
        from pig_pen_env import PigPenEnv
        from patrol_training_env import PatrolTrainingEnv
        from training_monitor import TrainingMonitor
        torch.set_num_threads(1)
        raw = (PatrolTrainingEnv(config_file=self.config) if self.mode == "patrol"
               else PigPenEnv(mode="train", config_file=self.config))
        metrics = ("success", "coverage", "return_rate", "failure_reason") if self.mode == "patrol" else ()
        directory = self.log_dir / f"env_{self.index}"
        directory.mkdir(parents=True, exist_ok=True)
        env = TrainingMonitor(raw, filename=str(directory / "monitor"),
                              info_keywords=metrics, worker_id=self.index)
        try:
            check_env(env, warn=False)
        except BaseException:
            env.close()
            raise
        return env


class TrainingVecEnv(SubprocVecEnv):
    """Bounded shutdown even if a worker is midway through a long FSM maneuver."""
    def __init__(self, factories):
        try:
            super().__init__(factories, start_method="spawn")
        except BaseException:
            self.close()
            raise

    def close(self):
        if getattr(self, "closed", False):
            return
        processes = getattr(self, "processes", [])
        remotes = getattr(self, "remotes", [])
        for process, remote in zip(processes, remotes):
            if process.is_alive():
                if self.waiting:
                    process.terminate()
                else:
                    try:
                        remote.send(("close", None))
                    except (OSError, EOFError):
                        process.terminate()
        deadline = time.monotonic() + 5.0
        for process in processes:
            process.join(timeout=max(0, deadline-time.monotonic()))
            if process.is_alive():
                process.terminate()
                process.join(timeout=1)
            if process.is_alive():
                process.kill()
                process.join(timeout=1)
        for remote in remotes:
            remote.close()
        self.closed = True


class ParallelProgress(BaseCallback):
    def __init__(self, initial_steps, target_steps):
        super().__init__()
        self.initial_steps, self.target_steps = initial_steps, target_steps

    def _on_training_start(self):
        self.started = self.last_report = time.monotonic()

    def _on_step(self):
        now = time.monotonic()
        if now-self.last_report >= 15:
            steps = self.num_timesteps-self.initial_steps
            elapsed = max(now-self.started, 1e-9)
            print(f"[平行總進度] {self.training_env.num_envs} 個環境共用 PPO "
                  f"| 本次 {steps:,}/{self.target_steps:,} ({100*steps/self.target_steps:.2f}%) "
                  f"| 累積 {self.num_timesteps:,} 步 | {steps/elapsed:.2f} 決策/秒", flush=True)
            self.last_report = now
        return True
