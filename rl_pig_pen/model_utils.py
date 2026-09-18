"""Keep incompatible waypoint policies out of automatic model selection."""

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
CHECKPOINT_DIR = ROOT / "checkpoints_corridor38"
LOG_DIR = ROOT / "logs_corridor38"
PATROL_MODEL_NAME = "ppo_patrol49_v2"
PATROL_CHECKPOINT_DIR = ROOT / "checkpoints_patrol49_v2"
PATROL_LOG_DIR = ROOT / "logs_patrol49_v2"
PATROL_POLICY_VERSION = "lidar49-paired-walls-corridor-progress-v2"


def latest_checkpoint(directory=CHECKPOINT_DIR, prefix="ppo_corridor38"):
    directory = Path(directory)
    candidates = list(directory.glob(f"{prefix}_*.zip"))
    return max(candidates, key=lambda p: p.stat().st_mtime_ns) if candidates else None


def resolve_checkpoint(path):
    """Match SB3's optional .zip suffix without retrying .zip.zip."""
    path = Path(path).expanduser()
    if path.is_file():
        return path
    if path.suffix != ".zip":
        zipped = Path(str(path) + ".zip")
        if zipped.is_file():
            return zipped
    raise FileNotFoundError(f"Checkpoint does not exist: {path.resolve()}")


def validate_patrol_version(model):
    if getattr(model, "navigation_version", None) != PATROL_POLICY_VERSION:
        raise ValueError("Incompatible patrol checkpoint: paired-wall perception and rewards require "
                         "a new patrol49_v2 model. Remove --resume and start in checkpoints_patrol49_v2; "
                         "old 49-D weights have different feature/reward semantics.")


def validate_model(model, observation_space, action_space):
    for name, expected in (("observation_space", observation_space), ("action_space", action_space)):
        actual = getattr(model, name)
        if (actual.shape != expected.shape or
                not np.array_equal(actual.low, expected.low) or
                not np.array_equal(actual.high, expected.high)):
            raise ValueError(f"Incompatible {name}: this checkpoint does not match the current policy space.")
