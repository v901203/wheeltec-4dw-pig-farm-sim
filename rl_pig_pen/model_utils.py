"""Keep incompatible waypoint policies out of automatic model selection."""

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
CHECKPOINT_DIR = ROOT / "checkpoints_corridor38"
LOG_DIR = ROOT / "logs_corridor38"


def latest_checkpoint(directory=CHECKPOINT_DIR):
    candidates = list(Path(directory).glob("ppo_corridor38_*.zip"))
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


def validate_model(model, observation_space, action_space):
    for name, expected in (("observation_space", observation_space), ("action_space", action_space)):
        actual = getattr(model, name)
        if (actual.shape != expected.shape or
                not np.array_equal(actual.low, expected.low) or
                not np.array_equal(actual.high, expected.high)):
            raise ValueError(f"Incompatible {name}: load a new 38-D corridor policy; "
                             "39-D waypoint models cannot be resumed directly.")
