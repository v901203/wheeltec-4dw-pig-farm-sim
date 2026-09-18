"""Versioned demonstration storage and filtering, independent of ROS."""

from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import uuid

import numpy as np

from navigation import MAX_ANG, MAX_LIN, SensorFault, collision_detected, safe_command, scan_features
from policy_config import policy_spaces

FORMAT_VERSION = "corridor38-command-v1"
DEMO_DIR = Path(__file__).with_name("demonstrations")


def demonstration_sample(scan, action, cfg):
    """Filter manual corridor actions; never clip unsupported expert commands."""
    action = np.asarray(action, dtype=np.float32)
    if action.shape != (2,) or not np.all(np.isfinite(action)):
        return None, "invalid_action"
    if not (0 <= action[0] <= MAX_LIN and abs(action[1]) <= MAX_ANG):
        return None, "unsupported_action"
    try:
        features = scan_features(scan, cfg)
    except SensorFault:
        return None, "invalid_scan"
    if collision_detected(features, cfg):
        return None, "collision"
    if max(features.left_edge_m, features.right_edge_m) > cfg.open_distance:
        return None, "junction_or_open_space"
    if action[0] < 0.02 and abs(action[1]) > 0.05:
        return None, "turn_in_place"
    if action[0] < 0.02 and abs(action[1]) <= 0.05 and features.front_m > 0.8:
        return None, "idle"
    _, blocked = safe_command(action, features, cfg)
    if blocked:
        return None, "unsafe_command"
    return (features.observation, action), "accepted"


class DemoWriter:
    def __init__(self, output_dir, cfg, chunk_size=500, extra_metadata=None):
        if chunk_size < 1:
            raise ValueError("chunk_size must be positive")
        session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8]
        self.directory = Path(output_dir) / session
        self.directory.mkdir(parents=True, exist_ok=False)
        self.metadata = dict(format=FORMAT_VERSION, session=session, config=asdict(cfg),
                             **(extra_metadata or {}))
        self.chunk_size = chunk_size
        self.rows = []
        self.chunks = 0
        self.saved = 0
        self.counts = Counter()
        self._summary()

    def add(self, observation, action, scan_stamp, command_time, scan_age):
        self.rows.append((observation.copy(), action.copy(), scan_stamp, command_time, scan_age))
        self.counts["accepted"] += 1
        if len(self.rows) >= self.chunk_size:
            self.flush()

    def _summary(self):
        path = self.directory / "session.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(dict(self.metadata, saved_samples=self.saved,
                                             pending_samples=len(self.rows), counts=dict(self.counts)),
                                         indent=2), encoding="utf-8")
        os.replace(temporary, path)

    def flush(self):
        if self.rows:
            obs, actions, stamps, commands, ages = zip(*self.rows)
            path = self.directory / f"chunk_{self.chunks:06d}.npz"
            temporary = path.with_suffix(".tmp")
            with temporary.open("xb") as handle:
                np.savez_compressed(handle, observations=np.asarray(obs, dtype=np.float32),
                                    actions=np.asarray(actions, dtype=np.float32),
                                    scan_stamps=np.asarray(stamps, dtype=np.float64),
                                    command_times=np.asarray(commands, dtype=np.float64),
                                    scan_age_s=np.asarray(ages, dtype=np.float64),
                                    metadata=np.array(json.dumps(self.metadata)))
            os.replace(temporary, path)
            self.saved += len(self.rows)
            self.chunks += 1
            self.rows.clear()
        self._summary()


def load_demonstrations(data_dir, cfg):
    files = sorted(Path(data_dir).rglob("chunk_*.npz"))
    if not files:
        raise ValueError(f"No demonstration chunks found in {data_dir}")
    observation_space, action_space = policy_spaces()
    observations, actions, groups = [], [], []
    for path in files:
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(str(archive["metadata"].item()))
            if metadata.get("format") != FORMAT_VERSION:
                raise ValueError(f"Incompatible demonstration format: {path}")
            if metadata["config"]["lidar_yaw"] != cfg.lidar_yaw:
                raise ValueError(f"LiDAR mounting differs from current config: {path}")
            obs, act = archive["observations"], archive["actions"]
            if (obs.ndim != 2 or obs.shape[1] != 38 or act.shape != (len(obs), 2) or
                    len(obs) == 0 or not np.all(np.isfinite(obs)) or not np.all(np.isfinite(act)) or
                    np.any(obs < observation_space.low) or np.any(obs > observation_space.high) or
                    np.any(act < action_space.low) or np.any(act > action_space.high)):
                raise ValueError(f"Invalid observations/actions: {path}")
            observations.append(obs.astype(np.float32))
            actions.append(act.astype(np.float32))
            groups.extend([metadata["session"]] * len(obs))
    return np.concatenate(observations), np.concatenate(actions), np.asarray(groups)


def split_demonstrations(groups, seed=0):
    """Hold out whole sessions, or the last 20% with a 12-sample gap."""
    n = len(groups)
    if n < 100:
        raise ValueError(f"Only {n} samples; record at least 100 (prefer several sessions)")
    sessions = np.unique(groups)
    if len(sessions) >= 2:
        holdout = np.random.default_rng(seed).permutation(sessions)[:max(1, round(len(sessions)*0.2))]
        mask = np.isin(groups, holdout)
        train, validation = np.flatnonzero(~mask), np.flatnonzero(mask)
        strategy = "held-out sessions"
    else:
        boundary = int(n*0.8)
        train, validation = np.arange(boundary-12), np.arange(boundary, n)
        strategy = "last 20% of one session, 12-sample gap"
    if len(train) < 32 or len(validation) < 16:
        raise ValueError("Not enough samples after splitting; record longer independent sessions")
    return train, validation, strategy
