"""Behavior policy for DAgger; supports the preserved 151-D v5 baseline explicitly."""
import hashlib
import json
import numpy as np
from stable_baselines3 import PPO
from learning_common import model_path, model_spec, contract
from policy_context import RAW_OBS_SIZE


class BehaviorPolicy:
    def __init__(self, path):
        path = model_path(path)
        manifest = json.loads(path.with_suffix(".json").read_text())
        saved = manifest["environment"]
        self.legacy = saved.get("schema") == "pig_pen_fsm_junction_bc_v5"
        if self.legacy:
            current = contract()
            # Only the known v5 raw-feature adapter is allowed. Geometry, actions,
            # teacher, map and robot remain identical; added context is stripped.
            keys = ("world_sha256", "config_sha256", "fsm_sha256", "localization_sha256",
                    "fsm_teacher_sha256", "navigation_sha256", "geometry_sha256",
                    "robot_sha256", "action_mapping", "dt")
            if any(saved.get(k) != current[k] for k in keys):
                raise ValueError("v5 behavior checkpoint has incompatible sensor/action/teacher configuration")
            self.spec = manifest["policy"]
            if self.spec["observation_size"] != RAW_OBS_SIZE*self.spec["history"]:
                raise ValueError("Invalid v5 observation layout")
        else:
            self.spec = model_spec(path)
        self.model = PPO.load(path, device="cpu")
        self.provenance = {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                           "legacy_raw_adapter": self.legacy, "tasks": self.spec["tasks"]}

    def _select(self, observation):
        return observation[:RAW_OBS_SIZE].copy() if self.legacy else observation.copy()

    def reset(self, observation):
        self.frames = [self._select(observation) for _ in range(self.spec["history"])]

    def observe(self, observation):
        self.frames = self.frames[1:] + [self._select(observation)]

    def action(self):
        action, _ = self.model.predict(np.concatenate(self.frames), deterministic=True)
        return action.astype(np.float32)
