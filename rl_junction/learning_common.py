"""Shared, ROS-free data/model contract. No simulator starts on import."""
import hashlib
import json
from pathlib import Path
import gymnasium as gym
import numpy as np
from tasks import TASKS, WORLD_FILE, WORLD_NAME
from policy_context import OBS_SIZE, RAW_OBS_SIZE, DISTANCE_SCALE

ROOT = Path(__file__).resolve().parents[1]
BASE_OBS_SIZE = OBS_SIZE
DEFAULT_HISTORY = 4


def contract():
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        "schema": "pig_pen_fsm_junction_bc_v6", "world": WORLD_NAME,
        "policy_scope": "junction acquisition/slowdown through FSM-confirmed corridor handoff",
        "world_sha256": digest(WORLD_FILE),
        "config_sha256": digest(ROOT / "rl_pig_pen/patrol_config.json"),
        "env_sha256": digest(ROOT / "rl_junction/gazebo_env.py"),
        "junction_env_sha256": digest(ROOT / "rl_junction/junction_env.py"),
        "policy_context_sha256": digest(ROOT / "rl_junction/policy_context.py"),
        "recovery_sha256": digest(ROOT / "rl_junction/recovery.py"),
        "scene_prepare_sha256": digest(ROOT / "rl_junction/prepare_scene.py"),
        "fsm_sha256": digest(ROOT / "rl_pig_pen/lidar_patrol_controller.py"),
        "localization_sha256": digest(ROOT / "rl_pig_pen/junction_localization.py"),
        "fsm_teacher_sha256": digest(ROOT / "rl_junction/fsm_teacher.py"),
        "tasks_sha256": digest(ROOT / "rl_junction/tasks.py"),
        "navigation_sha256": digest(ROOT / "rl_pig_pen/navigation.py"),
        "geometry_sha256": digest(ROOT / "rl_pig_pen/lidar_geometry.py"),
        "robot_sha256": digest(ROOT / "turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf"),
        "observation_size": BASE_OBS_SIZE, "raw_observation_size": RAW_OBS_SIZE,
        "context": "phase4,centre_xy/2m,heading/pi,centre_valid,heading_valid,repositioning",
        "centre_distance_scale": DISTANCE_SCALE, "action_size": 2, "dt": .2,
        "tasks": [t.name for t in TASKS],
        "action_mapping": "v=.2*a0; reverse v=-.1*(a0+1); w=.6*a1",
    }


def model_path(path):
    path = Path(path)
    return path if path.suffix == ".zip" else Path(str(path)+".zip")


def save_model(model, path, policy):
    path = model_path(path)
    model.save(path)
    path.with_suffix(".json").write_text(json.dumps({"environment": contract(),
                                                   "policy": policy}, indent=2))


def check_model(path):
    path = model_path(path)
    model_spec(path)
    return path


def model_spec(path):
    manifest = json.loads(model_path(path).with_suffix(".json").read_text())
    if manifest.get("environment") != contract():
        raise ValueError("Model contract differs from current map/config/environment; recollect/retrain")
    spec = manifest["policy"]
    if (spec["architecture"] != "mlp" or spec["history"] < 1 or
            spec["observation_size"] != BASE_OBS_SIZE*spec["history"] or
            not spec["tasks"] or not set(spec["tasks"]).issubset(contract()["tasks"])):
        raise ValueError("Invalid model policy specification")
    return spec


def history_windows(observations, history):
    """Oldest to newest; repeat first observation at episode start, never cross resets."""
    if history < 1:
        raise ValueError("history must be positive")
    indices = np.maximum(np.arange(len(observations))[:, None] + np.arange(1-history, 1), 0)
    return observations[indices].reshape(len(observations), -1).astype(np.float32)


class HistoryObservation(gym.Wrapper):
    def __init__(self, env, history):
        super().__init__(env)
        if history < 1:
            raise ValueError("history must be positive")
        self.history = history
        self.observation_space = gym.spaces.Box(-1., 1., (BASE_OBS_SIZE*history,), dtype=np.float32)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self.frames = [obs.copy() for _ in range(self.history)]
        return np.concatenate(self.frames), info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.frames = self.frames[1:] + [obs.copy()]
        return np.concatenate(self.frames), reward, terminated, truncated, info


def encode_command(v, w, task):
    """Inverse physical-to-policy mapping, including reverse-only zero = -1."""
    if task == "left_to_right_reverse":
        if v > 1e-6:
            raise ValueError("Reverse task cannot demonstrate forward motion")
        a = [-v/.1-1., w/.6]
    else:
        a = [v/.2, w/.6]
    if not np.isfinite(a).all() or np.max(np.abs(a)) > 1.00001:
        raise ValueError("Demonstration command outside policy bounds")
    return np.clip(a, -1., 1.).astype(np.float32)


class OfflineSpaces(gym.Env):
    """Construct a PPO policy offline; intentionally cannot collect rollouts."""
    action_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)

    def __init__(self, history=DEFAULT_HISTORY):
        self.observation_space = gym.spaces.Box(-1., 1., (BASE_OBS_SIZE*history,), dtype=np.float32)

    def reset(self, **kwargs):
        raise RuntimeError("Offline BC only: no simulator attached")

    def step(self, action):
        raise RuntimeError("Offline BC only: no simulator attached")
