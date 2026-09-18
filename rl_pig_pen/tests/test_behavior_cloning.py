from pathlib import Path
import sys
import tempfile
import unittest

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from model_utils import validate_model
from policy_config import PPO_PARAMS, policy_spaces
from train_bc import OfflineSpaces, fit_actor


class SyntheticRollout(gym.Env):
    """Only tests SB3 checkpoint compatibility, not driving performance."""
    def __init__(self):
        self.observation_space, self.action_space = policy_spaces()
        self.obs = np.full(38, 0.5, dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        return self.obs.copy(), {}

    def step(self, action):
        self.steps += 1
        return self.obs.copy(), 0.0, False, self.steps >= 8, {}


class BehaviorCloningTests(unittest.TestCase):
    def test_fit_generalizes_and_checkpoint_can_continue_ppo(self):
        torch.set_num_threads(1)
        rng = np.random.default_rng(2)
        obs = rng.uniform(0, 1, (400, 38)).astype(np.float32)
        obs[:, 36] = rng.uniform(-1, 1, 400)
        actions = np.column_stack((0.2+0.1*obs[:, 37], 0.4*obs[:, 36])).astype(np.float32)
        params = dict(PPO_PARAMS, n_steps=16, batch_size=8, n_epochs=1, verbose=0)
        model = PPO("MlpPolicy", OfflineSpaces(), device="cpu", seed=2, **params)
        critic_before = {key: value.clone() for key, value in model.policy.value_net.state_dict().items()}
        report = fit_actor(model, obs, actions, np.arange(300), np.arange(300, 400),
                           epochs=20, patience=10, batch_size=64, seed=2)
        self.assertGreater(report["best_epoch"], 0)
        self.assertLess(report["best_validation_loss"], report["baseline_validation_loss"]*0.2)
        for key, value in model.policy.value_net.state_dict().items():
            torch.testing.assert_close(value, critic_before[key])
        np.testing.assert_allclose(model.policy.log_std.detach().exp().numpy(), [0.1, 0.2], rtol=1e-6)
        before, _ = model.predict(obs[:10], deterministic=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bc.zip"
            model.save(path)
            loaded = PPO.load(path, device="cpu")
            env = SyntheticRollout()
            validate_model(loaded, env.observation_space, env.action_space)
            after, _ = loaded.predict(obs[:10], deterministic=True)
            np.testing.assert_allclose(after, before)
            loaded.set_env(env)
            loaded.learn(total_timesteps=32, reset_num_timesteps=False)
            self.assertEqual(loaded.num_timesteps, 32)


if __name__ == "__main__":
    unittest.main()
