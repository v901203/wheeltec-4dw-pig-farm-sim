import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tempfile
import unittest
from unittest.mock import patch
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from training_checkpoints import ValidationCheckpoints, validation_score


class Route(gym.Env):
    observation_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)
    action_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)
    fixed_task = 'a'

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        return np.zeros(2, dtype=np.float32), {}

    def step(self, action):
        self.steps += 1
        done = self.steps == 3
        return np.zeros(2, dtype=np.float32), 1., done, False, dict(
            success=True, collision=False, sim_seconds=self.steps*.2, route_state='DONE')


class CheckpointTests(unittest.TestCase):
    def test_worst_task_has_priority_over_speed(self):
        def rows(a, b):
            return [dict(task=t, success=s, collision=False, seconds=1., interventions=0)
                    for t, successes in [('a', a), ('b', b)] for s in successes]
        self.assertGreater(validation_score(rows([True, False], [True, False])),
                           validation_score(rows([True, True], [False, False])))

    def test_validation_does_not_add_training_steps_or_monitor_episodes(self):
        raw = Route()
        with tempfile.TemporaryDirectory() as directory:
            model = PPO('MlpPolicy', Monitor(raw), n_steps=4, batch_size=4, n_epochs=1,
                        policy_kwargs=dict(net_arch=[8]), device='cpu')
            callback = ValidationCheckpoints(directory, {'tasks':['a','b'], 'history':1}, raw,
                                              interval=4, episodes=1)
            # History dimensions in the real environment are larger; this test
            # isolates callback sequencing using the same observation shape.
            with patch('training_checkpoints.HistoryObservation', lambda env, history: env), \
                 patch('training_checkpoints.save_model') as save:
                model.learn(8, callback=callback)
            self.assertEqual(model.num_timesteps, 8)
            self.assertEqual(len(model.env.envs[0].get_episode_rewards()), 2)
            self.assertTrue((Path(directory)/'best_evaluation.json').exists())
            self.assertEqual(len((Path(directory)/'validation.jsonl').read_text().splitlines()), 3)
            self.assertGreaterEqual(save.call_count, 4)

if __name__ == '__main__':
    unittest.main()
