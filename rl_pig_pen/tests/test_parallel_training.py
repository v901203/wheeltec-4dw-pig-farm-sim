from pathlib import Path
import os
import sys
import tempfile
import unittest
from unittest.mock import patch, Mock

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from parallel_training import WorkerFactory, TrainingVecEnv
from policy_config import policy_spaces


class ParallelToy(gym.Env):
    """Fast ROS-free rollouts to verify SB3 buffer sizing and worker teardown."""
    def __init__(self):
        self.observation_space, self.action_space = policy_spaces()
        self.steps = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        return np.zeros(38, dtype=np.float32), {}

    def step(self, action):
        self.steps += 1
        return np.zeros(38, dtype=np.float32), float(action[0]), False, self.steps >= 4, {}


class ParallelTrainingTests(unittest.TestCase):
    def test_worker_sets_ros_and_gazebo_isolation_before_constructor(self):
        seen = []
        def construct(**kwargs):
            seen.append((os.environ['ROS_DOMAIN_ID'], os.environ['IGN_PARTITION'], os.environ['GZ_PARTITION']))
            return Mock()
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ), \
                patch('parallel_training.signal.signal'), \
                patch('parallel_training.check_env'), \
                patch('patrol_training_env.PatrolTrainingEnv', side_effect=construct), \
                patch('training_monitor.TrainingMonitor') as monitor:
            for index in (0, 1):
                WorkerFactory(index, 40, 'test_run', 'patrol', Path('cfg.json'), Path(directory))()
            self.assertEqual(seen, [('40', 'test_run_0', 'test_run_0'), ('41', 'test_run_1', 'test_run_1')])
            self.assertNotEqual(monitor.call_args_list[0].kwargs['filename'], monitor.call_args_list[1].kwargs['filename'])

    def test_single_checkpoint_resumes_with_two_workers_and_updates(self):
        torch.set_num_threads(1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'policy'
            original = PPO('MlpPolicy', ParallelToy(), n_steps=8, batch_size=8, n_epochs=1, device='cpu')
            original.save(path)
            env = TrainingVecEnv([ParallelToy, ParallelToy])
            try:
                model = PPO.load(path, env=env, n_steps=8, device='cpu')
                self.assertEqual(model.n_envs, 2)
                self.assertEqual(model.rollout_buffer.n_envs, 2)
                before = {key: value.clone() for key, value in model.policy.state_dict().items()}
                model.learn(32)
                self.assertEqual(model.num_timesteps, 32)
                self.assertTrue(any(not torch.equal(value, before[key]) for key, value in model.policy.state_dict().items()))
            finally:
                env.close()
            self.assertTrue(all(not process.is_alive() for process in env.processes))


if __name__ == '__main__':
    unittest.main()
