import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import unittest
import tempfile
from types import SimpleNamespace
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from phase_training import PhaseTrainingReward, AnchoredPPO
from tasks import TASKS, CENTRE_Y


class FakeRoute(gym.Env):
    observation_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)
    action_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)
    def reset(self, **kwargs):
        self.pose = np.array([0., CENTRE_Y-.3, np.pi/2])
        self.cached_context = np.array([0., 1., 0., 0.])
        self.label_valid = True
        self.task = TASKS[0]
        self.teacher = SimpleNamespace(status=lambda:dict(relocalizing=False))
        self.move = 0.
        self.hold = False
        self.active = True
        return np.zeros(2, dtype=np.float32), {}
    def step(self, action):
        self.pose[1] += self.move
        return np.zeros(2, dtype=np.float32), 123., False, False, dict(
            success=False, phase_id=1, teacher_intervened=self.hold,
            safety_blocked=False, executed_action=[0., 0.])
    def _publish(self, command): self.command = command
    def _drain(self): pass


class PhaseTests(unittest.TestCase):
    def test_current_centre_progress_is_rewarded_and_retreat_penalized(self):
        env = PhaseTrainingReward(FakeRoute()); env.reset()
        env.unwrapped.move = .02
        info = env.step([0, 0])[-1]
        self.assertAlmostEqual(info['reward_components']['progress'], .16)
        env.unwrapped.move = -.02
        self.assertLess(env.step([0,0])[-1]['reward_components']['progress'], 0.)
    def test_stationary_failure_but_hold_is_exempt(self):
        env = PhaseTrainingReward(FakeRoute(), stall_steps=3); env.reset()
        env.unwrapped.hold = True
        for _ in range(5): self.assertFalse(env.step([0,0])[2])
        env.unwrapped.hold = False
        for _ in range(2): self.assertFalse(env.step([0,0])[2])
        _, _, done, truncated, info = env.step([0,0])
        self.assertTrue(done); self.assertFalse(truncated)
        self.assertTrue(info['no_progress'])
        self.assertEqual(env.unwrapped.command, (0., 0.))
    def test_turn_error_is_angle_and_not_final_diagonal_distance(self):
        task=TASKS[0]
        a=PhaseTrainingReward.error(2,np.array([0.,CENTRE_Y, np.pi/2]),task)
        b=PhaseTrainingReward.error(2,np.array([0.,CENTRE_Y, np.pi*.75]),task)
        self.assertLess(b,a)
    def test_anchor_reference_frozen_and_save_reload(self):
        # A compact Gym environment is sufficient to test optimizer serialization.
        env=gym.make('Pendulum-v1')
        model=AnchoredPPO('MlpPolicy',env,n_steps=8,batch_size=8,n_epochs=1,
                          policy_kwargs=dict(net_arch=[8]),device='cpu')
        reference=PPO('MlpPolicy',env,n_steps=8,batch_size=8,policy_kwargs=dict(net_arch=[8]),device='cpu')
        old=[p.clone() for p in reference.policy.parameters()]
        model.attach_bc(reference.policy)
        model.learn(8)
        self.assertTrue(all(torch.equal(a,b) for a,b in zip(old,reference.policy.parameters())))
        with tempfile.TemporaryDirectory() as directory:
            model.save(Path(directory)/'model')
            loaded=PPO.load(Path(directory)/'model')
            self.assertFalse(hasattr(loaded,'bc_reference'))
            self.assertEqual(loaded.num_timesteps,8)
        env.close()

if __name__=='__main__': unittest.main()
