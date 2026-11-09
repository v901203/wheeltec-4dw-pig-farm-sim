"""PPO must respect the geometry gate and fit its critic before changing BC."""
from pathlib import Path
import sys
import tempfile
import json
import io
import contextlib
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from train import CriticWarmup, TaskRoundRobin, TrainingDiagnostics, main
from lidar_patrol_controller import TraditionalPatrolController
from navigation import load_config
from learning_common import contract
from tasks import TASKS


class ToyEnv(gym.Env):
    observation_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)
    action_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        return np.array([.1, -.1], dtype=np.float32), {}

    def step(self, action):
        self.steps += 1
        return np.array([.1, -.1], dtype=np.float32), float(.5+.1*action[0]), False, self.steps == 8, {}


class PPOStartTests(unittest.TestCase):
    def test_training_failure_snapshot_preserves_transition_and_cause(self):
        class FailingRoute(ToyEnv):
            def __init__(self):
                self.task = SimpleNamespace(name="main_to_left")
                self.scan = SimpleNamespace(ranges=np.ones(8), angle_min=-3., angle_increment=.1,
                                            range_min=.1, range_max=25.)
                self.scan_stamp = 1.
                self.pose = np.zeros(3)
                self.cached_context = np.zeros(10)
                self.teacher = SimpleNamespace(controller=TraditionalPatrolController(load_config()),
                                               status=lambda: {"state": "CENTER", "failure": "test_failure"})

            def step(self, action):
                return np.array([.1, -.1], dtype=np.float32), 1.25, True, False, {"success": False}

        with tempfile.TemporaryDirectory() as temp:
            e = TrainingDiagnostics(FailingRoute(), Path(temp))
            e.reset(seed=3)
            observation, reward, done, truncated, info = e.step(np.array([.2, .3]))
            self.assertEqual(reward, 1.25)
            self.assertTrue(done)
            self.assertFalse(truncated)
            np.testing.assert_allclose(observation, [.1, -.1])
            self.assertEqual(info, {"success": False})
            e.close()
            row = json.loads((Path(temp)/"episode_details.jsonl").read_text())
            self.assertEqual(row["fsm_status"]["failure"], "test_failure")
            self.assertTrue((Path(temp)/"failure_geometry/scans_00000.npz").exists())

    def test_curriculum_visits_all_tasks_before_repeating(self):
        class Route(ToyEnv):
            def reset(self, *, seed=None, options=None):
                observation, info = super().reset(seed=seed, options=options)
                return observation, {"task": self.fixed_task}
        e = TaskRoundRobin(Route(), ["left", "reverse", "right"])
        self.assertEqual([e.reset(seed=7)[1]["task"] for _ in range(7)],
                         ["left", "reverse", "right", "left", "reverse", "right", "left"])

    def test_first_rollout_updates_value_only_then_actor_can_learn(self):
        m = PPO("MlpPolicy", ToyEnv(), n_steps=8, batch_size=4, n_epochs=1,
                learning_rate=1e-3, seed=0, device="cpu",
                policy_kwargs={"net_arch": {"pi": [8], "vf": [8]}})
        actor = list(m.policy.mlp_extractor.policy_net.parameters())+list(m.policy.action_net.parameters())
        before = [p.detach().clone() for p in actor]
        value_before = [p.detach().clone() for p in m.policy.mlp_extractor.value_net.parameters()]
        log_std = m.policy.log_std.detach().clone()
        m.learn(8, callback=CriticWarmup(1))
        self.assertTrue(all(torch.equal(a, b) for a, b in zip(before, actor)))
        self.assertTrue(torch.equal(log_std, m.policy.log_std))
        self.assertTrue(any(not torch.equal(a, b) for a, b in
                            zip(value_before, m.policy.mlp_extractor.value_net.parameters())))
        self.assertTrue(all(p.requires_grad for p in actor))
        m.learn(8, callback=CriticWarmup(0))
        self.assertTrue(any(not torch.equal(a, b) for a, b in zip(before, actor)))
        m.env.close()

    def test_boosted_critic_warmup_preserves_resumed_actor_and_saves_checkpoints(self):
        m = PPO("MlpPolicy", ToyEnv(), n_steps=8, batch_size=4, n_epochs=1,
                learning_rate=1e-4, seed=0, device="cpu",
                policy_kwargs={"net_arch": {"pi": [8], "vf": [8]}})
        m.learn(8, callback=CriticWarmup(0))
        actor = list(m.policy.mlp_extractor.policy_net.parameters())+list(m.policy.action_net.parameters())
        before = [p.detach().clone() for p in actor]
        std_before = m.policy.log_std.detach().clone()
        with tempfile.TemporaryDirectory() as temp:
            callback = CriticWarmup(2, Path(temp), {"test_fixture": True},
                                    critic_learning_rate=3e-4, critic_epochs=3)
            m.learn(16, callback=callback, reset_num_timesteps=False)
            saved = PPO.load(Path(temp)/"latest.zip", device="cpu")
            self.assertAlmostEqual(saved.policy.optimizer.param_groups[0]["lr"], 3e-4)
            # Loading rebuilds the scheduler from the base learning_rate;
            # warmup must not become the actor's permanent learning rate.
            self.assertAlmostEqual(saved.lr_schedule(1.), 1e-4)
            progress = json.loads((Path(temp)/"progress.json").read_text())
            self.assertEqual(progress["timesteps"], 24)
            self.assertEqual(progress["additional_timesteps"], 16)
        self.assertTrue(all(torch.equal(a, b) for a, b in zip(before, actor)))
        self.assertTrue(torch.equal(std_before, m.policy.log_std))
        self.assertEqual(m.n_epochs, 1)
        self.assertAlmostEqual(m.lr_schedule(1.), 1e-4)
        m.env.close()

    def test_failed_geometry_gate_cannot_construct_simulator_or_train(self):
        with tempfile.TemporaryDirectory() as temp:
            gate = Path(temp)/"gate.json"
            checkpoint = Path(temp)/"dummy.zip"
            checkpoint.write_bytes(b"not used as a real checkpoint")
            for passed in (False, True):
                gate.write_text(json.dumps({"passed": passed, "contract": contract(),
                    "model_sha256": "wrong-model", "minimum_episodes_per_task": 2,
                    "centre_error_limit_m": .1, "unfinished_geometry_recoveries": [],
                    "tasks": {t.name: {"episodes": 2, "geometry_frames": 10,
                                       "wrong_centre_frames": 0, "max_centre_error_m": .01} for t in TASKS}}))
                with self.subTest(passed=passed), \
                        patch.object(sys, "argv", ["train.py", "--init-model", str(checkpoint), "--geometry-gate", str(gate)]), \
                        patch("train.check_model", return_value=checkpoint), \
                        patch("train.JunctionEnv") as env, contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as result:
                        main()
                    self.assertEqual(result.exception.code, 2)
                    env.assert_not_called()
