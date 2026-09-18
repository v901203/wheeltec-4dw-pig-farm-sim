from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import gymnasium as gym
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from training_monitor import TrainingMonitor, episode_reason


class SmallEnv(gym.Env):
    observation_space = gym.spaces.Box(0, 1, (1,), dtype=np.float32)
    action_space = gym.spaces.Box(0, 1, (1,), dtype=np.float32)

    def reset(self, **kwargs):
        return np.zeros(1, dtype=np.float32), {"spawn": (0, -14, 1.57)}

    def step(self, action):
        return np.zeros(1, dtype=np.float32), -100.0, True, False, {"collision": True}


class TrainingMonitorTests(unittest.TestCase):
    def test_reason_priority_and_non_collision_endings(self):
        self.assertEqual(episode_reason({"collision": True, "success": True}), "collision")
        self.assertEqual(episode_reason({"success": True}), "success")
        self.assertEqual(episode_reason({"segment_complete": True}), "segment_complete")
        self.assertEqual(episode_reason({"segment_complete": True, "failure_reason": "stuck"}), "stuck")
        self.assertEqual(episode_reason({"failure_reason": "sensor_fault"}), "sensor_fault")

    def test_check_resets_excluded_from_training_and_reason_survives_reset(self):
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(output):
            monitor = TrainingMonitor(SmallEnv(), filename=str(Path(directory)/"monitor"))
            monitor.reset()
            monitor.step([0])
            monitor.start_training(initial_steps=200, total_steps=100)
            monitor.reset()
            monitor.step([0])
            monitor.reset()
            monitor.last_report -= 16
            monitor.heartbeat()
            self.assertEqual(monitor.completed_steps, 1)
            self.assertEqual(monitor.completed_episodes, 1)
            self.assertEqual(monitor.reason_counts["collision"], 1)
            monitor.close()
        text = output.getvalue()
        self.assertIn("啟動環境檢查（非碰撞）", text)
        self.assertIn("[重生] 原因：碰撞", text)
        self.assertIn("獎勵 -100.00", text)
        self.assertIn("1/100 (1.00%)", text)
        self.assertIn("累積 201 步", text)

    def test_fsm_heartbeat_does_not_add_policy_steps(self):
        with redirect_stdout(io.StringIO()):
            env = SmallEnv()
            monitor = TrainingMonitor(env)
            monitor.start_training(0, 100)
            monitor.last_report -= 16
            env.progress_hook()
            self.assertEqual(monitor.completed_steps, 0)
            monitor.close()


if __name__ == "__main__":
    unittest.main()
