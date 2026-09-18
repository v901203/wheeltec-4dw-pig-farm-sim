"""Missing inputs must fail before creating a robot environment or policy."""

from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from model_utils import resolve_checkpoint
import train_bc
import train_ppo


class TrainingInputTests(unittest.TestCase):
    def test_checkpoint_accepts_explicit_or_omitted_zip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.zip"
            path.touch()
            self.assertEqual(resolve_checkpoint(path), path)
            self.assertEqual(resolve_checkpoint(path.with_suffix("")), path)

    def test_missing_resume_fails_before_ros_or_output_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stderr = io.StringIO()
            argv = ["train_ppo.py", "--resume", str(root / "missing.zip"),
                    "--checkpoint-dir", str(root / "checkpoints"), "--log-dir", str(root / "logs")]
            with patch.object(sys, "argv", argv), patch.object(train_ppo, "PigPenEnv") as env, \
                    redirect_stderr(stderr), self.assertRaises(SystemExit) as error:
                train_ppo.main()
            self.assertEqual(error.exception.code, 2)
            env.assert_not_called()
            self.assertFalse((root / "checkpoints").exists())
            self.assertFalse((root / "logs").exists())
            self.assertIn("train_bc.py", stderr.getvalue())
            self.assertIn(str(root / "missing.zip"), stderr.getvalue())
            self.assertNotIn(".zip.zip", stderr.getvalue())

    def test_insufficient_demos_does_not_create_policy(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "bc.zip"
            stderr = io.StringIO()
            data = (np.zeros((13, 38)), np.zeros((13, 2)), np.array(["session"] * 13))
            with patch.object(sys, "argv", ["train_bc.py", "--output", str(output)]), \
                    patch.object(train_bc, "load_demonstrations", return_value=data), \
                    patch.object(train_bc, "PPO") as model, redirect_stderr(stderr), \
                    self.assertRaises(SystemExit) as error:
                train_bc.main()
            self.assertEqual(error.exception.code, 2)
            self.assertIn("Only 13 samples", stderr.getvalue())
            self.assertIn("No BC checkpoint was created", stderr.getvalue())
            model.assert_not_called()
            self.assertFalse(output.exists())

    def test_patrol_training_saves_policy_on_rollout_failure_and_closes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            argv = ["train_ppo.py", "--mode", "patrol", "--timesteps", "32",
                    "--checkpoint-dir", str(root / "checkpoints"), "--log-dir", str(root / "logs")]
            with patch.object(sys, "argv", argv), \
                    patch.object(train_ppo, "PatrolTrainingEnv") as patrol, \
                    patch.object(train_ppo, "Monitor") as monitor, \
                    patch.object(train_ppo, "PPO") as ppo, \
                    patch.object(train_ppo, "check_env"):
                ppo.return_value.learn.side_effect = RuntimeError("reset sensor timeout")
                with self.assertRaisesRegex(RuntimeError, "reset sensor timeout"):
                    train_ppo.main()
                patrol.assert_called_once()
                self.assertIn("return_rate", monitor.call_args.kwargs["info_keywords"])
                ppo.return_value.save.assert_called_once_with(str(root / "checkpoints/ppo_corridor38_interrupted"))
                monitor.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
