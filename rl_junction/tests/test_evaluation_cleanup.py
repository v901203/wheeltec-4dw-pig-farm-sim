"""A disk failure in diagnostics must not bypass the environment's zero stop."""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate


class EvaluationCleanupTests(unittest.TestCase):
    def test_journal_close_failure_still_closes_environment(self):
        with tempfile.TemporaryDirectory() as temp:
            env = Mock()
            env.reset.side_effect = RuntimeError("reset failed")
            journal = Mock()
            journal.close.side_effect = OSError("disk full")
            with patch.object(sys, "argv", ["evaluate.py", "--teacher", "--task", "main_to_left",
                                           "--episodes-per-task", "1", "--geometry-diagnostics", "--output", temp]), \
                    patch.object(evaluate, "JunctionEnv", return_value=env), \
                    patch.object(evaluate, "GeometryRecorder", return_value=journal), \
                    patch.object(evaluate.signal, "signal"), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(OSError):
                    evaluate.main()
            env.close.assert_called_once()
