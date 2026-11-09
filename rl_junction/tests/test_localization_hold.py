"""Localization holds stop actions, preserve the route and remain bounded."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from junction_env import HybridJunctionEnv, RouteEnv
from tasks import TASKS


class LocalizationHoldTests(TestCase):
    def env(self):
        e = HybridJunctionEnv.__new__(HybridJunctionEnv)
        e.active = True
        e.task = TASKS[0]
        e.pose = np.asarray([0., -6.18, np.pi/2])
        e.dt = .2
        e.skill_steps = e.localization_hold_steps = e.localization_recoveries = 0
        e.skill_max_steps = 500
        e.was_relocalizing = False
        e.label_valid = False
        e.cached_context = np.zeros(10)
        e.last_step_perturbation = None
        e._prepare_decision = Mock()
        e._publish = Mock()
        e._drain = Mock()
        e._policy_observation = lambda raw: np.zeros(161)
        c = SimpleNamespace(relocalization={"reason": "junction_geometry_conflict"})
        e.teacher = SimpleNamespace(controller=c, status=Mock())
        return e

    def parent(self):
        return (np.zeros(151), 0., False, False, dict(
            collision=False, off_route=False, wrong_heading=False, success=False,
            safety_blocked=False))

    def status(self, localizing=True):
        return dict(state="CENTER", failure=None, obstacle=False, relocalizing=localizing)

    def test_hold_zeros_policy_motion_without_terminating(self):
        e = self.env()
        e.teacher.status.return_value = self.status()
        with patch.object(RouteEnv, "step", return_value=self.parent()) as parent:
            _, _, terminated, truncated, info = e.step([1., 1.])
        np.testing.assert_array_equal(parent.call_args.args[0], [0., 0.])
        self.assertFalse(terminated or truncated)
        self.assertTrue(info["localization_hold"])
        self.assertEqual(info["localization_recoveries"], 1)

    def test_hold_timeout_terminates_and_publishes_zero(self):
        e = self.env()
        e.teacher.status.return_value = self.status()
        e.localization_hold_steps = e.LOCALIZATION_HOLD_STEPS
        with patch.object(RouteEnv, "step", return_value=self.parent()):
            _, _, terminated, _, info = e.step([1., 1.])
        self.assertTrue(terminated)
        self.assertTrue(info["localization_recovery_failed"])
        e._publish.assert_called_with((0., 0.))

    def test_recovery_resumes_same_stage_and_clears_wait_budget(self):
        e = self.env()
        e.was_relocalizing = True
        e.localization_hold_steps = 5
        e.teacher.status.side_effect = [self.status(), self.status(False)]
        e.teacher.controller.relocalization = None
        with patch.object(RouteEnv, "step", return_value=self.parent()):
            _, _, terminated, _, info = e.step([1., 1.])
        self.assertFalse(terminated)
        self.assertTrue(info["localization_recovered"])
        self.assertEqual(info["route_state"], "CENTER")
        self.assertEqual(e.localization_hold_steps, 0)
