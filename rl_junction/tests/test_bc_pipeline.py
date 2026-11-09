"""Offline regression checks for phase boundaries and temporal data integrity."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import gymnasium as gym

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning_common import history_windows, HistoryObservation, contract, encode_command, BASE_OBS_SIZE
from policy_context import decision_context
from recovery import recovery_offset
from types import SimpleNamespace
from unittest.mock import patch
from dataset import load_episodes, split_episodes, arrays, sampling_weights, label_metadata
from fsm_teacher import TEACHER_ID
from policy_features import LocalGeometryFeatures
import torch


def episode(path, seed=0, mode='normal', **changes):
    n = 6
    x = np.zeros((n, BASE_OBS_SIZE), dtype=np.float32)
    phases = np.array([0, 1, 1, 2, 2, 3])
    x[:, 151:155] = np.eye(4)[phases]
    x[:, 158:160] = 1
    x[:, 144] = 1
    actions = np.tile([0., .5], (n, 1)).astype(np.float32)
    executed = actions*[.2, .6]
    x[1:, 149:151] = actions[:-1]
    stamps = np.tile(np.arange(n)[:, None]*.2+10, (1, 4))
    meta = {"contract": contract(), "teacher": TEACHER_ID, "task": "main_to_left", "seed": seed,
            "accepted": True, "blocked": False, "steps": n, "final": {"success": True},
            "teacher_final": {"state": "LEFT_OUTBOUND"}, "policy_scope": "JUNCTION_APPROACH_TO_HANDOFF",
            "data_mode": mode, "training_only": mode == "dagger",
            "recovered": True, "perturbations": [{"case": "yaw"}] if mode == "perturbed" else [],
            "behavior_policy": {"sha256": "unit-test-fixture"} if mode == "dagger" else None}
    values = dict(metadata=np.asarray(json.dumps(meta)), observations=x, next_observations=np.vstack([x[1:], x[-1:]]),
                  actions=actions, requested=executed, executed=executed, sensor_stamps=stamps,
                  safety_blocked=np.zeros(n, dtype=bool), fsm_states=np.array(["MAIN", "CENTER", "CENTER", "TURN_LEFT", "TURN_LEFT", "ENTRY_LEFT"]),
                  fsm_next_states=np.array(["CENTER", "CENTER", "TURN_LEFT", "TURN_LEFT", "ENTRY_LEFT", "LEFT_OUTBOUND"]),
                  entry_phases=np.full(n, "JUNCTION"), ranges=np.ones((n, 72)),
                  velocities=np.zeros((n, 2)), poses=np.zeros((n, 3)), next_poses=np.zeros((n, 3)),
                  expert_commands=executed, behavior_actions=actions.copy(), label_valid=np.ones(n, dtype=bool),
                  phase_ids=phases, sample_kinds=np.full(n, {"normal": 0, "perturbed": 1, "dagger": 2}[mode]))
    values.update(changes)
    np.savez_compressed(path, **values)


class CounterEnv(gym.Env):
    observation_space = gym.spaces.Box(-1., 1., (BASE_OBS_SIZE,), dtype=np.float32)
    action_space = gym.spaces.Box(-1., 1., (2,), dtype=np.float32)

    def reset(self, **kwargs):
        self.index = 0
        return np.zeros(BASE_OBS_SIZE, dtype=np.float32), {}

    def step(self, action):
        self.index += 1
        return np.full(BASE_OBS_SIZE, self.index/10, dtype=np.float32), 0., False, False, {}


class PipelineTests(unittest.TestCase):
    def test_reverse_action_encoding_has_distinct_stop_and_reverse(self):
        np.testing.assert_allclose(encode_command(0., 0., "left_to_right_reverse"), [-1., 0.])
        np.testing.assert_allclose(encode_command(-.2, .3, "left_to_right_reverse"), [1., .5])
        with self.assertRaises(ValueError):
            encode_command(.1, 0., "left_to_right_reverse")

    def test_online_and_offline_windows_match_and_reset(self):
        env = HistoryObservation(CounterEnv(), 4)
        online = [env.reset()[0]]
        frames = [online[0][-BASE_OBS_SIZE:]]
        for _ in range(3):
            obs = env.step(np.zeros(2))[0]
            online.append(obs)
            frames.append(obs[-BASE_OBS_SIZE:])
        np.testing.assert_array_equal(online, history_windows(np.asarray(frames), 4))
        np.testing.assert_array_equal(env.reset()[0], np.zeros(BASE_OBS_SIZE*4))

    def test_split_uses_complete_episodes(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i in range(5):
                episode(Path(tmp)/f"{i}.npz", seed=i)
            data = load_episodes(tmp)
            splits = split_episodes(data, 0)
            identities = [{e.seed for e in items} for items in splits.values()]
            self.assertEqual(sum(map(len, identities)), len(set.union(*identities)))
            self.assertEqual(arrays(splits["train"], 4)[0].shape[1], BASE_OBS_SIZE*4)

    def test_rejects_duplicate_episode_seeds(self):
        with tempfile.TemporaryDirectory() as tmp:
            episode(Path(tmp)/"a.npz")
            episode(Path(tmp)/"b.npz")
            with self.assertRaisesRegex(ValueError, "duplicate"):
                load_episodes(tmp)

    def test_rejects_unrelated_states_and_wrong_start(self):
        for changes in (dict(fsm_states=np.full(6, "FINAL_MAIN")),
                        dict(fsm_states=np.full(6, "TURN_LEFT"))):
            with self.subTest(changes=list(changes)), tempfile.TemporaryDirectory() as tmp:
                episode(Path(tmp)/"bad.npz", **changes)
                with self.assertRaisesRegex(ValueError, "junction skill"):
                    load_episodes(tmp)

    def test_rejects_unsynchronized_sensors(self):
        with tempfile.TemporaryDirectory() as tmp:
            episode(Path(tmp)/"bad.npz", sensor_stamps=np.tile([1., 2., 1., 1.], (6, 1)))
            with self.assertRaisesRegex(ValueError, "timestamps"):
                load_episodes(tmp)

    def test_dagger_prefix_is_training_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i in range(3):
                episode(Path(tmp)/f"clean{i}.npz", seed=i)
            episode(Path(tmp)/"dagger.npz", seed=100, mode="dagger")
            data = load_episodes(tmp)
            splits = split_episodes(data, 0)
            self.assertTrue(any(e.seed == 100 for e in splits["train"]))
            self.assertFalse(any(e.training_only for key in ("validation", "test") for e in splits[key]))

    def test_dagger_expert_labels_are_not_behavior_actions(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"dagger.npz"
            episode(path, mode="dagger")
            with np.load(path) as data:
                values = {key: data[key].copy() for key in data.files}
            meta = json.loads(str(values["metadata"]))
            meta["final"]["success"] = False
            meta["blocked"] = True
            values["metadata"] = np.asarray(json.dumps(meta))
            values["behavior_actions"][:] = 0.
            values["requested"][:] = 0.
            values["executed"][:] = 0.
            values["observations"][:, 149:151] = 0.
            values["next_observations"][:, 149:151] = 0.
            values["safety_blocked"][-1] = True
            np.savez_compressed(path, **values)
            data = load_episodes(tmp)
            self.assertTrue(data[0].training_only)
            np.testing.assert_array_equal(data[0].actions[:, 1], np.full(6, .5))

    def test_reference_heldout_seeds_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i in range(3):
                episode(Path(tmp)/f"{i}.npz", seed=i)
            reference = Path(tmp)/"split.json"
            reference.write_text(json.dumps({"splits": {
                "train": [{"task": "main_to_left", "seed": 2}],
                "validation": [{"task": "main_to_left", "seed": 0}],
                "test": [{"task": "main_to_left", "seed": 1}]}}))
            splits = split_episodes(load_episodes(tmp), 100, reference)
            self.assertEqual([e.seed for e in splits["test"]], [1])
            self.assertEqual([e.seed for e in splits["train"]], [2])

    def test_explicit_recovery_augmentation_is_training_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i in range(3):
                episode(Path(tmp)/f"clean{i}.npz", seed=i)
            episode(Path(tmp)/"recovery.npz", seed=100, mode="perturbed")
            data = load_episodes(tmp)
            with self.assertRaisesRegex(ValueError, "successful perturbed"):
                split_episodes(data, 0)
            next(e for e in data if e.source == "perturbed").training_only = True
            splits = split_episodes(data, 0)
            self.assertTrue(any(e.seed == 100 for e in splits["train"]))
            self.assertTrue(all(e.seed != 100 for name in ("validation", "test") for e in splits[name]))

    def test_phase_and_correction_sampling_mass(self):
        with tempfile.TemporaryDirectory() as tmp:
            episode(Path(tmp)/"clean.npz")
            episode(Path(tmp)/"correction.npz", seed=10, mode="perturbed")
            data = load_episodes(tmp)
            _, phases, kinds = label_metadata(data)
            weights = sampling_weights(data, .3)
            for phase in np.unique(phases):
                take = phases == phase
                self.assertAlmostEqual(weights[take].sum(), .25)
                self.assertAlmostEqual(weights[take & (kinds == 1)].sum()/weights[take].sum(), .3)

    def test_geometry_scaling_is_causal_and_preserves_other_inputs(self):
        space = gym.spaces.Box(-1., 1., (BASE_OBS_SIZE*4,), dtype=np.float32)
        extractor = LocalGeometryFeatures(space, geometry_gain=10.)
        x = torch.ones((2, BASE_OBS_SIZE*4))
        expected = x.clone()
        for offset in range(0, BASE_OBS_SIZE*4, BASE_OBS_SIZE):
            expected[:, offset+155:offset+158] = 10.
        torch.testing.assert_close(extractor(x), expected)
        self.assertIn("scale", extractor.state_dict())
        with self.assertRaises(ValueError):
            LocalGeometryFeatures(space, geometry_gain=float("nan"))

    def test_action_balance_preserves_phase_source_mass(self):
        def item(task, kind, actions):
            return SimpleNamespace(task=task, actions=np.asarray(actions, dtype=np.float32),
                mask=np.ones(len(actions), dtype=bool), phases=np.ones(len(actions), dtype=int),
                kinds=np.full(len(actions), kind))
        data = [item("main_to_left", 0, [[.2, 0]]*8+[[0, .5], [0, 0]]),
                item("main_to_left", 2, [[-.2, 0]]*3+[[.2, 0]]),
                item("left_to_right_reverse", 0, [[-1, 0]]*4+[[0, 0]])]
        w = sampling_weights(data, .3, action_balance=True)
        self.assertAlmostEqual(w[:10].sum(), .7)
        self.assertAlmostEqual(w[10:14].sum(), .3)
        self.assertAlmostEqual(w[:8].sum(), .7/3)
        self.assertAlmostEqual(w[8], .7/3)
        self.assertAlmostEqual(w[9], .7/3)
        self.assertAlmostEqual(w[10:13].sum(), .15)
        # Reverse-task normalized -1 means STOP, not physical reverse.
        self.assertAlmostEqual(w[14:18].sum(), .5)
        self.assertAlmostEqual(w[18], .5)
        self.assertTrue(np.isfinite(w).all())

    def test_current_local_context_and_stale_geometry_flags(self):
        cfg = SimpleNamespace(junction_min_confidence=.65)
        tracker = SimpleNamespace(stamp=10., incoming_axis=0.,
            geometry=SimpleNamespace(centre=np.array([.1, -.04]), confidence=.9))
        c = SimpleNamespace(state="TURN_LEFT", cfg=cfg, tracker=tracker, centre_retreat=False,
            failure=None, relocalization=None, obstacle=None, external_paused=False)
        context = decision_context(c, None, 10.)
        np.testing.assert_allclose(context[:4], [0, 0, 1, 0])
        np.testing.assert_allclose(context[4:7], [.05, -.02, .5])
        np.testing.assert_array_equal(context[7:9], [1, 1])
        with patch("policy_context.incoming_corridor", return_value=None):
            stale = decision_context(c, None, 11.)
        np.testing.assert_array_equal(stale[4:9], np.zeros(5))

    def test_overshoot_offset_uses_lidar_and_stays_bounded(self):
        c = SimpleNamespace(state="CENTER", obstacle=None, relocalization=None,
            tracker=SimpleNamespace(incoming_axis=0., geometry=SimpleNamespace(centre=np.array([.04, 0.]))))
        kind, offset = recovery_offset(c, np.random.default_rng(0), "overshoot", "center")
        self.assertEqual(kind, "overshoot")
        self.assertGreater(offset[0], .10)
        self.assertLess(offset[0], .13)
        self.assertEqual(offset[1], 0.)

    def test_invalid_teacher_labels_do_not_remove_history_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            mask = np.array([True, False, True, True, True, True])
            episode(Path(tmp)/"clean.npz", label_valid=mask)
            data = load_episodes(tmp)
            x, _ = arrays(data, 4)
            self.assertEqual(len(x), 5)
            expected = history_windows(data[0].observations, 4)[mask]
            np.testing.assert_array_equal(x, expected)

    def test_overshoot_data_without_reverse_correction_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"fake_recovery.npz"
            episode(path, mode="perturbed")
            with np.load(path) as data:
                values = {key: data[key].copy() for key in data.files}
            meta = json.loads(str(values["metadata"]))
            meta["perturbations"] = [{"case": "overshoot"}]
            values["metadata"] = np.asarray(json.dumps(meta))
            np.savez_compressed(path, **values)
            with self.assertRaisesRegex(ValueError, "actual reverse"):
                load_episodes(tmp)


if __name__ == "__main__":
    unittest.main()
