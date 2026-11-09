"""Validated complete FSM episodes and causal history windows, with no ROS dependency."""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import numpy as np
from learning_common import BASE_OBS_SIZE, contract, history_windows
from policy_context import PHASE_NAMES, RAW_OBS_SIZE
from fsm_teacher import TEACHER_ID, HANDOFF_STATES, SKILL_STATES, START_STATES


@dataclass
class Episode:
    path: Path
    task: str
    seed: int
    observations: np.ndarray
    actions: np.ndarray
    sha256: str
    phases: np.ndarray
    kinds: np.ndarray
    mask: np.ndarray
    training_only: bool
    source: str


def load_episodes(directory, task=None):
    version = contract()
    episodes, seen = [], set()
    roots = directory if isinstance(directory, (list, tuple)) else [directory]
    files = sorted({path.resolve() for root in roots for path in Path(root).rglob("*.npz")})
    if not files:
        raise ValueError(f"No accepted .npz episodes under {directory}; collect FSM demonstrations first")
    for path in files:
        try:
            with np.load(path, allow_pickle=False) as data:
                meta = json.loads(str(data["metadata"]))
                if task and meta["task"] != task:
                    continue
                mode = meta.get("data_mode")
                if meta["contract"] != version or meta["teacher"] != TEACHER_ID or not meta["accepted"]:
                    raise ValueError("wrong environment/teacher or unaccepted episode")
                if mode not in ("normal", "perturbed", "dagger"):
                    raise ValueError("missing data mode")
                training_only = meta.get("training_only", False)
                if mode == "dagger":
                    if not training_only or not meta.get("behavior_policy"):
                        raise ValueError("DAgger labels must be marked training-only with behavior provenance")
                elif meta["blocked"] or not meta["final"]["success"]:
                    raise ValueError("expert episode was unsuccessful/intervened")
                if mode == "perturbed" and (not meta.get("recovered") or not meta.get("perturbations")):
                    raise ValueError("recovery episode has no successful measured disturbance")
                if (meta["task"] not in HANDOFF_STATES or
                        meta.get("policy_scope") != "JUNCTION_APPROACH_TO_HANDOFF"):
                    raise ValueError("expected junction approach-to-handoff demonstrations")
                x, y, executed = [data[k].copy() for k in ("observations", "actions", "executed")]
                n = len(y)
                if (n == 0 or x.shape != (n, BASE_OBS_SIZE) or y.shape != (n, 2) or
                        executed.shape != (n, 2) or meta["steps"] != n):
                    raise ValueError("invalid observation/action shape or step count")
                for name, value in (("observations", x), ("actions", y), ("executed", executed)):
                    if not np.isfinite(value).all():
                        raise ValueError(f"nonfinite {name}")
                if np.max(np.abs(x)) > 1.00001 or np.max(np.abs(y)) > 1.00001:
                    raise ValueError("normalized values outside [-1,1]")
                if mode != "dagger" and np.any(data["safety_blocked"]):
                    raise ValueError("intervened expert steps")
                mask = data["label_valid"].astype(bool)
                phases = data["phase_ids"].astype(int)
                kinds = data["sample_kinds"].astype(int)
                if (mask.shape != (n,) or mask.sum() < 5 or phases.shape != (n,) or kinds.shape != (n,) or
                        np.any((phases < 0) | (phases >= len(PHASE_NAMES)))):
                    raise ValueError("invalid label mask, phase or too few valid expert labels")
                if (mode == "normal" and np.any(kinds != 0) or
                        mode == "perturbed" and (not np.any(kinds == 1) or np.any(~np.isin(kinds, [0, 1]))) or
                        mode == "dagger" and np.any(kinds != 2)):
                    raise ValueError("sample kind differs from collection mode")
                np.testing.assert_allclose(x[:, RAW_OBS_SIZE:RAW_OBS_SIZE+4], np.eye(4)[phases])
                validity = x[:, [158, 159, 160]]
                if not np.all(np.isin(validity, [0., 1.])):
                    raise ValueError("context validity/reposition flags are not binary")
                if np.any(mask & ((x[:, 159] != 1) | (np.isin(phases, [1, 2]) & (x[:, 158] != 1)))):
                    raise ValueError("valid teacher label has invalid local geometry")
                np.testing.assert_allclose(x[x[:, 158] == 0, 155:157], 0., atol=1e-6)
                np.testing.assert_allclose(x[x[:, 159] == 0, 157], 0., atol=1e-6)
                stamps = data["sensor_stamps"]
                if (stamps.shape != (n, 4) or not np.isfinite(stamps).all() or
                        np.any(np.diff(stamps[:, 0]) <= 0) or np.any(np.ptp(stamps, axis=1) > .121)):
                    raise ValueError("invalid/nonmonotonic/unsynchronized sensor timestamps")
                for key in ("fsm_states", "fsm_next_states", "entry_phases", "ranges", "velocities", "poses", "next_poses"):
                    if len(data[key]) != n:
                        raise ValueError(f"unaligned {key}")
                allowed_phases = SKILL_STATES[meta["task"]]
                handoff = HANDOFF_STATES[meta["task"]]
                if (not np.all(np.isin(data["fsm_states"], allowed_phases)) or
                        not np.all(np.isin(data["fsm_next_states"][mask], [*allowed_phases, handoff, "FAILED"])) or
                        data["fsm_states"][0] != START_STATES[meta["task"]] or
                        (mode != "dagger" and meta["teacher_final"]["state"] != handoff)):
                    raise ValueError("demonstration outside junction skill FSM phases")
                requested = y.astype(np.float64)*[.2, .6]
                if meta["task"] == "left_to_right_reverse":
                    requested[:, 0] = -.1*(y[:, 0]+1.)
                np.testing.assert_allclose(requested, data["expert_commands"], atol=1e-6)
                if mode == "perturbed" and meta["perturbations"][0]["case"] == "overshoot":
                    if not np.any(mask & (kinds == 1) & (phases == 1) & (requested[:, 0] < -.005)):
                        raise ValueError("overshoot episode did not demonstrate actual reverse correction")
                behavior = data["behavior_actions"].astype(np.float64)
                if behavior.shape != (n, 2) or not np.isfinite(behavior).all() or np.max(np.abs(behavior)) > 1.00001:
                    raise ValueError("invalid behavior action")
                actual = behavior*[.2, .6]
                if meta["task"] == "left_to_right_reverse":
                    actual[:, 0] = -.1*(behavior[:, 0]+1)
                np.testing.assert_allclose(actual[mask], data["requested"][mask], atol=1e-6)
                if mode != "dagger":
                    np.testing.assert_allclose(y, behavior, atol=1e-6)
                    np.testing.assert_allclose(requested[mask], executed[mask], atol=1e-6)
                np.testing.assert_allclose(x[1:], data["next_observations"][:-1], atol=1e-6)
                np.testing.assert_allclose(x[1:, 149:151], executed[:-1]/[.2, .6], atol=1e-6)
                task_index = version["tasks"].index(meta["task"])
                np.testing.assert_allclose(x[:, 144:147], np.tile(np.eye(3)[task_index], (n, 1)))
                np.testing.assert_allclose(x[:, 147:149], np.clip(data["velocities"]/[.2, .6], -1, 1), atol=1e-6)
                identity = (meta["task"], int(meta["seed"]))
                if identity in seen:
                    raise ValueError("duplicate task/seed across episodes")
                seen.add(identity)
        except (KeyError, ValueError, AssertionError, OSError) as exc:
            raise ValueError(f"Invalid demonstration {path}: {exc}") from exc
        episodes.append(Episode(path.resolve(), meta["task"], int(meta["seed"]),
                                x.astype(np.float32), y.astype(np.float32),
                                hashlib.sha256(path.read_bytes()).hexdigest(), phases, kinds, mask,
                                bool(training_only), mode))
    if not episodes:
        raise ValueError(f"No accepted episodes for task {task}")
    return episodes


def summarize(episodes):
    return {task: {"episodes": sum(e.task == task for e in episodes),
                   "steps": sum(len(e.actions) for e in episodes if e.task == task),
                   "valid_labels": sum(int(e.mask.sum()) for e in episodes if e.task == task),
                   "training_only_episodes": sum(e.training_only for e in episodes if e.task == task)}
            for task in sorted({e.task for e in episodes})}


def split_episodes(episodes, seed, reference=None):
    assignments = {}
    if reference:
        saved = json.loads(Path(reference).read_text())
        for name, rows in saved["splits"].items():
            for row in rows:
                key = (row["task"], int(row["seed"]))
                if key in assignments and assignments[key] != name:
                    raise ValueError("Reference split leaks an episode between partitions")
                assignments[key] = name
    splits = {"train": [], "validation": [], "test": []}
    for task in sorted({e.task for e in episodes}):
        # DAgger prefixes NEVER enter validation or test. Normal/recovery get
        # independent episode-level holdouts, preserving recovery evaluation.
        for source in ("normal", "perturbed"):
            items = sorted([e for e in episodes if e.task == task and e.source == source and not e.training_only],
                           key=lambda e: e.seed)
            if not items:
                continue
            if len(items) < 3:
                raise ValueError(f"Need >=3 successful {source} episodes for {task} (train/validation/test); got {len(items)}")
            if source == "normal" and assignments:
                for item in items:
                    key = (item.task, item.seed)
                    if key not in assignments:
                        raise ValueError(f"Normal episode missing from reference split: {key}")
                    splits[assignments[key]].append(item)
                continue
            group_seed = int.from_bytes(hashlib.sha256(f"{seed}/{task}/{source}".encode()).digest()[:8], "little")
            rng = np.random.default_rng(group_seed)
            rng.shuffle(items)
            holdout = max(1, int(len(items)*.2))
            splits["validation"].extend(items[:holdout])
            splits["test"].extend(items[holdout:2*holdout])
            splits["train"].extend(items[2*holdout:])
        for item in (e for e in episodes if e.task == task and e.training_only):
            if assignments.get((item.task, item.seed)) in ("validation", "test"):
                raise ValueError("DAgger attempted to use a held-out evaluation seed")
            splits["train"].append(item)
        if any(not any(e.task == task for e in splits[name]) for name in ("validation", "test")):
            raise ValueError(f"Task {task} has no independent held-out episodes")
    if any(not items for items in splits.values()):
        raise ValueError("Need clean or perturbed held-out episodes, not DAgger data alone")
    return splits


def arrays(episodes, history):
    # Form windows on the ENTIRE sequence before applying the label mask.
    # Invalid teacher labels are excluded without deleting temporal context.
    return (np.concatenate([history_windows(e.observations, history)[e.mask] for e in episodes]),
            np.concatenate([e.actions[e.mask] for e in episodes]))


def label_metadata(episodes):
    return (np.concatenate([np.full(e.mask.sum(), e.task) for e in episodes]),
            np.concatenate([e.phases[e.mask] for e in episodes]),
            np.concatenate([e.kinds[e.mask] for e in episodes]))


def sampling_weights(episodes, correction_fraction=.3, action_balance=False):
    if not 0 < correction_fraction < 1:
        raise ValueError("correction_fraction must be in (0,1)")
    tasks, phases, kinds = label_metadata(episodes)
    weights = np.zeros(len(tasks), dtype=np.float64)
    for task in np.unique(tasks):
        task_mask = tasks == task
        available_phases = np.unique(phases[task_mask])
        for phase in available_phases:
            group = task_mask & (phases == phase)
            clean = group & (kinds == 0)
            repair = group & (kinds != 0)
            if clean.any() and repair.any():
                weights[clean] = (1-correction_fraction)/clean.sum()
                weights[repair] = correction_fraction/repair.sum()
            else:
                weights[group] = 1./group.sum()
            weights[group] /= len(available_phases)
    if action_balance:
        # Balance stop/forward/reverse/turn labels WITHIN each existing
        # task/phase/source group, preserving all outer sampling masses.
        # Decode reverse-task actions before classifying physical motion.
        normalized = np.concatenate([e.actions[e.mask] for e in episodes])
        commands = normalized.copy()
        commands[:, 0] *= .2
        reverse = tasks == "left_to_right_reverse"
        commands[reverse, 0] = -.1*(normalized[reverse, 0]+1.)
        commands[:, 1] *= .6
        regimes = np.where(commands[:, 0] > .005, 1,
                    np.where(commands[:, 0] < -.005, 2,
                    np.where(np.abs(commands[:, 1]) > .02, 3, 0)))
        for task in np.unique(tasks):
            for phase in np.unique(phases[tasks == task]):
                for correction in (False, True):
                    group = (tasks == task) & (phases == phase) & ((kinds != 0) == correction)
                    if not group.any():
                        continue
                    mass = weights[group].sum()
                    present = np.unique(regimes[group])
                    for regime in present:
                        take = group & (regimes == regime)
                        weights[take] = mass/(len(present)*take.sum())
    return weights
