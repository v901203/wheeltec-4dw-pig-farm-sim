#!/usr/bin/env python3
"""Replay v5 raw LiDAR to add causal v6 context, requiring identical actions/states.

Changed teacher code requires explicit opt-in; every command/state must reproduce.
Never reconstruct inputs from poses.
Original files are left untouched; incompatible episodes stop the conversion.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rl_pig_pen"))
from navigation import load_config, scan_features
from fsm_teacher import FSMTeacher, HANDOFF_STATES
from tasks import TASKS
from policy_context import decision_context, phase_id
from learning_common import contract


def convert(source, destination, allow_teacher_replay=False):
    version = contract()
    with np.load(source, allow_pickle=False) as data:
        meta = json.loads(str(data["metadata"]))
        old = meta["contract"]
        if old.get("schema") != "pig_pen_fsm_junction_bc_v5" or not meta["accepted"] or meta["blocked"]:
            raise ValueError("Expected clean accepted v5 FSM data")
        for key in ("world_sha256", "config_sha256", "fsm_sha256", "localization_sha256",
                    "fsm_teacher_sha256", "navigation_sha256", "geometry_sha256", "robot_sha256", "dt", "action_mapping"):
            if old.get(key) != version[key] and not (
                    allow_teacher_replay and key in ("fsm_sha256", "localization_sha256")):
                raise ValueError(f"Replay configuration changed: {key}")
        cfg = replace(load_config(), control_dt=.2, vision_required=False)
        task = next(t for t in TASKS if t.name == meta["task"])
        teacher = FSMTeacher(task, cfg)
        observations, states = [], []
        for i, ranges in enumerate(data["ranges"]):
            scan = SimpleNamespace(ranges=ranges, **meta["scan_geometry"])
            features = scan_features(scan, cfg)
            state_before = teacher.controller.state
            command = teacher.command(features, float(data["scan_stamps"][i]))
            if state_before != data["fsm_states"][i] or teacher.controller.state != data["fsm_next_states"][i]:
                raise ValueError(f"FSM replay state mismatch at frame {i}")
            np.testing.assert_allclose(command, data["requested"][i], atol=2e-6)
            context = decision_context(teacher.controller, features, float(data["scan_stamps"][i]))
            observations.append(np.concatenate((data["observations"][i], context)))
            states.append(teacher.controller.state)
        if states[-1] != HANDOFF_STATES[task.name] or not np.allclose(data["actions"][-1], 0., atol=1e-6):
            # Reverse-only stop has policy a0=-1, while physical stop remains zero.
            if states[-1] != HANDOFF_STATES[task.name] or not np.allclose(data["requested"][-1], 0., atol=1e-6):
                raise ValueError("Missing final zero-command handoff frame")
        # v6 commits next-scan FSM events before returning: remove the old extra
        # stationary handoff step, whose scan now supplies the final next context.
        n = len(observations)-1
        augmented = np.asarray(observations, dtype=np.float32)
        arrays = {key: data[key][:-1].copy() for key in data.files if key != "metadata"}
        arrays.update(observations=augmented[:-1], next_observations=augmented[1:],
                      fsm_states=np.asarray(states[:-1]), fsm_next_states=np.asarray(states[1:]),
                      expert_commands=data["requested"][:-1], behavior_actions=data["actions"][:-1],
                      phase_ids=np.asarray([phase_id(x) for x in augmented[:-1]], dtype=np.int64),
                      sample_kinds=np.zeros(n, dtype=np.int64),
                      label_valid=np.asarray([bool(x[159] and (x[158] or states[i] in
                                              (*teacher.controller.ENTRY_STATES, *teacher.controller.JUNCTION_APPROACH_STATES)))
                                              for i, x in enumerate(augmented[:-1])]))
        if arrays["label_valid"].sum() < 5:
            raise ValueError("Too few valid replayed expert labels")
        meta.update(contract=version, steps=n, data_mode="normal", training_only=False,
                    valid_labels=int(arrays["label_valid"].sum()), recovered=True, perturbations=[], behavior_policy=None,
                    provenance={"source": str(source.resolve()), "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                                "original_contract": old, "method": "raw_lidar_exact_fsm_replay_v1"})
        np.savez_compressed(destination, **arrays, metadata=np.asarray(json.dumps(meta)))
        return {"source": str(source), "output": str(destination), "task": task.name, "seed": meta["seed"], "steps": n}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True, help="New directory; old data is never overwritten")
    p.add_argument("--limit", type=int)
    p.add_argument("--task", choices=[t.name for t in TASKS])
    p.add_argument("--allow-teacher-replay", action="store_true",
                   help="Permit changed FSM/localizer code ONLY if every recorded state/action reproduces exactly")
    p.add_argument("--skip-incompatible", action="store_true",
                   help="Report and exclude entire incompatible episodes; never relabel mismatched actions")
    args = p.parse_args()
    files = sorted(args.source.rglob("*.npz"))
    if args.task:
        files = [f for f in files if f.name.startswith(args.task+"_")]
    if args.limit is not None:
        if args.limit < 1:
            p.error("limit must be positive")
        files = files[:args.limit]
    if not files:
        p.error("No source episodes")
    args.output.mkdir(parents=True, exist_ok=False)
    rows, rejected = [], []
    try:
        for i, source in enumerate(files):
            try:
                row = convert(source, args.output / f"{i:04d}_{source.stem}.npz", args.allow_teacher_replay)
            except (ValueError, AssertionError) as exc:
                if not args.skip_incompatible:
                    raise
                rejected.append({"source": str(source), "reason": str(exc)})
                print(f"EXCLUDED {source.name}: {exc}", flush=True)
                continue
            rows.append(row)
            print(f"REPLAY {i+1}/{len(files)}: {row}", flush=True)
    finally:
        (args.output / "conversion.json").write_text(json.dumps({"completed": len(rows), "expected": len(files),
                                                                "episodes": rows, "rejected": rejected}, indent=2))
    if not rows:
        raise SystemExit("No episodes reproduced under the current teacher")


if __name__ == "__main__":
    main()
