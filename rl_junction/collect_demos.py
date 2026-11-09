#!/usr/bin/env python3
"""Collect real LiDAR FSM demonstrations in the isolated original-map simulator."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import numpy as np
from junction_env import HybridJunctionEnv as JunctionEnv, HANDOFF_STATES
from learning_common import contract, encode_command
from tasks import TASKS
from fsm_teacher import TEACHER_ID
from policy_context import phase_id


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--episodes-per-task", type=int, default=30, help="Attempts, not guaranteed successes")
    p.add_argument("--target-successes", type=int, help="Stop a task once this many accepted episodes are collected")
    p.add_argument("--task", choices=["all"]+list(HANDOFF_STATES), default="main_to_left")
    p.add_argument("--mode", choices=["normal", "perturbed", "dagger"], default="normal")
    p.add_argument("--behavior-model", type=Path)
    p.add_argument("--teacher-probability", type=float, default=.5)
    p.add_argument("--perturb-case", choices=["mixed", "lateral", "overshoot", "yaw"], default="mixed")
    p.add_argument("--perturb-phase", choices=["center", "turn", "entry"], default="center")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-steps", type=int, default=1200, help="Junction skill steps; long-distance FSM cruise is separate")
    p.add_argument("--output", type=Path, default=Path("logs/rl_junction_fsm_demos"))
    p.add_argument("--run-dir", type=Path, help="New exact run directory, used by the automatic pipeline")
    args = p.parse_args()
    if min(args.episodes_per_task, args.max_steps) < 1 or args.seed < 0:
        p.error("episode/step counts must be positive and seed nonnegative")
    if args.target_successes is not None and not 1 <= args.target_successes <= args.episodes_per_task:
        p.error("target-successes must be between 1 and episodes-per-task")
    if not 0 <= args.teacher_probability <= 1:
        p.error("teacher-probability must be in [0,1]")
    if (args.mode == "dagger") != bool(args.behavior_model):
        p.error("dagger mode requires --behavior-model; other modes do not use it")
    if args.mode == "dagger" and args.target_successes:
        p.error("DAgger prefixes are not successful episodes; use a fixed attempt count")
    if args.mode == "perturbed" and args.perturb_case == "overshoot" and (
            args.task in ("all", "left_to_right_reverse") or args.perturb_phase != "center"):
        p.error("overshoot perturbation requires a turning task and --perturb-phase center")
    if args.mode == "perturbed" and args.perturb_phase == "turn" and args.task in ("all", "left_to_right_reverse"):
        p.error("Reverse crossing has no TURN phase; choose a turning task or center/entry")
    behavior = None
    if args.behavior_model:
        from behavior_policy import BehaviorPolicy
        behavior = BehaviorPolicy(args.behavior_model)
    run = args.run_dir or args.output / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True)
    version = contract()
    (run / "contract.json").write_text(json.dumps(version, indent=2))
    env = JunctionEnv(max_steps=args.max_steps,
        recovery_case=args.perturb_case if args.mode == "perturbed" else None,
        recovery_phase=args.perturb_phase)
    def stop(*_):
        raise KeyboardInterrupt
    old = signal.signal(signal.SIGTERM, stop)
    accepted, results = 0, []
    counts = {}
    task = None
    try:
        tasks = [t for t in TASKS if args.task == "all" or t.name == args.task]
        if behavior and any(t.name not in behavior.spec["tasks"] for t in tasks):
            p.error("Behavior checkpoint was not trained for every selected task")
        counts = {t.name: 0 for t in tasks}
        for attempt in range(args.episodes_per_task):
            for task in tasks:
                if args.target_successes and counts[task.name] >= args.target_successes:
                    continue
                env.fixed_task = task.name
                seed = args.seed + attempt*3 + [t.name for t in TASKS].index(task.name)
                obs, _ = env.reset(seed=seed)
                teacher = env.teacher
                mix_rng = np.random.default_rng(seed+10000000)
                if behavior:
                    behavior.reset(obs)
                print(f"START {task.name}: seed={seed}", flush=True)
                records = {key: [] for key in (
                    "observations", "next_observations", "actions", "requested", "executed",
                    "scan_stamps", "sensor_stamps", "ranges", "velocities", "poses", "next_poses",
                    "fsm_states", "fsm_next_states", "entry_phases", "safety_blocked", "rewards",
                    "expert_commands", "behavior_actions", "label_valid", "phase_ids", "sample_kinds")}
                intervened = False
                episode_truncated = False
                while True:
                    before = teacher.status()
                    sensor_stamps = [env.scan_stamp, env.odom_stamp, env.pose_stamp, env.sim_time]
                    raw = np.asarray(env.scan.ranges, dtype=np.float32).copy()
                    pose = env.pose.copy()
                    velocity = env.odom.twist.twist
                    measured = [velocity.linear.x, velocity.angular.z]
                    command = env.expert_command()
                    status = teacher.status()
                    # Recovery is excluded from clean expert demonstrations.
                    teacher_blocked = bool(status["obstacle"] or status["relocalizing"] or status["failure"])
                    if status["failure"]:
                        info = {"task": task.name, "success": False, "teacher_failure": status["failure"]}
                        break
                    action = encode_command(*command, task.name)
                    label_valid = env.label_valid
                    kind = 2 if behavior else int(env.perturbation_applied)
                    actual_action = (behavior.action() if behavior and mix_rng.random() >= args.teacher_probability else action)
                    next_obs, reward, done, truncated, info = env.step(actual_action)
                    episode_truncated = bool(truncated)
                    if behavior:
                        behavior.observe(next_obs)
                    after = teacher.status()
                    blocked = bool(info["safety_blocked"] or teacher_blocked or after["obstacle"] or
                                   after["relocalizing"] or after["failure"])
                    intervened |= blocked
                    row = {
                        "observations": obs.copy(), "next_observations": next_obs.copy(),
                        "actions": action, "requested": info["requested_action"], "executed": info["executed_action"],
                        "expert_commands": command, "behavior_actions": actual_action,
                        "label_valid": label_valid, "phase_ids": phase_id(obs), "sample_kinds": kind,
                        "scan_stamps": sensor_stamps[0], "sensor_stamps": sensor_stamps,
                        "ranges": raw, "velocities": measured, "poses": pose, "next_poses": env.pose.copy(),
                        "fsm_states": before["state"], "fsm_next_states": teacher.controller.state,
                        "entry_phases": status["entry_phase"], "safety_blocked": blocked, "rewards": reward,
                    }
                    for key in records:
                        records[key].append(row[key])
                    obs = next_obs
                    if len(records["actions"]) % 50 == 0:
                        print(f"{task.name}: step={len(records['actions'])}, state={status['state']}, "
                              f"distance={info['distance_to_goal']:.3f}, intervened={intervened}", flush=True)
                    if done or truncated:
                        break
                # A final expected road state proves that FSM completed its junction handoff.
                expected = HANDOFF_STATES[task.name]
                recovered = bool(info.get("success") and not intervened and teacher.controller.state == expected)
                if args.mode == "perturbed":
                    recovered = recovered and env.perturbation_applied
                valid_count = int(np.sum(records["label_valid"]))
                repair_mask = ((np.asarray(records["sample_kinds"]) == 1) & np.asarray(records["label_valid"], dtype=bool))
                expert_v = np.asarray(records["expert_commands"])[:, 0] if records["expert_commands"] else np.array([])
                reverse_repairs = int(np.sum(repair_mask & (expert_v < -.005)))
                if (args.mode == "perturbed" and env.perturbations and
                        env.perturbations[0]["case"] == "overshoot" and reverse_repairs == 0):
                    recovered = False
                good = (valid_count >= 5) if behavior else (recovered and valid_count >= 5)
                meta = {"contract": version, "teacher": TEACHER_ID, "task": task.name, "seed": seed,
                        "policy_scope": "JUNCTION_APPROACH_TO_HANDOFF", "start_route_state": env.start_route_state,
                        "accepted": good, "blocked": intervened, "steps": len(records["actions"]),
                        "data_mode": args.mode, "training_only": bool(behavior), "valid_labels": valid_count,
                        "truncated": episode_truncated, "label_prefix": bool(behavior),
                        "recovered": recovered, "perturbations": env.perturbations,
                        "recovery_statistics": {"valid_post_disturbance_labels": int(repair_mask.sum()),
                                                "reverse_correction_labels": reverse_repairs},
                        "behavior_policy": None if behavior is None else behavior.provenance,
                        "teacher_probability": args.teacher_probability if behavior else 1.,
                        "max_steps": args.max_steps, "final": info, "teacher_final": teacher.status(),
                        "teacher_speed_limits": {"linear": teacher.cfg.cruise_speed, "angular": teacher.cfg.turn_speed},
                        "teacher_centre_tolerance": teacher.cfg.junction_centre_tolerance,
                        "scan_geometry": {"angle_min": env.scan.angle_min,
                            "angle_increment": env.scan.angle_increment,
                            "range_min": env.scan.range_min, "range_max": env.scan.range_max}}
                stem = f"{task.name}_{attempt:04d}"
                (run / (stem+".json")).write_text(json.dumps(meta, indent=2, default=json_default))
                results.append({"file": stem, "accepted": good, "task": task.name, "seed": seed,
                                "steps": len(records["actions"]), "final": info})
                if good:
                    arrays = {key: np.asarray(value, dtype=(str if key in
                        ("fsm_states", "fsm_next_states", "entry_phases") else
                        bool if key in ("safety_blocked", "label_valid") else
                        np.int64 if key in ("phase_ids", "sample_kinds") else np.float64 if key in
                        ("sensor_stamps", "scan_stamps") else np.float32))
                        for key, value in records.items()}
                    np.savez_compressed(run / (stem+".npz"), **arrays,
                        metadata=np.asarray(json.dumps(meta, default=json_default)))
                    accepted += 1
                    counts[task.name] += 1
                progress = {"accepted": accepted, "accepted_per_task": counts,
                            "target_per_task": args.target_successes,
                            "completed_attempts": len(results), "last_episode": results[-1]}
                (run / "progress.json").write_text(json.dumps(progress, indent=2))
                verdict = ("LABEL_PREFIX_ACCEPTED" if behavior else "ACCEPTED") if good else "REJECTED"
                print(f"{stem}: {verdict} {info}", flush=True)
            if args.target_successes and all(n >= args.target_successes for n in counts.values()):
                break
    except KeyboardInterrupt:
        print("Interrupted: incomplete episode discarded")
    except Exception as exc:
        (run / "infrastructure_error.json").write_text(json.dumps({
            "task": task.name if task else None, "type": type(exc).__name__, "error": str(exc)}, indent=2))
        raise
    finally:
        env.close()
        signal.signal(signal.SIGTERM, old)
        (run / "summary.json").write_text(json.dumps({
            "accepted": accepted, "accepted_per_task": counts,
            "completed_attempts": len(results), "episodes": results}, indent=2))
    print(f"Accepted {accepted} episodes; directory: {run}")
    if accepted == 0:
        raise SystemExit("No clean successful FSM demonstrations. Inspect episode JSON before training.")
    if args.target_successes and any(n < args.target_successes for n in counts.values()):
        raise SystemExit(f"Collection target not met: {counts}; inspect rejected episodes before training")


if __name__ == "__main__":
    main()
