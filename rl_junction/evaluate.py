#!/usr/bin/env python3
"""Deterministic per-task Gazebo evaluation; no policy updates."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import numpy as np
from stable_baselines3 import PPO
from junction_env import HybridJunctionEnv as JunctionEnv, HANDOFF_STATES
from learning_common import check_model, contract, encode_command, model_spec, HistoryObservation
from tasks import TASKS
from fsm_teacher import FSMTeacher, TEACHER_ID
from geometry_recording import GeometryRecorder


def main():
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--model", type=Path)
    source.add_argument("--teacher", action="store_true")
    p.add_argument("--task", choices=list(HANDOFF_STATES))
    p.add_argument("--episodes-per-task", type=int, default=10)
    p.add_argument("--seed", type=int, default=100000, help="Use held-out seeds, same for all comparisons")
    p.add_argument("--max-steps", type=int, default=1200)
    p.add_argument("--recovery-case", choices=["mixed", "lateral", "overshoot", "yaw"])
    p.add_argument("--recovery-phase", choices=["center", "turn", "entry"], default="center")
    p.add_argument("--output", type=Path, default=Path("logs/rl_junction_eval"))
    p.add_argument("--geometry-diagnostics", action="store_true",
                   help="Save compressed raw scans, wall identities/candidates and rejection reasons")
    args = p.parse_args()
    if min(args.episodes_per_task, args.max_steps) < 1:
        p.error("episodes-per-task and max-steps must be positive")
    if args.recovery_case == "overshoot" and (args.task in (None, "left_to_right_reverse") or args.recovery_phase != "center"):
        p.error("overshoot recovery test requires a turning task and center phase")
    model = PPO.load(check_model(args.model), device="cpu") if args.model else None
    spec = model_spec(args.model) if model else None
    allowed = spec["tasks"] if spec else list(HANDOFF_STATES)
    if args.task and args.task not in allowed:
        p.error("Task was not trained in this checkpoint")
    raw_env = JunctionEnv(available_tasks=allowed, max_steps=args.max_steps,
                         recovery_case=args.recovery_case, recovery_phase=args.recovery_phase)
    env = HistoryObservation(raw_env, spec["history"]) if model else raw_env
    run = args.output / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True)
    rows = []
    trace = (run / "steps.jsonl").open("w")
    geometry_log = GeometryRecorder(run/"geometry") if args.geometry_diagnostics else None
    print(f"Evaluation directory: {run}", flush=True)
    def encode(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(type(value).__name__)
    def stop(*_):
        raise KeyboardInterrupt
    old = signal.signal(signal.SIGTERM, stop)
    complete = False
    try:
        for i, task in enumerate(TASKS):
            if task.name not in allowed or (args.task and task.name != args.task):
                continue
            raw_env.fixed_task = task.name
            for episode in range(args.episodes_per_task):
                seed = args.seed + episode*3+i
                obs, _ = env.reset(seed=seed)
                print(f"START {task.name}: seed={seed}", flush=True)
                teacher = raw_env.teacher if not model else None
                blocks, clearance_blocks, fsm_blocks, reward_sum, stop_steps, step = 0, 0, 0, 0., 0, 0
                while True:
                    expert = raw_env.expert_command().copy()
                    decision_context = raw_env.cached_context.copy()
                    decision_state = raw_env.teacher.controller.state
                    decision_label_valid = raw_env.label_valid
                    if geometry_log:
                        geometry_log.record(raw_env, task.name, seed, step)
                    action = (model.predict(obs, deterministic=True)[0] if model else
                              encode_command(*expert, task.name))
                    obs, reward, done, truncated, info = env.step(action)
                    step += 1
                    trace.write(json.dumps({"task": task.name, "seed": seed, "step": step,
                        "pose": raw_env.pose, "policy_action": action,
                        "decision_state": decision_state, "decision_context": decision_context,
                        "expert_command": expert, "expert_label_valid": decision_label_valid,
                        "requested": info["requested_action"], "executed": info["executed_action"],
                        "route_state": info["route_state"], "reward": reward,
                        "safety_blocked": info["safety_blocked"],
                        "teacher_intervened": info["teacher_intervened"],
                        "terminated": done, "truncated": truncated}, default=encode)+"\n")
                    trace.flush()
                    if step % 50 == 0:
                        print(f"{task.name}: step={step}, state={info['route_state']}, "
                              f"heading_error={np.degrees(info['heading_error_rad']):.1f} deg, "
                              f"blocked={info['safety_blocked']}", flush=True)
                    clearance_blocks += int(info["safety_blocked"])
                    fsm_blocks += int(info["teacher_intervened"])
                    blocks += int(info["safety_blocked"] or info["teacher_intervened"])
                    stop_steps += int(abs(info["executed_action"][0]) < .01 and
                                      abs(info["executed_action"][1]) < .02)
                    reward_sum += reward
                    if done or truncated or (teacher is not None and teacher.controller.failure):
                        break
                rows.append(dict(info, seed=seed, safety_interventions=blocks,
                                 clearance_interventions=clearance_blocks, fsm_interventions=fsm_blocks,
                                 episode_return=reward_sum, truncated=truncated,
                                 stopped_seconds=stop_steps*raw_env.dt,
                                 teacher_failure=info.get("teacher_failure"),
                                 fsm_diagnostics=raw_env.teacher.status(),
                                 perturbations=raw_env.perturbations,
                                 recovery_test=bool(args.recovery_case),
                                 disturbance_applied=raw_env.perturbation_applied,
                                 recovery_reason=(raw_env.teacher.controller.relocalization or {}).get("reason")))
                print(rows[-1], flush=True)
        complete = True
    finally:
        try:
            if geometry_log:
                geometry_log.close()
        finally:
            try:
                trace.close()
            finally:
                try:
                    env.close()
                finally:
                    signal.signal(signal.SIGTERM, old)
        summary = {}
        for task in TASKS:
            data = [r for r in rows if r["task"] == task.name]
            successes = [r for r in data if r["success"]]
            if data:
                disturbed = [r for r in data if r.get("disturbance_applied")]
                summary[task.name] = {
                    "episodes": len(data), "success_rate": len(successes)/len(data),
                    "collision_rate": sum(r["collision"] for r in data)/len(data),
                    "mean_success_seconds": float(np.mean([r["sim_seconds"] for r in successes])) if successes else None,
                    "mean_safety_interventions": float(np.mean([r["safety_interventions"] for r in data])),
                    "disturbed_episodes": len(disturbed),
                    "recovery_success_rate": sum(r["success"] for r in disturbed)/len(disturbed) if disturbed else None,
                }
        report = {"complete": complete, "model": str(args.model) if model else TEACHER_ID,
                  "metrics_definition": "safety_interventions counts steps with clearance OR FSM protection; components recorded separately",
                  "contract": contract(), "summary": summary, "episodes": rows}
        (run / "evaluation.json").write_text(json.dumps(report, indent=2, default=encode))
        print(json.dumps(summary, indent=2))
        print(f"Report: {run / 'evaluation.json'}")


if __name__ == "__main__":
    main()
