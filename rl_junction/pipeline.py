#!/usr/bin/env python3
"""Collect balanced FSM skills, validate, then start offline MLP+BC automatically."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--successes-per-task", type=int, default=30)
    p.add_argument("--max-attempts-per-task", type=int, default=90)
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--recovery-successes-per-task", type=int, default=12,
                   help="Successful perturbed episodes per task; 0 disables")
    p.add_argument("--dagger-model", type=Path)
    p.add_argument("--dagger-episodes-per-task", type=int, default=3)
    p.add_argument("--normal-demos", type=Path, help="Reuse validated v6 normal episodes instead of recollecting")
    p.add_argument("--split-from", type=Path, help="Preserve the original normal-data split")
    p.add_argument("--seed", type=int, default=2000)
    p.add_argument("--detach", action="store_true", help="Run collection and training as a background job")
    p.add_argument("--output", type=Path, default=ROOT / "logs/rl_junction_pipeline")
    args = p.parse_args()
    if args.successes_per_task < 3 or args.max_attempts_per_task < args.successes_per_task or args.epochs < 1:
        p.error("Need >=3 successes per task, enough attempts, and positive epochs")
    if (args.recovery_successes_per_task not in (0,) and not 3 <= args.recovery_successes_per_task <= args.max_attempts_per_task):
        p.error("recovery-successes-per-task must be 0 or at least 3 and within the attempt budget")
    if args.dagger_episodes_per_task < 1:
        p.error("dagger-episodes-per-task must be positive")
    if os.environ.get("ROS_DOMAIN_ID") != "42" or os.environ.get("IGN_PARTITION") != "4wd_rl_junction":
        p.error("Source ROS and set ROS_DOMAIN_ID=42, IGN_PARTITION=4wd_rl_junction")
    if args.detach:
        args.output.mkdir(parents=True, exist_ok=True)
        log_path = args.output / ("launch_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f")+".log")
        with log_path.open("w") as log:
            job = subprocess.Popen([sys.executable, "-u", str(Path(__file__).resolve()),
                *[item for item in sys.argv[1:] if item != "--detach"]], cwd=ROOT,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True)
        print(json.dumps({"background_pid": job.pid, "log": str(log_path)}, indent=2), flush=True)
        return 0
    lock = open("/tmp/4wd_mlp_bc_pipeline.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        p.error("An MLP+BC collection/training pipeline is already running")
    run = args.output / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True)
    (run / "config.json").write_text(json.dumps(vars(args), default=str, indent=2))
    child = None
    stage = "starting"
    def status(state, **extra):
        (run / "status.json").write_text(json.dumps({"state": state, "stage": stage,
            "pid": os.getpid(), "run": str(run), **extra}, indent=2))
    def stop(*_):
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
        raise KeyboardInterrupt
    old_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    def execute(name, arguments):
        nonlocal child, stage
        stage = name
        status("running", command=arguments)
        print(f"STAGE {name}: {' '.join(arguments)}", flush=True)
        with (run / f"{name}.log").open("w") as log:
            child = subprocess.Popen(arguments, cwd=ROOT, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, close_fds=True)
            try:
                for line in child.stdout:
                    log.write(line)
                    log.flush()
                    print(line, end="", flush=True)
                result = child.wait()
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        child.terminate()
                        child.wait(timeout=10)
                child.stdout.close()
            if result:
                raise RuntimeError(f"{name} failed with exit code {result}; see {run / (name+'.log')}")
    print(f"Pipeline directory: {run}", flush=True)
    try:
        normal = args.normal_demos.resolve() if args.normal_demos else run / "demos/normal"
        roots = [str(normal)]
        if args.normal_demos:
            execute("validate_normal", [sys.executable, "rl_junction/train_bc.py", "--demos", str(normal), "--check-only"])
        else:
            execute("collect", [sys.executable, "-u", "rl_junction/collect_demos.py", "--task", "all",
                "--target-successes", str(args.successes_per_task),
                "--episodes-per-task", str(args.max_attempts_per_task), "--seed", str(args.seed),
                "--run-dir", str(normal)])
        if args.recovery_successes_per_task:
            execute("collect_recovery", [sys.executable, "-u", "rl_junction/collect_demos.py", "--task", "all",
                "--mode", "perturbed", "--target-successes", str(args.recovery_successes_per_task),
                "--episodes-per-task", str(args.max_attempts_per_task), "--seed", str(args.seed+100000),
                "--run-dir", str(run / "demos/recovery")])
            roots.append(str(run / "demos/recovery"))
        if args.dagger_model:
            execute("collect_dagger", [sys.executable, "-u", "rl_junction/collect_demos.py", "--task", "all",
                "--mode", "dagger", "--behavior-model", str(args.dagger_model),
                "--episodes-per-task", str(args.dagger_episodes_per_task), "--seed", str(args.seed+200000),
                "--run-dir", str(run / "demos/dagger")])
            roots.append(str(run / "demos/dagger"))
        reference = ["--split-from", str(args.split_from.resolve())] if args.split_from else []
        execute("validate", [sys.executable, "rl_junction/train_bc.py", "--demos", *roots, *reference, "--check-only"])
        execute("train", [sys.executable, "-u", "rl_junction/train_bc.py", "--demos", *roots, *reference,
            "--history", "4", "--hidden-sizes", "128", "128", "--epochs", str(args.epochs),
            "--seed", "0", "--output", str(run / "bc")])
        checkpoints = sorted((run / "bc").glob("*/bc_model.zip"))
        if not checkpoints:
            raise RuntimeError("Training finished without a checkpoint")
        status("complete", checkpoint=str(checkpoints[-1]))
        print(f"COMPLETE: {checkpoints[-1]}", flush=True)
    except KeyboardInterrupt:
        status("interrupted")
        print(f"Pipeline interrupted; completed data retained: {run}", flush=True)
        return 130
    except Exception as exc:
        status("failed", error=str(exc))
        print(str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        lock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
