#!/usr/bin/env python3
"""Audit localization against simulator truth OFFLINE; never a policy input."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
from learning_common import contract
from tasks import TASKS, CENTRE_Y


def assess(directory, minimum=2, limit=.1):
    report = json.loads((directory/"evaluation.json").read_text())
    counters = {t.name: {"episodes": 0, "successes": 0, "geometry_frames": 0,
                           "wrong_centre_frames": 0, "max_centre_error_m": 0.} for t in TASKS}
    geometry_reasons = {"junction_geometry_lost", "junction_geometry_conflict", "junction_position_jump",
                        "junction_axis_jump", "junction_scan_gap", "scan_gap"}
    unfinished = []
    for episode in report["episodes"]:
        counters[episode["task"]]["episodes"] += 1
        counters[episode["task"]]["successes"] += int(episode["success"])
        if episode.get("recovery_reason") in geometry_reasons:
            unfinished.append({"task": episode["task"], "seed": episode["seed"],
                               "reason": episode["recovery_reason"]})
    with gzip.open(directory/"geometry/decisions.jsonl.gz", "rt") as log:
        for line in log:
            r = json.loads(line)
            if not r["context"][7]:
                continue
            x, y, h = r["diagnostic_pose_only"]
            bx, by = [v*2 for v in r["context"][4:6]]
            error = math.hypot(x+math.cos(h)*bx-math.sin(h)*by,
                               y+math.sin(h)*bx+math.cos(h)*by-CENTRE_Y)
            counter = counters[r["task"]]
            counter["geometry_frames"] += 1
            counter["max_centre_error_m"] = max(counter["max_centre_error_m"], error)
            counter["wrong_centre_frames"] += int(error > limit)
    passed = (report["complete"] and report["contract"] == contract() and not unfinished and
              all(v["episodes"] >= minimum and v["geometry_frames"] > 0 and
                  v["wrong_centre_frames"] == 0 for v in counters.values()))
    model = Path(report["model"])
    result = {"passed": bool(passed), "contract": report["contract"], "evaluation": str(directory.resolve()),
              "model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
              "minimum_episodes_per_task": minimum, "centre_error_limit_m": limit,
              "tasks": counters, "unfinished_geometry_recoveries": unfinished,
              "note": "Finite localization regression gate; not a claim of deployment safety or navigation success rate."}
    (directory/"geometry_gate.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evaluation", type=Path, required=True)
    p.add_argument("--minimum-episodes", type=int, default=2)
    args = p.parse_args()
    if args.minimum_episodes < 1:
        p.error("minimum-episodes must be positive")
    result = assess(args.evaluation, args.minimum_episodes)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 2)
