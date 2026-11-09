#!/usr/bin/env python3
"""Copy unchanged weights into an explicitly recorded changed-geometry environment.

This is a transfer for re-evaluation, never proof of old-checkpoint compatibility.
Sensor/action layouts, map, configuration and robot must remain identical.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from learning_common import contract, model_path


def transfer(source, output):
    source = model_path(source)
    output = model_path(output)
    old = json.loads(source.with_suffix(".json").read_text())
    current = contract()
    changes = {k: {"before": old["environment"].get(k), "after": v}
               for k, v in current.items() if old["environment"].get(k) != v}
    permitted = {"localization_sha256", "fsm_sha256", "junction_env_sha256"}
    if not set(changes).issubset(permitted) or set(old["environment"]) != set(current):
        raise ValueError(f"Cannot transfer changed sensor/action/map/schema contract: {changes}")
    if output.exists() or output.with_suffix(".json").exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output)
    old.update(environment=current, transfer={
        "source": str(source.resolve()), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_environment": old["environment"], "changes": changes,
        "weights_unchanged": True, "requires_closed_loop_validation": True})
    output.with_suffix(".json").write_text(json.dumps(old, indent=2))
    print(f"Transferred unchanged weights for evaluation: {output}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    transfer(args.source, args.output)
