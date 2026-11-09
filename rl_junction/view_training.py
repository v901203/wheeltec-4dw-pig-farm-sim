#!/usr/bin/env python3
"""Read-only console monitor and CSV-to-TensorBoard exporter; no robot control."""
import argparse
import csv
import json
import math
from pathlib import Path
import time


def metrics(run):
    with (run/"metrics/progress.csv").open(newline="") as log:
        return list(csv.DictReader(log))


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--follow", action="store_true", help="Refresh until training completes; Ctrl-C stops viewing only")
    args = parser.parse_args()
    from tensorboard.compat.proto.event_pb2 import Event
    from tensorboard.compat.proto.summary_pb2 import Summary
    from tensorboard.summary.writer.event_file_writer import EventFileWriter
    output = args.run/"tensorboard"
    writer = EventFileWriter(str(output))
    seen, last_printed = set(), None
    try:
        while True:
            try:
                rows = metrics(args.run)
            except (OSError, csv.Error):
                rows = []  # The live CSV may briefly be rewriting its header.
            latest = {}
            for row in rows:
                step = number(row.get("time/total_timesteps"))
                if step is None:
                    continue
                for tag, value in row.items():
                    scalar = number(value)
                    if scalar is None:
                        continue
                    latest[tag] = scalar
                    key = (int(step), tag, scalar)
                    if key in seen:
                        continue
                    seen.add(key)
                    writer.add_event(Event(wall_time=time.time(), step=int(step),
                        summary=Summary(value=[Summary.Value(tag=tag, simple_value=scalar)])))
            writer.flush()
            keys = ("time/total_timesteps", "time/run_timesteps", "rollout/ep_rew_mean",
                    "train/explained_variance", "train/value_loss", "train/approx_kl")
            current = {key: latest[key] for key in keys if key in latest}
            if current and current != last_printed:
                print(json.dumps(current, ensure_ascii=False), flush=True)
                last_printed = current
            if not args.follow or any((args.run/name).exists() for name in
                    ("completion.json", "interrupted.json", "infrastructure_error.json")):
                break
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        writer.close()
    print(f"TensorBoard directory: {output}", flush=True)


if __name__ == "__main__":
    main()
