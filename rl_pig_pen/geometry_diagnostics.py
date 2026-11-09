"""Optional compressed raw-LiDAR and local-geometry journal; no control inputs."""
import gzip
import json
import numpy as np


def encode(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class GeometryDiagnosticsLog:
    def __init__(self, directory, chunk_size=64):
        if chunk_size < 1:
            raise ValueError("chunk_size must be positive")
        directory.mkdir(parents=True, exist_ok=False)
        self.directory, self.chunk_size = directory, chunk_size
        self.log = gzip.open(directory/"decisions.jsonl.gz", "wt", encoding="utf-8")
        self.frames, self.chunk, self.count = [], 0, 0
        self.last_fault = False

    def record_scan(self, scan, stamp, controller, metadata=None):
        c = controller
        self.frames.append(np.asarray(scan.ranges, dtype=np.float32).copy())
        record = {"frame": self.count, "scan_stamp": stamp,
                  "scan_geometry": {"angle_min": scan.angle_min, "angle_increment": scan.angle_increment,
                                    "range_min": scan.range_min, "range_max": scan.range_max},
                  "state": c.state, "detections": c.geometry_diagnostics,
                  "controller": c.diagnostics,
                  "localization_recovery": None if c.relocalization is None else {
                      k: c.relocalization[k] for k in ("reason", "failed_state", "started", "frames")},
                  "tracker": None if c.tracker is None else {
                      "centre": c.tracker.geometry.centre, "axes": c.tracker.geometry.axes,
                      "widths": c.tracker.geometry.widths, "evidence": c.tracker.geometry.evidence,
                      "wall_matches": c.tracker.wall_diagnostics, "update": c.tracker.update_diagnostics},
                  "raw_chunk": self.chunk, "raw_index": len(self.frames)-1}
        if metadata:
            record.update(metadata)
        self.log.write(json.dumps(record, default=encode)+"\n")
        self.log.flush()
        self.count += 1
        fault = bool(c.failure or c.relocalization is not None)
        if len(self.frames) >= self.chunk_size or (fault and not self.last_fault):
            self.flush()
        self.last_fault = fault

    def flush(self):
        if self.frames:
            np.savez_compressed(self.directory/f"scans_{self.chunk:05d}.npz",
                                ranges=np.asarray(self.frames, dtype=np.float32))
            self.frames.clear()
            self.chunk += 1

    def close(self):
        if not self.log.closed:
            self.flush()
            self.log.close()
