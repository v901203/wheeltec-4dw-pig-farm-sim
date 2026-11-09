"""Training adapter for the same optional geometry journal used on the robot."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"rl_pig_pen"))
from geometry_diagnostics import GeometryDiagnosticsLog


class GeometryRecorder(GeometryDiagnosticsLog):
    def record(self, env, task, seed, step):
        self.record_scan(env.scan, env.scan_stamp, env.teacher.controller, {
            "task": task, "seed": seed, "step": step,
            "pose_stamp": env.pose_stamp, "odom_stamp": env.odom_stamp, "clock_stamp": env.sim_time,
            "diagnostic_pose_only": env.pose.copy(), "context": env.cached_context.copy(),
            "expert": env.cached_command.copy()})
