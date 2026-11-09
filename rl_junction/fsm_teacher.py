"""Adapter for the project's real LiDAR FSM; no world pose enters the teacher."""
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rl_pig_pen"))
from lidar_patrol_controller import TraditionalPatrolController

TEACHER_ID = "lidar_fsm_v1"
START_STATES = {"main_to_left": "MAIN", "left_to_right_reverse": "REVERSE_LEFT",
                "right_to_main": "RETURN_RIGHT"}
HANDOFF_STATES = {"main_to_left": "LEFT_OUTBOUND",
                  "left_to_right_reverse": "RIGHT_OUTBOUND", "right_to_main": "MAIN"}
SKILL_STATES = {
    "main_to_left": ("MAIN", "CENTER", "TURN_LEFT", "ENTRY_LEFT"),
    "left_to_right_reverse": ("REVERSE_LEFT", "CROSS_REVERSE"),
    "right_to_main": ("RETURN_RIGHT", "RETURN_CENTER", "TURN_MAIN", "EXIT_MAIN"),
}


class FSMTeacher:
    def __init__(self, task, cfg):
        # Only speed limits are adapted to the learning action space.
        self.cfg = replace(cfg, cruise_speed=min(cfg.cruise_speed, .2),
                           turn_speed=min(cfg.turn_speed, .6))
        self.controller = TraditionalPatrolController(self.cfg)
        self.controller.state = START_STATES[task.name]

    def command(self, features, stamp):
        output = np.asarray(self.controller.command(features, stamp), dtype=np.float32)
        if output.shape != (2,) or not np.isfinite(output).all():
            raise ValueError("FSM produced an invalid command")
        return output

    def status(self):
        c = self.controller
        return {"state": c.state, "failure": c.failure,
                "entry_phase": c.entry_phase,
                "obstacle": c.obstacle is not None,
                "relocalizing": c.relocalization is not None,
                "diagnostics": c.diagnostics}
