"""Causal policy context from the current LiDAR FSM decision; never world pose."""
import math
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rl_pig_pen"))
from lidar_patrol_controller import incoming_corridor
from navigation import wrap_angle

RAW_OBS_SIZE = 151
PHASE_NAMES = ("APPROACH", "CENTER", "TURN", "ENTRY")
PHASE_STATES = {
    "MAIN": 0, "REVERSE_LEFT": 0, "RETURN_RIGHT": 0,
    "CENTER": 1, "RETURN_CENTER": 1,
    "TURN_LEFT": 2, "TURN_MAIN": 2,
    "CROSS_REVERSE": 3, "ENTRY_LEFT": 3, "EXIT_MAIN": 3,
    "LEFT_OUTBOUND": 3, "RIGHT_OUTBOUND": 3,
}
CONTEXT_SIZE = 10
OBS_SIZE = RAW_OBS_SIZE + CONTEXT_SIZE
DISTANCE_SCALE = 2.0


def decision_context(controller, features, stamp):
    """4 phase flags, body centre x/y, heading error, 2 validity bits, reposition bit.

    Must be called AFTER processing this scan exactly once through FSM.command.
    Missing/old geometry is zero-filled with validity=0, not extrapolated.
    """
    phase = PHASE_STATES.get(controller.state)
    one_hot = np.zeros(4, dtype=np.float32)
    if phase is not None:
        one_hot[phase] = 1.
    values = np.zeros(6, dtype=np.float32)
    fault = (controller.failure is not None or controller.relocalization is not None or
             controller.obstacle is not None or controller.external_paused)
    tracker = controller.tracker
    fresh = (not fault and tracker is not None and abs(tracker.stamp-stamp) < 1e-6 and
             tracker.geometry.confidence >= controller.cfg.junction_min_confidence)
    if fresh:
        centre = np.asarray(tracker.geometry.centre)
        heading = tracker.incoming_axis
        if controller.state in ("TURN_LEFT", "ENTRY_LEFT"):
            heading += math.pi/2
        elif controller.state in ("TURN_MAIN", "EXIT_MAIN"):
            heading -= math.pi/2
        if np.isfinite(centre).all() and math.isfinite(heading):
            values[:2] = np.clip(centre/DISTANCE_SCALE, -1., 1.)
            values[2] = wrap_angle(heading)/math.pi
            values[3:5] = 1.
    elif not fault:
        corridor = incoming_corridor(features, controller.cfg)
        if corridor is not None and math.isfinite(corridor[0]):
            values[2] = wrap_angle(corridor[0])/math.pi
            values[4] = 1.
    values[5] = float(controller.centre_retreat)
    return np.concatenate((one_hot, values))


def phase_id(observation):
    phase = np.asarray(observation)[RAW_OBS_SIZE:RAW_OBS_SIZE+4]
    if phase.shape != (4,) or not np.isclose(phase.sum(), 1.):
        return -1
    return int(np.argmax(phase))
