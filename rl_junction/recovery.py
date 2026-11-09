"""Small, bounded simulation disturbances for expert recovery demonstrations."""
import math
import numpy as np
from policy_context import PHASE_STATES


def recovery_offset(controller, rng, case="mixed", phase="center"):
    """Return body-frame dx/dy/dyaw or None when not in the requested phase.

    Planning uses LiDAR only. World pose is used by the simulator solely to apply
    the disturbance, not to choose expert labels or construct observations.
    """
    index = PHASE_STATES.get(controller.state, -1)
    target = {"center": 1, "turn": 2, "entry": 3}[phase]
    if index != target:
        return None
    tracker = controller.tracker
    if tracker is None or controller.obstacle or controller.relocalization:
        return None
    chosen = case
    if case == "mixed":
        chosen = str(rng.choice(["lateral", "yaw", "overshoot"] if target == 1 else ["lateral", "yaw"]))
    centre = tracker.geometry.centre
    if target == 1 and np.linalg.norm(centre) > .35:
        return None
    offset = np.zeros(3)
    if chosen == "overshoot":
        if target != 1:
            return None
        direction = np.array([math.cos(tracker.incoming_axis), math.sin(tracker.incoming_axis)])
        along = float(centre @ direction)
        if not .01 <= along <= .055:
            return None
        # Go beyond the 5 cm turn-ready region, but remain inside the 15 cm
        # recovery envelope so the expert must demonstrate reverse correction.
        offset[:2] = direction * (along + float(rng.uniform(.06, .08)))
    elif chosen == "lateral":
        # 2–4 cm is inside existing tracking bounds; geometry/safety still decide recovery.
        offset[1] = float(rng.choice([-1, 1])) * float(rng.uniform(.02, .04))
    elif chosen == "yaw":
        offset[2] = float(rng.choice([-1, 1])) * float(rng.uniform(math.radians(2), math.radians(4)))
    else:
        raise ValueError("Unknown recovery case")
    return chosen, offset
