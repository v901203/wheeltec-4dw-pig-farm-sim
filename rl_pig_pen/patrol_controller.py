"""Topology-based patrol. No world coordinates or calibrated waypoints.

Only odometry deltas are used for short-range travel and relative headings.
`command` returns None when the corridor policy owns control, otherwise [v,w].
"""

import math
import numpy as np

from navigation import wrap_angle


class PatrolController:
    def __init__(self, cfg):
        self.cfg = cfg

    def reset(self, pose, stamp):
        self.state = "APPROACH"
        self.main_heading = pose[2]
        self.heading = pose[2]
        self.previous_pose = pose
        self.motion_pose = pose
        self.motion_stamp = stamp
        self.state_stamp = stamp
        self.junction = 0
        self.side = 0
        self.reached = set()
        self.returned = set()
        self.distance = 0.0
        self.branch_length = 0.0
        self.confirm = 0
        self.seen_open = False
        self.walls_seen = False
        self.failure = None
        self.motion_diagnostics = {}
        self.failure_diagnostics = {}

    def _transition(self, state, stamp):
        self.state = state
        self.state_stamp = stamp
        self.confirm = 0
        self.motion_stamp = stamp
        self.motion_pose = self.previous_pose

    def _confirmed(self, condition):
        self.confirm = self.confirm + 1 if condition else 0
        return self.confirm >= self.cfg.confirm_frames

    def _drive(self, yaw):
        error = wrap_angle(self.heading - yaw)
        return np.array([self.cfg.cruise_speed * max(0.0, math.cos(error)),
                         np.clip(2.0 * error, -self.cfg.turn_speed, self.cfg.turn_speed)])

    def _turn(self, heading, next_state, stamp):
        self.heading = wrap_angle(heading)
        self.after_turn = next_state
        self._transition("TURN", stamp)

    def _fail(self, reason, stamp):
        self.failure = reason
        self.failure_diagnostics = dict(self.motion_diagnostics, failed_state=self.state)
        self._transition("FAILED", stamp)

    def command(self, features, pose, stamp):
        """Update exactly once per fresh scan, then arbitrate control ownership."""
        cfg = self.cfg
        dx, dy = pose[0] - self.previous_pose[0], pose[1] - self.previous_pose[1]
        if self.state not in ("TURN", "DONE", "FAILED"):
            # Signed progress cannot be accumulated by circling or backing up.
            self.distance += dx * math.cos(self.heading) + dy * math.sin(self.heading)
        self.previous_pose = pose

        displacement = math.hypot(pose[0] - self.motion_pose[0], pose[1] - self.motion_pose[1])
        rotation = abs(wrap_angle(pose[2] - self.motion_pose[2]))
        # PPO may rotate in place too; physical immobility is independent of
        # which controller requested the turn. State timeout still bounds loops.
        if displacement > 0.05 or rotation > 0.08:
            self.motion_pose, self.motion_stamp = pose, stamp
        self.motion_diagnostics = dict(motion_distance_m=displacement, motion_angle_rad=rotation,
                                       no_motion_sim_seconds=stamp-self.motion_stamp,
                                       state_sim_seconds=stamp-self.state_stamp)
        if self.state not in ("DONE", "FAILED"):
            if stamp - self.motion_stamp > cfg.stuck_seconds:
                self._fail("stuck", stamp)
            elif stamp - self.state_stamp > cfg.state_timeout:
                self._fail("state_timeout", stamp)

        if self.state in ("DONE", "FAILED"):
            return np.zeros(2)

        if self.state == "TURN":
            error = wrap_angle(self.heading - pose[2])
            if self._confirmed(abs(error) < cfg.yaw_tolerance):
                self.distance = 0.0
                self.walls_seen = False
                self.seen_open = False
                self._transition(self.after_turn, stamp)
                return np.zeros(2)
            return np.array([0.0, np.clip(2.0 * error, -cfg.turn_speed, cfg.turn_speed)])

        if self.state in ("APPROACH", "EXIT"):
            if self._confirmed(features.walls_present(cfg)):
                self.distance = 0.0
                self.walls_seen = True
                self._transition("MAIN", stamp)
                return None
            if self.distance > cfg.max_main_run:
                self._fail("main_corridor_not_found", stamp)
                return np.zeros(2)
            return self._drive(pose[2])

        if self.state == "MAIN":
            if self._confirmed(features.both_open(cfg) and self.distance >= cfg.min_main_run):
                if len(self.returned) == 2 * cfg.junctions:
                    self._transition("DONE", stamp)
                else:
                    self.junction += 1
                    self.side = 0
                    self.distance = 0.0
                    self._transition("CENTER", stamp)
                return np.zeros(2)
            if self.distance > cfg.max_main_run:
                self._fail("junction_or_main_end_not_found", stamp)
                return np.zeros(2)
            # Slow at the first sign of an opening while confirming the event.
            return self._drive(pose[2]) if features.both_open(cfg) else None

        if self.state == "CENTER":
            if self.distance >= cfg.junction_advance:
                self._turn(self.main_heading + math.pi / 2, "ENTRY", stamp)
                return np.zeros(2)
            return self._drive(pose[2])

        if self.state == "ENTRY":
            if self._confirmed(features.walls_present(cfg) and self.distance >= cfg.entry_distance):
                self.walls_seen = True
                self._transition("OUTBOUND", stamp)
                return None
            if self.distance > cfg.max_branch_run:
                self._fail("branch_corridor_not_found", stamp)
                return np.zeros(2)
            return self._drive(pose[2])

        if self.state == "OUTBOUND":
            if self._confirmed(self.walls_seen and features.both_open(cfg) and
                               self.distance >= cfg.min_branch_run):
                self.branch_length = self.distance
                self.reached.add((self.junction, self.side))
                self._turn(self.heading + math.pi, "RETURN", stamp)
                return np.zeros(2)
            if self.distance > cfg.max_branch_run:
                self._fail("branch_end_not_found", stamp)
                return np.zeros(2)
            return self._drive(pose[2]) if features.both_open(cfg) else None

        if self.state == "RETURN":
            # Leave the exposed branch endpoint under deterministic heading control.
            if features.walls_present(cfg):
                self.walls_seen = True
            if self.distance >= self.branch_length - cfg.return_takeover:
                self._transition("RETURN_CENTER", stamp)
                return self._drive(pose[2])
            return None if self.walls_seen else self._drive(pose[2])

        if self.state == "RETURN_CENTER":
            if self._confirmed(features.both_open(cfg)):
                self.seen_open = True
            if self.distance >= self.branch_length - cfg.return_tolerance:
                if not self.seen_open:
                    self._fail("return_junction_not_confirmed", stamp)
                    return np.zeros(2)
                self.returned.add((self.junction, self.side))
                if self.side == 0:
                    # First return already faces the opposite branch.
                    self.side = 1
                    self.distance = 0.0
                    self.walls_seen = False
                    self._transition("ENTRY", stamp)
                else:
                    self._turn(self.main_heading, "EXIT", stamp)
                return np.zeros(2)
            return self._drive(pose[2])

        raise RuntimeError(f"Unknown patrol state: {self.state}")

    def info(self):
        total = 2 * self.cfg.junctions
        return {
            "state": self.state,
            "junction": self.junction,
            "side": self.side,
            "endpoints_reached": len(self.reached),
            "branches_done": len(self.returned),
            "branches_total": total,
            "coverage": len(self.reached) / total,
            "return_rate": len(self.returned) / total,
            "success": self.state == "DONE",
            "failure_reason": self.failure,
            **(self.failure_diagnostics if self.failure else self.motion_diagnostics),
        }
