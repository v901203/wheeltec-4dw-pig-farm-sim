"""LiDAR-topology patrol state machine.

The controller remembers route order, but never consumes odometry: openings,
side walls, end walls and wall alignment are all inferred from the current
LiDAR scan. PPO owns translation in every live state; the FSM only rotates in
``TURN`` and issues one-frame stops while changing state.
"""

import math

import numpy as np

from junction_localization import JunctionTracker, junction_geometry
from lidar_geometry import axis_angle


POLICY_STATES = (
    "APPROACH", "MAIN", "CENTER", "ENTRY",
    "OUTBOUND", "RETURN", "RETURN_CENTER", "EXIT",
)


class PatrolController:
    def __init__(self, cfg):
        self.cfg = cfg

    def reset(self, _pose=None, stamp=0.0):
        """Reset route memory. ``_pose`` remains accepted for old callers.

        It is intentionally ignored: physical patrol operation has no odom
        dependency.
        """
        self.state = "APPROACH"
        self.state_stamp = float(stamp)
        self.state_frames = 0
        self.junction = 0
        self.side = 0
        self.reached = set()
        self.returned = set()
        self.return_reverse = False
        self.confirm = 0
        self.end_confirm = 0
        self.seen_open = False
        self.walls_seen = False
        self.after_turn = None
        self.turn_direction = 0.0
        self.failure = None
        self.motion_diagnostics = {}
        self.failure_diagnostics = {}
        self.junction_tracker = None
        self.geometry_missing_since = None
        self.last_scan_stamp = None

    def _transition(self, state, stamp):
        self.state = state
        self.state_stamp = float(stamp)
        self.state_frames = 0
        self.confirm = 0
        self.end_confirm = 0

    def _confirmed(self, condition):
        self.confirm = self.confirm + 1 if condition else 0
        return self.confirm >= self.cfg.confirm_frames

    def _end_confirmed(self, condition):
        self.end_confirm = self.end_confirm + 1 if condition else 0
        return self.end_confirm >= self.cfg.confirm_frames

    def expect_reverse(self):
        return ((self.side == 1 and self.state in ("ENTRY", "OUTBOUND")) or
                (self.state in ("RETURN", "RETURN_CENTER") and self.return_reverse))

    def _start_turn(self, direction, next_state, stamp):
        self.turn_direction = float(direction)
        self.after_turn = next_state
        self._transition("TURN", stamp)

    def _fail(self, reason, stamp):
        self.failure = reason
        self.failure_diagnostics = dict(self.motion_diagnostics, failed_state=self.state)
        self._transition("FAILED", stamp)

    def _missing_geometry(self, stamp):
        self.confirm = 0
        if self.geometry_missing_since is None:
            self.geometry_missing_since = stamp
        self.motion_diagnostics.update(junction_geometry_valid=False,
                                       geometry_missing_seconds=stamp-self.geometry_missing_since)
        if stamp-self.geometry_missing_since >= self.cfg.junction_geometry_timeout:
            self._fail("junction_geometry_lost", stamp)

    def _acquire_junction(self, features, stamp):
        geometry = junction_geometry(features, self.cfg)
        if geometry is None:
            self._missing_geometry(stamp)
            return False
        tracker = JunctionTracker(geometry, stamp, self.cfg)
        # Establish the incoming road while approximately facing along it.
        if abs(tracker.incoming_axis) > math.radians(20):
            self._missing_geometry(stamp)
            return False
        direction = np.array([math.cos(tracker.incoming_axis), math.sin(tracker.incoming_axis)])
        travel_sign = -1.0 if self.expect_reverse() else 1.0
        if travel_sign*float(geometry.centre @ direction) < -self.cfg.junction_centre_tolerance:
            # Rail gaps may expose the junction again AFTER EXIT. Its measured
            # centre is behind us: never count the same crossing a second time.
            self.confirm = 0
            self.geometry_missing_since = None
            return False
        self.junction_tracker = tracker
        self.geometry_missing_since = None
        return True

    def _track_junction(self, features, stamp):
        geometry = junction_geometry(features, self.cfg)
        if geometry is None:
            self._missing_geometry(stamp)
            return None
        tracker = self.junction_tracker
        if tracker is None:
            self._fail("junction_reference_missing", stamp)
            return None
        if not tracker.update(geometry, stamp):
            self._fail(tracker.failure, stamp)
            return None
        self.geometry_missing_since = None
        self.motion_diagnostics.update(junction_geometry_valid=True,
                                       junction_confidence=geometry.confidence,
                                       centre_error_m=float(np.linalg.norm(geometry.centre)),
                                       centre_x_m=float(geometry.centre[0]),
                                       centre_y_m=float(geometry.centre[1]),
                                       lidar_delta_yaw=float(tracker.delta_pose[2]))
        return geometry

    def _at_centre(self, geometry):
        return (np.linalg.norm(geometry.centre) <= self.cfg.junction_centre_tolerance and
                abs(axis_angle(self.junction_tracker.incoming_axis)) <= self.cfg.yaw_tolerance)

    def _junction_opening(self, features):
        """A branch can expose one side first when returning off-centre."""
        return features.junction_open(self.cfg, both=False)

    def context(self, features):
        """Return the 11 LiDAR-only context dimensions appended to the scan."""
        if self.state not in POLICY_STATES:
            raise RuntimeError(f"No PPO context for state {self.state}")
        state = np.zeros(len(POLICY_STATES), dtype=np.float32)
        state[POLICY_STATES.index(self.state)] = 1.0
        # A wall has no forward/backward direction, hence the pi-periodic error.
        parallel_error = np.clip(features.wall_parallel_error / (math.pi / 2.0), -1.0, 1.0)
        return np.concatenate((state, [parallel_error, features.wall_alignment_confidence,
                                       float(self.expect_reverse())])).astype(np.float32)

    def observation(self, features):
        return np.concatenate((features.observation, self.context(features))).astype(np.float32)

    def command(self, features, pose_or_stamp=None, stamp=None):
        """Update once per fresh scan and return an FSM override or ``None``.

        Both old ``(features, pose, stamp)`` and LiDAR-only ``(features,
        stamp)`` calls are accepted. Pose data is never read.
        """
        if stamp is None:
            stamp = pose_or_stamp
        if stamp is None:
            raise ValueError("A LiDAR scan timestamp is required")
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError("A finite LiDAR timestamp is required")
        # A cached scan must not accumulate confirmation or movement.
        if self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            return np.zeros(2)
        self.last_scan_stamp = stamp
        self.state_frames += 1
        self.motion_diagnostics = dict(state_sim_seconds=stamp-self.state_stamp,
                                       scan_frames=self.state_frames)

        if self.state not in ("DONE", "FAILED") and stamp - self.state_stamp > self.cfg.state_timeout:
            self._fail("state_timeout", stamp)

        if self.state in ("DONE", "FAILED"):
            return np.zeros(2)

        if self.state == "TURN":
            geometry = self._track_junction(features, stamp)
            if geometry is None:
                return np.zeros(2)
            if np.linalg.norm(geometry.centre) > self.cfg.junction_turn_centre_limit:
                self._fail("junction_turn_drift", stamp)
                return np.zeros(2)
            # Track the SAME incoming axis throughout the maneuver, then aim
            # at its chosen perpendicular. Any parallel wall is not enough.
            target = self.junction_tracker.incoming_axis + self.turn_direction*math.pi/2
            error = math.atan2(math.sin(target), math.cos(target))
            self.motion_diagnostics["turn_error_rad"] = error
            at_target = abs(error) <= self.cfg.yaw_tolerance
            if self._confirmed(at_target):
                self._transition(self.after_turn, stamp)
                self.junction_tracker = None
                return np.zeros(2)
            return np.array([0.0, 0.0 if at_target else
                             np.clip(self.cfg.turn_kp*error, -self.cfg.turn_speed, self.cfg.turn_speed)])

        if self.state == "APPROACH":
            if self._confirmed(features.walls_present(self.cfg) and features.aligned()):
                self._transition("MAIN", stamp)
                return np.zeros(2)
            return None

        if self.state == "MAIN":
            if self._end_confirmed(features.end_wall_present(self.cfg)):
                if len(self.returned) == 2 * self.cfg.junctions:
                    self._transition("DONE", stamp)
                else:
                    self._fail("main_end_before_patrol_complete", stamp)
                return np.zeros(2)
            if self._confirmed(features.junction_open(self.cfg)) and self.junction < self.cfg.junctions:
                if self._acquire_junction(features, stamp):
                    self.junction += 1
                    self.side = 0
                    self._transition("CENTER", stamp)
                    return np.zeros(2)
                return np.zeros(2) if self.geometry_missing_since is not None else None
            return None

        if self.state == "CENTER":
            geometry = self._track_junction(features, stamp)
            if geometry is None:
                return np.zeros(2)
            at_target = self._at_centre(geometry)
            if self._confirmed(at_target):
                self._start_turn(+1.0, "ENTRY", stamp)
                return np.zeros(2)
            return np.zeros(2) if at_target else None

        if self.state == "ENTRY":
            if self._confirmed(features.walls_present(self.cfg) and features.aligned()):
                self.walls_seen = True
                self._transition("OUTBOUND", stamp)
                return np.zeros(2)
            return None

        if self.state == "OUTBOUND":
            if self._end_confirmed(self.walls_seen and features.end_wall_present(
                    self.cfg, reverse=self.expect_reverse())):
                self.reached.add((self.junction, self.side))
                self.return_reverse = self.side == 0
                self._transition("RETURN", stamp)
                return np.zeros(2)
            return None

        if self.state == "RETURN":
            if self._confirmed(self._junction_opening(features)):
                if self._acquire_junction(features, stamp):
                    self.seen_open = True
                    self._transition("RETURN_CENTER", stamp)
                    return np.zeros(2)
                return np.zeros(2) if self.geometry_missing_since is not None else None
            return None

        if self.state == "RETURN_CENTER":
            geometry = self._track_junction(features, stamp)
            if geometry is None:
                return np.zeros(2)
            at_target = self._at_centre(geometry)
            if self._confirmed(at_target):
                if not self.seen_open:
                    self._fail("return_junction_not_confirmed", stamp)
                    return np.zeros(2)
                self.returned.add((self.junction, self.side))
                if self.side == 0:
                    # Keep reversing through the junction into the opposite branch.
                    self.side = 1
                    self.return_reverse = False
                    self.walls_seen = False
                    self._transition("ENTRY", stamp)
                    self.junction_tracker = None
                else:
                    self.return_reverse = False
                    self._start_turn(-1.0, "EXIT", stamp)
                return np.zeros(2)
            return np.zeros(2) if at_target else None

        if self.state == "EXIT":
            if self._confirmed(features.walls_present(self.cfg) and features.aligned()):
                self._transition("MAIN", stamp)
                return np.zeros(2)
            return None

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
            "expect_reverse": self.expect_reverse(),
            "success": self.state == "DONE",
            "failure_reason": self.failure,
            **(self.failure_diagnostics if self.failure else self.motion_diagnostics),
        }
