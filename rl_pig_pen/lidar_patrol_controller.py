"""LiDAR-only traditional patrol. No ROS, odometry or timed manoeuvres."""

import math
from dataclasses import replace

import numpy as np

from junction_localization import JunctionTracker, junction_geometry
from lidar_geometry import axis_angle, wall_lines
from navigation import (MAX_ANG, MAX_LIN, MIN_WALL_CONFIDENCE,
                        command_clearance_masks, safe_command, wrap_angle)


def incoming_corridor(features, cfg):
    """Fit the nearby forward corridor, excluding perpendicular pen walls.

    This requires starting within 30 degrees of the incoming road. Opposite
    wall support is mandatory; a single line cannot authorize forward motion.
    """
    lines = [line for line in wall_lines(features.ranges, features.angles, 1.5)
             if abs(line.angle) < math.radians(30) and line.span >= .5]
    candidates = []
    for index, a in enumerate(lines):
        for b in lines[index + 1:]:
            if abs(axis_angle(a.angle - b.angle)) > math.radians(8):
                continue
            angle = a.angle + axis_angle(b.angle - a.angle) / 2
            normal = np.array([-math.sin(angle), math.cos(angle)])
            da = a.offset * (1 if a.normal @ normal >= 0 else -1)
            db = b.offset * (1 if b.normal @ normal >= 0 else -1)
            if da * db >= 0 or min(abs(da), abs(db)) <= cfg.half_width:
                continue
            if max(abs(da), abs(db)) > cfg.wall_distance:
                continue
            quality = 1 - max(a.residual, b.residual) / .06
            if quality >= MIN_WALL_CONFIDENCE:
                candidates.append((quality * min(a.count, b.count), angle, da + db))
    return max(candidates)[1:] if candidates else None


class TraditionalPatrolController:
    """Visit both branches at each crossing, then continue down the main road.

    LEFT_OUTBOUND drives forward. REVERSE_LEFT and RIGHT_OUTBOUND keep the
    same body heading while reversing through the crossing. RETURN_RIGHT
    drives forward back to its centre before a clockwise turn onto MAIN.
    No world coordinates, odometry or command integration are consumed.
    """

    TERMINAL = ("DONE", "FAILED")
    # One bounded escape per encounter, using junction displacement or
    # corridor obstacle clearance depending on available observations.
    # Time is only an abort watchdog, never a distance/completion estimate.
    OBSTACLE_RETREAT_SPEED = .05
    OBSTACLE_RETREAT_DISTANCE = .10
    OBSTACLE_RETREAT_TIMEOUT = 4.0
    OBSTACLE_RELEASE_MARGIN = .04
    RETREAT_STATES = ("CENTER", "RETURN_CENTER", "ENTRY_LEFT", "CROSS_REVERSE", "EXIT_MAIN")
    CORRIDOR_STATES = ("MAIN", "FINAL_MAIN", "LEFT_OUTBOUND", "REVERSE_LEFT",
                       "RIGHT_OUTBOUND", "RETURN_RIGHT")
    ENTRY_STATES = ("ENTRY_LEFT", "CROSS_REVERSE", "EXIT_MAIN")
    # These failures describe a lost or inconsistent local pose, not a proven
    # collision.  Hold position and reacquire current geometry before deciding
    # that the route itself has failed.
    RELOCALIZATION_REASONS = (
        "junction_position_jump", "junction_axis_jump", "junction_scan_gap", "scan_gap",
        "junction_geometry_lost", "centre_overshoot", "centre_reposition_limit",
        "turn_centre_drift", "entry_not_confirmed", "state_timeout",
    )

    def __init__(self, cfg):
        if cfg.entry_distance >= cfg.junction_acquire_distance - .15:
            raise ValueError("entry_distance must remain inside the junction tracking range")
        # Position is a turn-entry region, not an exact parking target.
        # Honour the configured region (default 5 cm) from the first attempt.
        self.cfg = cfg
        self.state = "MAIN"
        self.failure = None
        self.tracker = None
        self.last_stamp = None
        self.state_stamp = None
        self.confirm = 0
        self.junctions_done = 0
        self.endpoints_reached = 0
        self.diagnostics = {}
        self.failure_diagnostics = {}
        self.centre_retreat = False
        self.centre_retries = 0
        self.centre_limit_confirm = 0
        self.obstacle = None
        self.entry_phase = "JUNCTION"
        self.relocalization = None
        self.relocalization_attempts = 0

    def fail(self, reason):
        if reason in self.RELOCALIZATION_REASONS and self.state not in self.TERMINAL:
            return self._start_relocalization(reason)
        if self.state not in self.TERMINAL:
            self.failure_diagnostics = dict(self.diagnostics, failed_state=self.state)
            self.failure = reason
            self.state = "FAILED"
        return np.zeros(2)

    def _start_relocalization(self, reason):
        """Stop without discarding route progress, then seek a stable local pose."""
        if self.relocalization is None:
            self.relocalization_attempts += 1
            heading_hint = (0.0 if self.tracker is None
                            else float(self.tracker.incoming_axis))
            self.relocalization = {
                "reason": reason,
                "failed_state": self.state,
                "started": self.last_stamp,
                "frames": 0,
                "candidate": None,
                "heading_hint": heading_hint,
                "diagnostics": dict(self.diagnostics),
            }
        self.confirm = self.centre_limit_confirm = 0
        self.centre_retreat = False
        return np.zeros(2)

    def _fresh_junction_geometry(self, features):
        """Acquire a crossing independently of the possibly stale tracker."""
        for scale in (1.0, .8, .65):
            geometry = junction_geometry(features, replace(
                self.cfg, junction_fit_radius=scale * self.cfg.junction_fit_radius))
            if geometry is not None:
                return geometry
        return None

    def _finish_relocalization(self, stamp, tracker=None, *, corridor=False):
        recovery = self.relocalization
        self.tracker = tracker
        if corridor and self.state in self.ENTRY_STATES:
            self.entry_phase = "WALL_CONFIRM"
        self.relocalization = None
        self.confirm = self.centre_limit_confirm = 0
        self.centre_retreat = False
        self.centre_retries = 0
        # Recovery time is a stationary safety hold, not manoeuvre time.
        self.state_stamp = stamp
        self.diagnostics = {
            "relocalization_status": "recovered",
            "relocalization_reason": recovery["reason"],
            "relocalization_attempt": self.relocalization_attempts,
            "relocalization_source": "corridor_walls" if corridor else "four_openings",
        }
        return np.zeros(2)

    def _relocalization_command(self, features, stamp):
        """Require consecutive self-consistent measurements while stopped."""
        recovery = self.relocalization
        self.diagnostics = {
            "relocalization_status": "waiting",
            "relocalization_reason": recovery["reason"],
            "relocalization_state": recovery["failed_state"],
            "relocalization_attempt": self.relocalization_attempts,
            "relocalization_frames": recovery["frames"],
            "relocalization_required_frames": self.cfg.confirm_frames,
        }

        # Near the branch handoff the complete four-opening junction may have
        # legitimately left the field of view.  Stable paired corridor walls
        # are sufficient to resume the already selected route state.
        corridor = (incoming_corridor(features, self.cfg)
                    if self.state in self.ENTRY_STATES + self.CORRIDOR_STATES else None)
        if self.state in self.ENTRY_STATES:
            wall_ready = self._entry_wall_ready(features, corridor)
            if wall_ready:
                recovery["frames"] += 1
                recovery["candidate"] = None
                self.diagnostics.update(relocalization_source="corridor_walls",
                                        relocalization_frames=recovery["frames"])
                if recovery["frames"] >= self.cfg.confirm_frames:
                    return self._finish_relocalization(stamp, corridor=True)
                return np.zeros(2)

        # Straight-road states have no longitudinal landmark to reacquire.
        # Reconfirm both corridor walls and heading, then continue the same
        # semantic route state; never infer travelled distance while stopped.
        if self.state in self.CORRIDOR_STATES and corridor is not None:
            recovery["frames"] += 1
            recovery["candidate"] = None
            self.diagnostics.update(relocalization_source="corridor_walls",
                                    relocalization_frames=recovery["frames"])
            if recovery["frames"] >= self.cfg.confirm_frames:
                return self._finish_relocalization(stamp, corridor=True)
            return np.zeros(2)

        geometry = self._fresh_junction_geometry(features)
        if geometry is None:
            recovery["frames"] = 0
            recovery["candidate"] = None
            self.diagnostics["relocalization_source"] = "unavailable"
            return np.zeros(2)

        candidate = recovery["candidate"]
        if candidate is None:
            candidate = JunctionTracker(geometry, stamp, self.cfg,
                                        heading_hint=recovery["heading_hint"])
            recovery["candidate"] = candidate
            recovery["frames"] = 1
        elif candidate.update(geometry, stamp):
            recovery["frames"] += 1
        else:
            # The proposed replacement also jumped. Start a new confirmation
            # sequence from this frame; never average incompatible centres.
            candidate = JunctionTracker(geometry, stamp, self.cfg,
                                        heading_hint=recovery["heading_hint"])
            recovery["candidate"] = candidate
            recovery["frames"] = 1
        self.diagnostics.update(
            relocalization_source="four_openings",
            relocalization_frames=recovery["frames"],
            candidate_centre=candidate.geometry.centre.tolist(),
            candidate_incoming_axis=float(candidate.incoming_axis))
        if recovery["frames"] >= self.cfg.confirm_frames:
            return self._finish_relocalization(stamp, tracker=candidate)
        return np.zeros(2)

    def _transition(self, state, stamp, *, clear_tracker=False):
        self.state, self.state_stamp, self.confirm = state, stamp, 0
        self.centre_retreat = False
        self.centre_retries = 0
        self.centre_limit_confirm = 0
        self.entry_phase = "JUNCTION"
        self.relocalization = None
        if clear_tracker:
            self.tracker = None
        return np.zeros(2)

    def _confirmed(self, condition):
        self.confirm = self.confirm + 1 if condition else 0
        return self.confirm >= self.cfg.confirm_frames

    def _safe(self, command, features):
        output, blocked = safe_command(command, features, self.cfg, allow_reverse=True)
        if blocked:
            return self.stop_for_obstacle(command, features)
        return output

    def stop_for_obstacle(self, command, features):
        """Stop first. Only possible body contact remains a latched failure.

        Also called by the ROS adapter on genuine returns in a partially bad
        scan. Such a scan can stop us, but cannot authorize retreat/resumption.
        """
        if self.state in self.TERMINAL:
            return np.zeros(2)
        details = self._clearance_details(command, features)
        self.diagnostics.update(details)
        guards = {hit["guard"] for hit in details["clearance_hits"]}
        if "footprint" in guards:
            return self.fail("obstacle_clearance")
        if self.obstacle is None:
            self.obstacle = dict(phase="STOP", command=tuple(map(float, command)),
                                 guards=guards, details=dict(self.diagnostics), clear_frames=0,
                                 ready_frames=0, attempted=False, reference=None,
                                 progress=0.0, max_progress=0.0, started=None,
                                 mode=("clearance" if (self.state in self.CORRIDOR_STATES
                                       or self.entry_phase == "WALL_CONFIRM")
                                       and self.tracker is None else "junction"),
                                 corridor_reference=None, end_frames=0,
                                 reason="initial_stop")
            self.confirm = self.centre_limit_confirm = 0
        elif self.obstacle["phase"] == "RETREAT":
            self.obstacle.update(phase="WAIT", reason="retreat_obstructed", attempted=True)
        return np.zeros(2)

    def _obstacle_command(self, features, stamp, *, allow_retreat):
        """Validate perception while held; only confirmed branch ends advance the route."""
        hold = self.obstacle
        if safe_command([0., 0.], features, self.cfg, allow_reverse=True)[1]:
            return self.stop_for_obstacle([0., 0.], features)

        corridor = incoming_corridor(features, self.cfg)
        localized = False
        if self.tracker is not None:
            geometry = self._geometry(features)
            if geometry is not None:
                geometry = self._track_geometry(geometry, features, stamp)
                localized = geometry is not None
                if self.state == "FAILED":
                    return np.zeros(2)
            if not localized and stamp - self.tracker.stamp > self.cfg.junction_tracking_max_gap:
                return self.fail("junction_geometry_lost")
        else:
            localized = corridor is not None

        # A junction supplies two independent wall axes. Parallel corridor
        # walls alone do NOT measure longitudinal displacement.
        measured = localized and self.tracker is not None
        yaw_change = lateral = 0.0
        if measured:
            if hold["reference"] is None:
                hold["reference"] = (self.tracker.geometry.centre.copy(), self.tracker.incoming_axis)
            centre, axis = hold["reference"]
            yaw_change = axis - self.tracker.incoming_axis
            c, s = math.cos(yaw_change), math.sin(yaw_change)
            displacement = centre - np.array([[c, -s], [s, c]]) @ self.tracker.geometry.centre
            sign = float(np.sign(hold["command"][0]))
            hold["progress"] = -sign * float(displacement[0])
            hold["max_progress"] = max(hold["max_progress"], hold["progress"])
            lateral = float(displacement[1])

        if not allow_retreat or not localized:
            # A sensor pause interrupts an escape; never restart it blindly.
            if hold["phase"] == "RETREAT":
                hold["attempted"] = True
            hold.update(phase="WAIT", clear_frames=0, ready_frames=0,
                        end_frames=0,
                        reason="sensor_pause" if not allow_retreat else "localization_unavailable")
            return np.zeros(2)

        # A permanent branch end must remain observable during an obstacle
        # stop. Confirm the same transverse-wall evidence as normal driving;
        # a lone close return must never advance endpoint/route counters.
        if self.state in ("LEFT_OUTBOUND", "RIGHT_OUTBOUND"):
            end_sign = 1 if self.state == "LEFT_OUTBOUND" else -1
            end = self._end_wall(features, corridor, end_sign)
            hold["end_frames"] = hold["end_frames"] + 1 if end else 0
            if end:
                hold.update(phase="WAIT", reason="confirming_branch_end", ready_frames=0)
                if hold["end_frames"] >= self.cfg.confirm_frames:
                    self.obstacle = None
                    self.endpoints_reached += 1
                    return self._transition("REVERSE_LEFT" if end_sign > 0 else "RETURN_RIGHT",
                                            stamp, clear_tracker=True)
                return np.zeros(2)

        clearance_mode = hold["mode"] == "clearance"
        if clearance_mode and hold["corridor_reference"] is None:
            hold["corridor_reference"] = corridor

        # Test the ORIGINAL intended stopping envelope, expanded by measured
        # retreat and hysteresis. Backing away must not itself look like an
        # obstacle removal and cause endless forward/backward oscillation.
        # Without longitudinal localization use a conservative full escape
        # allowance for release, NOT a claimed measurement of vehicle travel.
        # This prevents a fixed obstacle from clearing merely because we backed
        # away. The timeout only aborts escape; it never certifies displacement.
        release_extra = (self.OBSTACLE_RETREAT_SPEED * self.OBSTACLE_RETREAT_TIMEOUT
                         if clearance_mode and hold["attempted"] else hold["max_progress"])
        release_cfg = replace(self.cfg,
                              stop_margin=self.cfg.stop_margin + release_extra + self.OBSTACLE_RELEASE_MARGIN,
                              half_width=self.cfg.half_width + self.OBSTACLE_RELEASE_MARGIN,
                              collision_margin=self.cfg.collision_margin + self.OBSTACLE_RELEASE_MARGIN)
        clear = not safe_command(hold["command"], features, release_cfg, allow_reverse=True)[1]
        hold["clear_frames"] = hold["clear_frames"] + 1 if clear else 0
        if clear:
            hold.update(phase="WAIT", ready_frames=0, reason="confirming_clearance")
            if hold["clear_frames"] >= self.cfg.confirm_frames:
                self.obstacle = None
                self.confirm = self.centre_limit_confirm = 0
            return np.zeros(2)  # Recompute route control from the NEXT scan.

        if hold["phase"] == "RETREAT":
            if clearance_mode:
                reference_heading, reference_balance = hold["corridor_reference"]
                retreat_clear_cfg = replace(self.cfg,
                                            stop_margin=self.cfg.stop_margin + self.OBSTACLE_RELEASE_MARGIN,
                                            half_width=self.cfg.half_width + .04)
                if (abs(corridor[0]) > .15 or
                        abs(axis_angle(corridor[0] - reference_heading)) > self.cfg.yaw_tolerance or
                        abs(corridor[1] - reference_balance) / 2 > .05):
                    hold.update(phase="WAIT", reason="retreat_pose_uncertain")
                elif not safe_command(hold["command"], features, retreat_clear_cfg,
                                      allow_reverse=True)[1]:
                    hold.update(phase="WAIT", reason="retreat_clearance_restored")
            elif (not measured or abs(yaw_change) > self.cfg.yaw_tolerance or abs(lateral) > .05
                    or hold["progress"] < -.03):
                hold.update(phase="WAIT", reason="retreat_pose_uncertain")
            elif hold["progress"] >= self.OBSTACLE_RETREAT_DISTANCE:
                hold.update(phase="WAIT", reason="retreat_distance_reached")
            if stamp - hold["started"] >= self.OBSTACLE_RETREAT_TIMEOUT:
                hold.update(phase="WAIT", reason="retreat_watchdog")
            if hold["phase"] != "RETREAT":
                return np.zeros(2)

        v, w = hold["command"]
        tracking_room = (self.tracker is not None and
                         np.linalg.norm(self.tracker.geometry.centre) <=
                         self.cfg.junction_acquire_distance - self.OBSTACLE_RETREAT_DISTANCE - .05)
        motion_obstacle = ("travel" in hold["guards"] and
                           hold["guards"].issubset({"travel", "sweep"}))
        eligible = (self.state in self.RETREAT_STATES and measured and v != 0
                    and motion_obstacle and abs(w) <= .15
                    and abs(yaw_change) <= self.cfg.yaw_tolerance and abs(lateral) <= .05
                    and tracking_room
                    and min(abs(axis_angle(a)) for a in self.tracker.geometry.axes) <= self.cfg.yaw_tolerance)
        if clearance_mode:
            eligible = (v != 0 and bool(hold["guards"]) and
                        hold["guards"].issubset({"travel", "sweep"}) and
                        corridor is not None and abs(corridor[0]) <= .15)
        if hold["phase"] != "RETREAT":
            if not eligible or hold["attempted"]:
                hold.update(phase="WAIT", ready_frames=0,
                            reason=hold["reason"] if hold["attempted"] else "retreat_unobservable_or_ineligible")
                return np.zeros(2)
            hold["ready_frames"] += 1

        reverse = [-float(np.sign(v)) * self.OBSTACLE_RETREAT_SPEED, 0.]
        # Check the entire remaining straight escape sweep, including a wider
        # lateral margin and the existing reaction/braking allowance.
        remaining = max(0., self.OBSTACLE_RETREAT_DISTANCE - hold["progress"])
        retreat_cfg = replace(self.cfg, half_width=self.cfg.half_width + .05,
                              stop_margin=max(self.cfg.stop_margin, remaining + .05))
        output, blocked = safe_command(reverse, features, retreat_cfg, allow_reverse=True)
        if blocked:
            hold.update(phase="WAIT", attempted=True, reason="reverse_path_blocked")
            return np.zeros(2)
        if hold["phase"] != "RETREAT":
            if hold["ready_frames"] < self.cfg.confirm_frames:
                hold.update(phase="STOP", reason="confirming_retreat_path")
                return np.zeros(2)
            hold.update(phase="RETREAT", attempted=True, started=stamp,
                        reason="clearance_retreat" if clearance_mode else "measured_retreat")
        return output

    def _clearance_details(self, command, features):
        """Explain the existing safe_command guard without relaxing it.

        Report the ray causing each guard, not just the global nearest ray,
        which may be on an unrelated side of the vehicle.
        """
        cfg = self.cfg
        v, w = np.clip(command, [-MAX_LIN, -MAX_ANG], [MAX_LIN, MAX_ANG])
        x = features.ranges * np.cos(features.angles)
        y = features.ranges * np.sin(features.angles)
        masks, metrics = command_clearance_masks([v, w], features, cfg)
        hits = []
        for guard, mask in masks.items():
            indices = np.flatnonzero(mask)
            if not len(indices):
                continue
            index = int(indices[np.argmin(features.ranges[indices])])
            hits.append({"guard": guard, "ray_index": index,
                         "range_m": float(features.ranges[index]),
                         "angle_deg": math.degrees(float(features.angles[index])),
                         "x_m": float(x[index]), "y_m": float(y[index])})
        return {"requested_v": float(v), "requested_w": float(w), **metrics,
                "footprint_half_length_m": cfg.half_length + cfg.collision_margin,
                "footprint_half_width_m": cfg.half_width + cfg.collision_margin,
                "clearance_hits": hits}

    def _geometry(self, features):
        # Rail sampling changes abruptly at an opening edge. Tighter windows
        # retain the same four-gap validation without farther pen interiors.
        for scale in (1.0, .8, .65):
            geometry = junction_geometry(features, replace(
                self.cfg, junction_fit_radius=scale * self.cfg.junction_fit_radius))
            if geometry is not None:
                self.diagnostics["geometry_source"] = "four_openings"
                return geometry
        if self.tracker is not None and self.state not in ("MAIN", "REVERSE_LEFT", "RETURN_RIGHT"):
            geometry = self.tracker.observe_walls(features)
            if geometry is not None:
                self.diagnostics["geometry_source"] = "tracked_walls"
                return geometry
        return None

    def _acquire(self, geometry, stamp, travel_sign, next_state):
        if geometry is None:
            self.tracker, self.confirm = None, 0
            return None
        candidate = JunctionTracker(geometry, stamp, self.cfg)
        axis = candidate.incoming_axis
        direction = np.array([math.cos(axis), math.sin(axis)])
        ahead = travel_sign * float(geometry.centre @ direction)
        if abs(axis) > math.radians(20) or ahead < -self.cfg.junction_centre_tolerance:
            self.tracker, self.confirm = None, 0
            return None
        if self.tracker is None:
            self.tracker = candidate
        elif not self.tracker.update(geometry, stamp):
            self.diagnostics.update(self.tracker.update_diagnostics)
            return self.fail(self.tracker.failure)
        if self._confirmed(True):
            return self._transition(next_state, stamp)
        return np.zeros(2)

    def _track_geometry(self, geometry, features, stamp):
        """Validate a new detection before replacing the acquired crossing.

        A spurious complete rectangle must not preempt continuous wall evidence.
        Retry with current measured walls only after a spatial rejection. Time
        gaps remain failures and cannot be repaired by changing geometry source.
        """
        if self.tracker.update(geometry, stamp):
            return geometry
        reason = self.tracker.failure
        rejected = dict(self.tracker.update_diagnostics, reason=reason)
        self.diagnostics["rejected_geometry"] = rejected
        if (self.diagnostics.get("geometry_source") == "four_openings" and
                reason in ("junction_position_jump", "junction_axis_jump")):
            fallback = self.tracker.observe_walls(features)
            if fallback is not None:
                if self.tracker.update(fallback, stamp):
                    self.diagnostics["geometry_source"] = "tracked_walls_after_rejection"
                    return fallback
                self.diagnostics["rejected_fallback"] = dict(self.tracker.update_diagnostics,
                                                             reason=self.tracker.failure)
        self.fail(reason)
        return None

    def _corridor_drive(self, features, corridor, sign):
        if corridor is None:
            return np.zeros(2)
        heading, balance = corridor
        w = 1.4 * heading + sign * balance
        speed = self.cfg.cruise_speed
        distance = features.front_wall_m if sign > 0 else features.rear_wall_m
        if self.state in ("LEFT_OUTBOUND", "RIGHT_OUTBOUND", "MAIN", "FINAL_MAIN") and distance < .85:
            speed = min(speed, .07)
        # Preserve curvature when slowing near a potential end wall.
        w = np.clip(w, -.3, .3) * speed / self.cfg.cruise_speed
        return self._safe([sign * speed, w], features)

    def _end_wall(self, features, corridor, sign):
        if corridor is None or abs(corridor[0]) > .15:
            return False
        distance = features.front_wall_m if sign > 0 else features.rear_wall_m
        if distance > .70:
            return False
        # Require a fitted transverse wall, not a lone near return.
        for line in wall_lines(features.ranges, features.angles, 1.3):
            if abs(abs(line.angle) - math.pi / 2) > .12 or line.span < .45:
                continue
            if abs(line.normal[0]) < .9:
                continue
            forward_distance = sign * line.offset / line.normal[0]
            if 0 < forward_distance <= .65:
                return True
        return False

    def _entry_wall_ready(self, features, corridor):
        # incoming_corridor already requires a fitted pair of opposite walls
        # inside wall_distance.  The fixed +/-90 degree edge rays used by
        # walls_present() can still look through a junction opening even when
        # that fitted corridor is excellent; requiring both observations here
        # can therefore stop WALL_CONFIRM forever at the crossing boundary.
        return (corridor is not None
                and abs(corridor[0]) <= self.cfg.entry_handoff_yaw_tolerance
                and abs(corridor[1] / 2) <= self.cfg.entry_handoff_wall_offset)

    def _entry_wall_command(self, features, stamp):
        """Latched wall acquisition: never reacquire the distant junction.

        Entry depth was measured before this phase. Wall loss interrupts
        confirmation and stops motion; the existing state timeout bounds it.
        """
        corridor = incoming_corridor(features, self.cfg)
        ready = self._entry_wall_ready(features, corridor)
        self.diagnostics.update(
            entry_phase=self.entry_phase, geometry_source="corridor_walls",
            entry_wall_weight=1.0, entry_wall_confirmed=bool(ready),
            entry_wall_heading_error=None if corridor is None else float(corridor[0]),
            entry_wall_offset_m=None if corridor is None else float(corridor[1] / 2))
        confirmed = self._confirmed(ready)
        self.diagnostics["entry_confirmation_frames"] = self.confirm
        if not ready:
            return self._safe([0., 0.], features)
        sign = -1 if self.state == "CROSS_REVERSE" else 1
        # Low-speed curved correction, including during confirmation.
        speed = min(.07, self.cfg.cruise_speed)
        w = np.clip(1.4 * corridor[0] + sign * corridor[1], -.3, .3)
        command = self._safe([sign * speed, w * speed / self.cfg.cruise_speed], features)
        if self.state in self.TERMINAL or self.obstacle is not None:
            return command
        if confirmed:
            target = {"ENTRY_LEFT": "LEFT_OUTBOUND", "CROSS_REVERSE": "RIGHT_OUTBOUND",
                      "EXIT_MAIN": "MAIN"}[self.state]
            if self.state == "EXIT_MAIN":
                self.junctions_done += 1
                if self.junctions_done >= self.cfg.junctions:
                    target = "FINAL_MAIN"
            return self._transition(target, stamp, clear_tracker=True)
        return command

    def command(self, features, stamp, *, allow_retreat=True):
        if self.state in self.TERMINAL:
            return np.zeros(2)
        if not math.isfinite(stamp):
            return self.fail("invalid_scan_stamp")
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                return self.fail("scan_time_reversed")
            if stamp == self.last_stamp:
                self.confirm = 0
                if self.obstacle is not None:
                    self.obstacle.update(phase="WAIT", clear_frames=0, ready_frames=0,
                                         end_frames=0, reason="repeated_scan_stamp")
                return np.zeros(2)
            if stamp - self.last_stamp > self.cfg.junction_tracking_max_gap:
                return self.fail("scan_gap")
        previous_stamp = self.last_stamp
        self.last_stamp = stamp
        if self.state_stamp is None:
            self.state_stamp = stamp
        if self.obstacle is not None:
            # Waiting for a real obstacle is not a failed route manoeuvre.
            if previous_stamp is not None:
                self.state_stamp += stamp - previous_stamp
            return self._obstacle_command(features, stamp, allow_retreat=allow_retreat)
        if self.relocalization is not None:
            if previous_stamp is not None:
                self.state_stamp += stamp - previous_stamp
            return self._relocalization_command(features, stamp)
        if stamp - self.state_stamp > self.cfg.state_timeout:
            return self.fail("state_timeout")

        self.diagnostics = {}
        # Completing all configured crossings does not mean we have reached
        # the main-road end. Continue straight in a state that cannot acquire
        # another crossing, and independently confirm the terminal wall.
        if self.state == "MAIN" and self.junctions_done >= self.cfg.junctions:
            return self._transition("FINAL_MAIN", stamp, clear_tracker=True)
        corridor_states = ("MAIN", "FINAL_MAIN", "LEFT_OUTBOUND", "REVERSE_LEFT",
                           "RIGHT_OUTBOUND", "RETURN_RIGHT")
        if self.state in corridor_states:
            sign = -1 if self.state in ("REVERSE_LEFT", "RIGHT_OUTBOUND") else 1
            corridor = incoming_corridor(features, self.cfg)
            if self.state in ("MAIN", "FINAL_MAIN", "LEFT_OUTBOUND", "RIGHT_OUTBOUND"):
                if self._end_wall(features, corridor, sign):
                    if self._confirmed(True):
                        if self.state in ("MAIN", "FINAL_MAIN"):
                            if (self.junctions_done < self.cfg.junctions or
                                    self.endpoints_reached < 2 * self.cfg.junctions):
                                self.diagnostics.update(junctions_done=self.junctions_done,
                                                        endpoints_reached=self.endpoints_reached,
                                                        expected_junctions=self.cfg.junctions)
                                return self.fail("main_end_before_patrol_complete")
                            return self._transition("DONE", stamp, clear_tracker=True)
                        self.endpoints_reached += 1
                        next_state = ("REVERSE_LEFT" if self.state == "LEFT_OUTBOUND"
                                      else "RETURN_RIGHT")
                        return self._transition(next_state, stamp, clear_tracker=True)
                    return np.zeros(2)
                self.confirm = 0 if self.tracker is None else self.confirm
            if self.state in ("MAIN", "REVERSE_LEFT", "RETURN_RIGHT"):
                target = {"MAIN": "CENTER", "REVERSE_LEFT": "CROSS_REVERSE",
                          "RETURN_RIGHT": "RETURN_CENTER"}[self.state]
                acquired = self._acquire(self._geometry(features), stamp, sign, target)
                if acquired is not None:
                    return acquired
            return self._corridor_drive(features, corridor, sign)

        if self.state in self.ENTRY_STATES and self.entry_phase == "WALL_CONFIRM":
            return self._entry_wall_command(features, stamp)

        geometry = self._geometry(features)
        if geometry is None:
            self.confirm = 0
            self.centre_limit_confirm = 0
            self.diagnostics.update(
                geometry_source="unavailable",
                geometry_missing_seconds=stamp - self.tracker.stamp,
                last_centre_x=float(self.tracker.geometry.centre[0]),
                last_centre_y=float(self.tracker.geometry.centre[1]),
                last_incoming_axis=float(self.tracker.incoming_axis))
            if stamp - self.tracker.stamp > self.cfg.junction_tracking_max_gap:
                return self.fail("junction_geometry_lost")
            return np.zeros(2)
        geometry = self._track_geometry(geometry, features, stamp)
        if geometry is None:
            return np.zeros(2)
        incoming, centre = self.tracker.incoming_axis, geometry.centre
        self.diagnostics.update(centre_x=float(centre[0]), centre_y=float(centre[1]))

        if self.state in ("CENTER", "RETURN_CENTER"):
            direction = np.array([math.cos(incoming), math.sin(incoming)])
            normal = np.array([-direction[1], direction[0]])
            along, lateral = float(centre @ direction), float(centre @ normal)
            # A small overshoot is recoverable: wheel inertia and LiDAR noise
            # do not stop exactly on the estimated centre. Retreat only within
            # the existing local turn-centre envelope, with rear clearance.
            recovery_limit = self.cfg.junction_turn_centre_limit
            distance = float(np.linalg.norm(centre))
            self.diagnostics.update(centre_along_m=along, centre_lateral_m=lateral,
                                    centre_recovery_limit_m=recovery_limit,
                                    centre_distance_m=distance,
                                    centre_target_tolerance_m=self.cfg.junction_centre_tolerance,
                                    centre_retries=self.centre_retries,
                                    heading_error=float(incoming))
            if along < -recovery_limit:
                return self.fail("centre_overshoot")
            tolerance = self.cfg.junction_centre_tolerance
            in_turn_region = distance <= tolerance
            # A differential drive cannot remove a purely lateral error by
            # rotating in place. Make longitudinal room, then re-approach.
            # Both phases use the CURRENT measured crossing, never elapsed
            # time or integrated commands. Hysteresis prevents sign chatter.
            if (not self.centre_retreat and not in_turn_region and abs(along) < .04 and
                    abs(lateral) > tolerance and distance > tolerance):
                if abs(lateral) > recovery_limit:
                    self.diagnostics["centre_limit_reason"] = "lateral_out_of_bounds"
                    return self.fail("centre_reposition_limit")
                if self.centre_retries >= 2:
                    # Do not fail on one noisy boundary sample, nor allow
                    # motion outside the turn-entry region after exhausted tries.
                    self.confirm = 0
                    self.centre_limit_confirm += 1
                    self.diagnostics.update(centre_phase="limit_confirmation",
                                            centre_limit_confirm=self.centre_limit_confirm,
                                            centre_limit_reason="retries_exhausted")
                    if self.centre_limit_confirm >= self.cfg.confirm_frames:
                        return self.fail("centre_reposition_limit")
                    return self._safe([0.0, 0.0], features)
                self.centre_retreat = True
                self.centre_retries += 1
                self.confirm = 0
            self.centre_limit_confirm = 0
            if self.centre_retreat:
                self.diagnostics.update(centre_phase="retreat", centre_retries=self.centre_retries)
                if along >= .35:
                    self.centre_retreat = False
                    return np.zeros(2)
                # Keep the original road heading while backing away. Rear
                # travel and rotation clearance are still checked by _safe.
                v = -.05 if abs(incoming) <= .15 else 0.0
                w = np.clip(self.cfg.turn_kp * incoming, -.3, .3)
                return self._safe([v, w], features)
            self.diagnostics.update(centre_phase="turn_ready" if in_turn_region else "approach",
                                    centre_retries=self.centre_retries)
            if self._confirmed(in_turn_region and abs(incoming) <= self.cfg.yaw_tolerance):
                # Check rotation space BEFORE accepting the turn-entry pose.
                # This is only a guard probe; no turn command is emitted here.
                self.diagnostics["clearance_check"] = "before_turn"
                self._safe([0.0, self.cfg.turn_speed], features)
                if self.state == "FAILED" or self.obstacle is not None:
                    return np.zeros(2)
                return self._transition("TURN_LEFT" if self.state == "CENTER" else "TURN_MAIN", stamp)
            sign = -1 if along < 0 else 1
            heading = (incoming if in_turn_region else incoming + sign * np.clip(
                math.atan2(lateral, max(abs(along), .08)), -.55, .55))
            # Slow down before the centre; reverse corrections are capped at
            # 5 cm/s. Reverse lateral steering has the opposite sign.
            speed_limit = .05 if sign < 0 else (.08 if along < .30 else .12)
            v = (0.0 if in_turn_region else sign * min(
                speed_limit, self.cfg.cruise_speed, .50 * abs(along)))
            if abs(wrap_angle(heading)) > .3:
                v = 0.0
            w = np.clip(self.cfg.turn_kp * wrap_angle(heading), -.3, .3)
            self.diagnostics["centre_reversing"] = v < 0
            return self._safe([v, w], features)

        turn_sign = -1 if self.state in ("TURN_MAIN", "EXIT_MAIN") else 1
        heading = (wrap_angle(incoming) if self.state == "CROSS_REVERSE"
                   else wrap_angle(incoming + turn_sign * math.pi / 2))
        self.diagnostics["heading_error"] = heading
        if self.state in ("TURN_LEFT", "TURN_MAIN"):
            if np.linalg.norm(centre) > self.cfg.junction_turn_centre_limit:
                return self.fail("turn_centre_drift")
            aligned = abs(heading) <= self.cfg.yaw_tolerance
            if self._confirmed(aligned):
                return self._transition("ENTRY_LEFT" if self.state == "TURN_LEFT" else "EXIT_MAIN", stamp)
            w = 0.0 if aligned else np.clip(self.cfg.turn_kp * heading,
                                           -self.cfg.turn_speed, self.cfg.turn_speed)
            return self._safe([0.0, w], features)

        sign = -1 if self.state == "CROSS_REVERSE" else 1
        direction = np.array([math.cos(heading), math.sin(heading)])
        normal = np.array([-direction[1], direction[0]])
        depth = -sign * float(centre @ direction)
        lateral = float(centre @ normal)
        corridor = incoming_corridor(features, self.cfg)
        wall_precise = (corridor is not None and abs(corridor[0]) <= self.cfg.yaw_tolerance
                        and abs(corridor[1] / 2) <= self.cfg.junction_centre_tolerance)
        # Once paired branch walls are stable, corridor following is a better
        # reference than the increasingly distant intersection centre. The
        # handoff envelope is deliberately wider than the 5 cm control target;
        # normal straight driving continues reducing the residual afterwards.
        wall_handoff = (corridor is not None
                        and abs(corridor[0]) <= self.cfg.entry_handoff_yaw_tolerance
                        and abs(corridor[1] / 2) <= self.cfg.entry_handoff_wall_offset)
        self.diagnostics.update(entry_depth=depth, entry_lateral_m=lateral,
                                entry_wall_heading_error=None if corridor is None else float(corridor[0]),
                                entry_wall_offset_m=None if corridor is None else float(corridor[1] / 2),
                                entry_wall_precise=bool(wall_precise),
                                entry_wall_confirmed=bool(wall_handoff),
                                entry_handoff_yaw_limit=self.cfg.entry_handoff_yaw_tolerance,
                                entry_handoff_offset_limit_m=self.cfg.entry_handoff_wall_offset)
        entered = depth >= self.cfg.entry_distance and self._entry_wall_ready(features, corridor)
        if entered:
            # Latch the measured depth once; remaining confirmation needs only
            # current walls, even if the junction disappears on the next scan.
            self.entry_phase = "WALL_CONFIRM"
            self.tracker = None
            self.confirm = 0
            return self._entry_wall_command(features, stamp)
        if depth > self.cfg.junction_acquire_distance - .15 and not entered:
            return self.fail("entry_not_confirmed")
        # Blend only live measurements over the latter half of entry distance.
        # No time integration or stale junction steering survives the handoff.
        weight = (float(np.clip(2 * depth / self.cfg.entry_distance - 1, 0., 1.))
                  if corridor is not None else 0.)
        self.entry_phase = "BLEND" if weight > 0 else "JUNCTION"
        junction_w = np.clip(self.cfg.turn_kp * heading + sign * lateral, -.3, .3)
        wall_w = (np.clip(1.4 * corridor[0] + sign * corridor[1], -.3, .3)
                  if corridor is not None else 0.)
        w = (1 - weight) * junction_w + weight * wall_w
        self.diagnostics.update(entry_phase=self.entry_phase, entry_wall_weight=weight)
        entry_speed = .12 if wall_precise or corridor is None else .07
        v = sign * min(entry_speed, self.cfg.cruise_speed)
        w *= abs(v) / self.cfg.cruise_speed
        return self._safe([v, w], features)
