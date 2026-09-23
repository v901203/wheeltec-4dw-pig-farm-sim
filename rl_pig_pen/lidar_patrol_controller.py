"""LiDAR-only traditional patrol. No ROS, odometry or timed manoeuvres."""

import math
from dataclasses import replace

import numpy as np

from junction_localization import JunctionTracker, junction_geometry
from lidar_geometry import axis_angle, wall_lines
from navigation import MAX_ANG, MAX_LIN, MIN_WALL_CONFIDENCE, safe_command, wrap_angle


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

    def __init__(self, cfg):
        if cfg.entry_distance >= cfg.junction_acquire_distance - .15:
            raise ValueError("entry_distance must remain inside the junction tracking range")
        self.cfg = replace(cfg, junction_centre_tolerance=min(.02, cfg.junction_centre_tolerance))
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

    def fail(self, reason):
        if self.state not in self.TERMINAL:
            self.failure_diagnostics = dict(self.diagnostics, failed_state=self.state)
            self.failure = reason
            self.state = "FAILED"
        return np.zeros(2)

    def _transition(self, state, stamp, *, clear_tracker=False):
        self.state, self.state_stamp, self.confirm = state, stamp, 0
        if clear_tracker:
            self.tracker = None
        return np.zeros(2)

    def _confirmed(self, condition):
        self.confirm = self.confirm + 1 if condition else 0
        return self.confirm >= self.cfg.confirm_frames

    def _safe(self, command, features):
        output, blocked = safe_command(command, features, self.cfg, allow_reverse=True)
        if blocked:
            self.diagnostics.update(self._clearance_details(command, features))
            return self.fail("obstacle_clearance")
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
        braking = abs(v) * cfg.control_dt + v ** 2 / (2 * cfg.brake_deceleration)
        travel_limit = cfg.half_length + cfg.stop_margin + braking
        rotation_limit = math.hypot(cfg.half_length, cfg.half_width) + cfg.collision_margin
        along = x if v >= 0 else -x
        masks = {
            "footprint": ((np.abs(x) < cfg.half_length + cfg.collision_margin) &
                          (np.abs(y) < cfg.half_width + cfg.collision_margin)),
            "travel": ((abs(v) > 0) & (along > 0) & (along < travel_limit) &
                       (np.abs(y) < cfg.half_width + .03)),
            "rotation": (abs(w) > 0) & (features.ranges < rotation_limit),
        }
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
        return {"requested_v": float(v), "requested_w": float(w),
                "travel_limit_m": float(travel_limit), "rotation_limit_m": rotation_limit,
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
        return self._safe([sign * self.cfg.cruise_speed, np.clip(w, -.3, .3)], features)

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

    def command(self, features, stamp):
        if self.state in self.TERMINAL:
            return np.zeros(2)
        if not math.isfinite(stamp):
            return self.fail("invalid_scan_stamp")
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                return self.fail("scan_time_reversed")
            if stamp == self.last_stamp:
                return np.zeros(2)
            if stamp - self.last_stamp > self.cfg.junction_tracking_max_gap:
                return self.fail("scan_gap")
        self.last_stamp = stamp
        if self.state_stamp is None:
            self.state_stamp = stamp
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

        geometry = self._geometry(features)
        if geometry is None:
            self.confirm = 0
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
            self.diagnostics.update(centre_along_m=along, centre_lateral_m=lateral,
                                    centre_recovery_limit_m=recovery_limit)
            if along < -recovery_limit:
                return self.fail("centre_overshoot")
            at_centre = np.linalg.norm(centre) <= self.cfg.junction_centre_tolerance
            if self._confirmed(at_centre and abs(incoming) <= self.cfg.yaw_tolerance):
                return self._transition("TURN_LEFT" if self.state == "CENTER" else "TURN_MAIN", stamp)
            sign = -1 if along < 0 else 1
            heading = (incoming if at_centre else incoming +
                       sign * math.atan2(lateral, max(abs(along), .25)))
            # Slow down before the centre; reverse corrections are capped at
            # 5 cm/s. Reverse lateral steering has the opposite sign.
            speed_limit = .05 if sign < 0 else (.08 if along < .30 else .12)
            v = (0.0 if at_centre else sign * min(
                speed_limit, self.cfg.cruise_speed, .50 * abs(along)))
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
        self.diagnostics["entry_depth"] = depth
        entered = (depth >= self.cfg.entry_distance and features.walls_present(self.cfg)
                   and abs(heading) <= self.cfg.yaw_tolerance
                   and abs(lateral) <= .05)
        if self._confirmed(entered):
            target = {"ENTRY_LEFT": "LEFT_OUTBOUND", "CROSS_REVERSE": "RIGHT_OUTBOUND",
                      "EXIT_MAIN": "MAIN"}[self.state]
            if self.state == "EXIT_MAIN":
                self.junctions_done += 1
                if self.junctions_done >= self.cfg.junctions:
                    target = "FINAL_MAIN"
            return self._transition(target, stamp, clear_tracker=True)
        if depth > self.cfg.junction_acquire_distance - .15:
            return self.fail("entry_not_confirmed")
        w = np.clip(self.cfg.turn_kp * heading + sign * lateral, -.3, .3)
        v = 0.0 if entered else sign * min(.12, self.cfg.cruise_speed)
        return self._safe([v, w], features)
