"""Model-owned junction traversal: no internal FSM transitions after takeover."""
import math
import numpy as np
from junction_env import HybridJunctionEnv
from gazebo_env import JunctionEnv as RouteEnv
from junction_localization import junction_geometry
from lidar_patrol_controller import incoming_corridor
from navigation import wrap_angle
from tasks import CENTRE_Y


def exit_measurement(centre, target_heading):
    axis = np.array([math.cos(target_heading), math.sin(target_heading)])
    return float(-np.asarray(centre) @ axis), float(np.asarray(centre) @ [-axis[1], axis[0]])


def completed_exit(depth, lateral, heading, corridor, frames):
    valid = (depth >= .9 and abs(lateral) <= .12 and abs(heading) <= math.radians(10)
             and corridor is not None and abs(corridor[0]) <= math.radians(10)
             and abs(corridor[1]/2) <= .12)
    return frames+1 if valid else 0


class ContinuousJunctionEnv(HybridJunctionEnv):
    """FSM only acquires the incoming junction before policy takeover.

    Thereafter independent LiDAR tracking supplies context, safe_command guards
    the body, and measured exit depth/corridor alignment decides completion.
    World pose is used only for reward/collision scoring, never actor context.
    """
    def __init__(self, *args, **kwargs):
        self.continuous = False
        super().__init__(*args, **kwargs)

    def _prepare_decision(self):
        if not self.continuous:
            return super()._prepare_decision()
        if self.cached_stamp == self.scan_stamp:
            return
        features = self._features()
        geometry = junction_geometry(features, self.cfg)
        if geometry is None:
            geometry = self.continuous_tracker.observe_walls(features)
        valid = bool(self.scan_stamp == self.continuous_tracker.stamp or
                     (geometry is not None and self.continuous_tracker.update(geometry, self.scan_stamp)))
        self.geometry_valid = valid
        self.cached_context = np.zeros(10, dtype=np.float32)
        # First four legacy phase slots remain zero: no control phase gate.
        if valid:
            target = wrap_angle(self.continuous_tracker.incoming_axis+self.exit_turn)
            self.cached_context[4:6] = np.clip(self.continuous_tracker.geometry.centre/2., -1., 1.)
            self.cached_context[6] = target/math.pi
            self.cached_context[7:9] = 1.
            self.exit_depth, self.exit_lateral = exit_measurement(self.continuous_tracker.geometry.centre,
                target+(math.pi if self.task.name == "left_to_right_reverse" else 0.))
            self.exit_heading = target
        self.cached_context[9] = float(self.task.name == 'left_to_right_reverse')
        self.cached_stamp = self.scan_stamp

    def reset(self, **kwargs):
        self.continuous = False
        super().reset(**kwargs)
        # The pre-entry teacher's acquired junction initializes geometry only.
        self.continuous_tracker = self.teacher.controller.tracker
        self.exit_turn = {'main_to_left':math.pi/2, 'right_to_main':-math.pi/2,
                          'left_to_right_reverse':0.}[self.task.name]
        self.continuous = True
        self.cached_stamp = None
        self.exit_depth, self.exit_lateral, self.exit_heading = 0., 0., 0.
        self.exit_frames = self.geometry_hold = self.continuous_stall = 0
        self.skill_steps = 0
        self._prepare_decision()
        self.best_route_error = self.route_error(self.pose)
        return self._policy_observation(), dict(task=self.task.name, policy_scope='CONTINUOUS_JUNCTION')

    def route_error(self, pose):
        # Smooth centre-to-exit polyline approximation. No stop or phase switch:
        # rewards a collision-free continuous bend instead of an exact centre.
        centre = np.array([0., CENTRE_Y])
        start_axis = np.array([math.cos(self.task.start[2]), math.sin(self.task.start[2])])
        if self.task.name == 'left_to_right_reverse':
            return float(np.linalg.norm(pose[:2]-self.task.goal[:2]))
        turn_axis = np.array([math.cos(self.task.goal[2]), math.sin(self.task.goal[2])])
        incoming_remaining = max(0., -float((pose[:2]-centre) @ start_axis))
        outgoing_progress = max(0., float((pose[:2]-centre) @ turn_axis))
        return incoming_remaining+max(0., 2.-outgoing_progress)

    def step(self, action):
        self._prepare_decision()
        valid_before = self.geometry_valid
        before = self.route_error(self.pose)
        previous = self.previous.copy()
        execute = np.asarray(action, dtype=np.float32).copy()
        if execute.shape != (2,) or not np.isfinite(execute).all():
            raise ValueError('Expected two finite actions')
        if not valid_before:
            execute[:] = [-1., 0.] if self.task.name == 'left_to_right_reverse' else [0., 0.]
        self.idle_steps = 0
        raw_obs, _, _, _, info = RouteEnv.step(self, execute)
        self.skill_steps += 1
        self._prepare_decision()  # No teacher.command, CENTER or TURN transitions.
        self.geometry_hold = 0 if self.geometry_valid else self.geometry_hold+1
        corridor = incoming_corridor(self._features(), self.cfg)
        self.exit_frames = (completed_exit(self.exit_depth, self.exit_lateral, self.exit_heading,
                                          corridor, self.exit_frames) if self.geometry_valid else 0)
        failed = bool(info['collision'] or info['off_route'] or info['wrong_heading'] or
                      self.geometry_hold > self.LOCALIZATION_HOLD_STEPS)
        success = bool(self.exit_frames >= 3 and not failed)
        after = self.route_error(self.pose)
        eligible = valid_before and self.geometry_valid and not info['safety_blocked']
        if eligible:
            if after < self.best_route_error-.01:
                self.best_route_error, self.continuous_stall = after, 0
            else:
                self.continuous_stall += 1
        stalled = self.continuous_stall >= 150 and not success
        failed |= stalled
        done = success or failed
        truncated = self.skill_steps >= self.skill_max_steps and not done
        progress = 8.*float(np.clip(before-after, -.08, .08)) if eligible else 0.
        smooth = -.02*float(np.sum((self.previous-previous)**2))
        reward = progress-.02+smooth-.1*info['safety_blocked']+(20 if success else -20 if failed else 0)
        self.active = not (done or truncated)
        if not self.active:
            self._publish((0., 0.))
            self._drain()
        info.update(success=success, no_progress=stalled, teacher_intervened=not valid_before,
                    perception_hold=not valid_before, phase_id=-1,
                    route_state='HANDOFF' if success else 'CONTINUOUS_JUNCTION',
                    sim_seconds=self.skill_steps*self.dt, exit_depth_m=self.exit_depth,
                    exit_lateral_m=self.exit_lateral, exit_heading_error=self.exit_heading,
                    geometry_valid=self.geometry_valid,
                    reward_components=dict(progress=progress, time=-.02, smooth=smooth),
                    policy_scope='CONTINUOUS_JUNCTION')
        return np.concatenate((raw_obs, self.cached_context)), float(reward), bool(done), bool(truncated), info
