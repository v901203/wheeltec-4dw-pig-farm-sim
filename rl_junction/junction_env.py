"""Hybrid junction skills with causal phase/geometry context and recovery disturbances."""
import math
from dataclasses import replace
import gymnasium as gym
import numpy as np
from ros_gz_interfaces.srv import SetEntityPose
from gazebo_env import JunctionEnv as RouteEnv
from fsm_teacher import FSMTeacher, HANDOFF_STATES, SKILL_STATES
from tasks import angle_error, collision
from learning_common import encode_command, BASE_OBS_SIZE
from policy_context import decision_context, phase_id, RAW_OBS_SIZE
from recovery import recovery_offset
from navigation import safe_command


class HybridJunctionEnv(RouteEnv):
    LOCALIZATION_HOLD_STEPS = 50  # 10 simulated seconds stopped, not a travel estimate.
    MAX_LOCALIZATION_RECOVERIES = 3
    def __init__(self, task=None, max_steps=1200, available_tasks=None,
                 recovery_case=None, recovery_phase="center"):
        allowed = list(HANDOFF_STATES) if available_tasks is None else available_tasks
        if not set(allowed).issubset(HANDOFF_STATES):
            raise ValueError("Unknown junction skill")
        if recovery_case not in (None, "mixed", "lateral", "overshoot", "yaw"):
            raise ValueError("Unknown recovery case")
        if recovery_phase not in ("center", "turn", "entry"):
            raise ValueError("Unknown recovery phase")
        self.skill_max_steps = max_steps
        self.recovery_case = recovery_case
        self.recovery_phase = recovery_phase
        self.in_skill = False
        super().__init__(task=task, max_steps=2400, available_tasks=allowed)
        self.observation_space = gym.spaces.Box(-1., 1., (BASE_OBS_SIZE,), dtype=np.float32)

    def _prepare_decision(self):
        if self.cached_stamp != self.scan_stamp:
            features = self._features()
            self.cached_command = self.teacher.command(features, self.scan_stamp)
            self.cached_stamp = self.scan_stamp
            self.cached_context = decision_context(self.teacher.controller, features, self.scan_stamp)
            self._cache_label_validity(features)

    def _cache_label_validity(self, features):
        c = self.teacher.controller
        ready = (c.state in SKILL_STATES[self.task.name] and not c.failure and
                 c.obstacle is None and c.relocalization is None and not c.external_paused)
        # Centre/turn labels need both a current centre and a current heading.
        context_valid = bool(self.cached_context[8] and
                             (self.cached_context[7] or c.state in c.ENTRY_STATES or
                              c.state in c.JUNCTION_APPROACH_STATES))
        _, blocked = safe_command(self.cached_command, features, self.cfg,
            measured_speed=abs(self.odom.twist.twist.linear.x), allow_reverse=True)
        self.label_valid = bool(ready and context_valid and not blocked)

    def _policy_observation(self, raw=None):
        self._prepare_decision()
        base = RouteEnv._observation(self) if raw is None else raw
        return np.concatenate((base, self.cached_context)).astype(np.float32)

    def expert_command(self):
        self._prepare_decision()
        return self.cached_command.copy()

    def reset(self, *, seed=None, options=None):
        self.in_skill = False
        self.perturbation_applied = False
        self.perturbations = []
        self.last_step_perturbation = None
        _, info = super().reset(seed=seed, options=options)
        cases = ["lateral", "yaw"] if self.task.name == "left_to_right_reverse" else ["lateral", "yaw", "overshoot"]
        self.episode_recovery_case = (str(self.np_random.choice(cases)) if self.recovery_case == "mixed"
                                      else self.recovery_case)
        self.teacher = FSMTeacher(self.task, self.cfg)
        if self.recovery_case:
            # The stock teacher starts turning at 5 cm. After an overshoot its
            # rotation can consume the remaining 15 cm tracking envelope.
            # Recovery demonstrations re-centre more accurately before turning;
            # the deployed patrol configuration and normal teacher are unchanged.
            self.teacher.cfg = replace(self.teacher.cfg, junction_centre_tolerance=.025)
            self.teacher.controller.cfg = self.teacher.cfg
        self.cached_stamp = None
        warmup_hold = 0
        for warmup in range(1200):
            self._prepare_decision()
            status = self.teacher.status()
            warmup_hold = warmup_hold+1 if status["relocalizing"] else 0
            if status["failure"] or status["obstacle"] or warmup_hold > self.LOCALIZATION_HOLD_STEPS:
                self._publish((0., 0.))
                self._drain()
                raise RuntimeError(f"FSM cruise failed before policy takeover: {status}")
            acquired = (self.teacher.controller.tracker is not None and
                        status["state"] in SKILL_STATES[self.task.name])
            if acquired:
                self.skill_start = self.pose.copy()
                self.steps = self.goal_frames = self.idle_steps = 0
                self.active = self.in_skill = True
                self.skill_steps = 0
                self.localization_hold_steps = 0
                self.localization_recoveries = 0
                self.was_relocalizing = False
                self.start_route_state = status["state"]
                print(f"Policy takeover: {status['state']} (junction acquired), FSM cruise_steps={warmup+1}", flush=True)
                return self._policy_observation(), dict(info, route_state=status["state"],
                    warmup_steps=warmup+1, policy_scope="JUNCTION_APPROACH_TO_HANDOFF")
            _, _, done, truncated, _ = super().step(encode_command(*self.cached_command, self.task.name))
            if done or truncated:
                raise RuntimeError("FSM cruise did not acquire the intended junction")
        raise RuntimeError("FSM cruise warmup exceeded 1200 steps")

    def _before_advance(self):
        self.last_step_perturbation = None
        if not self.in_skill or not self.recovery_case or self.perturbation_applied:
            return
        phase = self.recovery_phase
        if self.task.name == "left_to_right_reverse" and phase == "center":
            phase = "entry"  # Straight crossing has no CENTER/TURN state.
        proposal = recovery_offset(self.teacher.controller, self.np_random, self.episode_recovery_case, phase)
        if proposal is None:
            return
        case, offset = proposal
        x, y, yaw = self.pose
        dx, dy, da = offset
        candidate = np.array([x+math.cos(yaw)*dx-math.sin(yaw)*dy,
                              y+math.sin(yaw)*dx+math.cos(yaw)*dy, yaw+da])
        if collision(candidate, self.cfg.half_length+self.cfg.collision_margin,
                     self.cfg.half_width+self.cfg.collision_margin):
            return
        request = SetEntityPose.Request()
        request.entity.name = "wheeltec_mini"
        request.entity.type = request.entity.MODEL
        request.pose.position.x, request.pose.position.y = map(float, candidate[:2])
        request.pose.position.z = self.pose_z
        request.pose.orientation.z = math.sin(candidate[2]/2)
        request.pose.orientation.w = math.cos(candidate[2]/2)
        self._service(self.pose_client, request)
        self.perturbation_applied = True
        self.last_step_perturbation = {"case": case, "body_offset": offset.tolist(),
            "phase": phase, "step": self.skill_steps+1, "scan_stamp": self.scan_stamp}
        self.perturbations.append(self.last_step_perturbation)
        print(f"RECOVERY DISTURBANCE: {self.last_step_perturbation}", flush=True)

    def step(self, action):
        if not self.active:
            raise RuntimeError("Call reset before step")
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (2,) or not np.isfinite(action).all():
            raise ValueError("Expected two finite actions")
        self._prepare_decision()
        current = self.teacher.status()
        current_handoff = current["state"] == HANDOFF_STATES[self.task.name]
        current_fault = bool(current["failure"] or current["obstacle"])
        hold = not self.label_valid and not current_handoff
        execute = np.clip(action, -1, 1).copy()
        if current_handoff or current_fault or current["relocalizing"] or hold:
            execute[:] = [-1., 0.] if self.task.name == "left_to_right_reverse" else [0., 0.]
        before_yaw = abs(angle_error(self.task.goal[2], self.pose[2]))
        self.idle_steps = 0
        raw, base_reward, _, _, info = super().step(execute)
        self.skill_steps += 1
        # Process the NEW scan once before making the next observation.
        # Collection and inference see the same decision-time phase and geometry.
        self._prepare_decision()
        status = self.teacher.status()
        handoff = status["state"] == HANDOFF_STATES[self.task.name]
        localizing = bool(status["relocalizing"])
        self.localization_hold_steps = self.localization_hold_steps+1 if localizing else 0
        if localizing and not self.was_relocalizing:
            self.localization_recoveries += 1
        recovered = self.was_relocalizing and not localizing and not status["failure"]
        self.was_relocalizing = localizing
        recovery_failed = (self.localization_hold_steps > self.LOCALIZATION_HOLD_STEPS or
                           self.localization_recoveries > self.MAX_LOCALIZATION_RECOVERIES)
        fault = bool(status["failure"] or status["obstacle"] or recovery_failed)
        after_yaw = abs(angle_error(self.task.goal[2], self.pose[2]))
        success = bool(handoff and not fault and not localizing and not info["collision"] and
                       not info["off_route"] and after_yaw < math.radians(10))
        failed = bool(fault or info["collision"] or info["off_route"] or
                      info["wrong_heading"] or (handoff and not success))
        terminated = success or failed
        truncated = self.skill_steps >= self.skill_max_steps and not terminated
        if info["success"]:
            base_reward -= 20
        reward = base_reward + .3*(before_yaw-after_yaw) + (20 if success else -20 if fault else 0)
        recovery = self.teacher.controller.relocalization
        info.update(success=success, no_progress=False, teacher_failure=status["failure"],
                    route_state=status["state"], heading_error_rad=after_yaw,
                    sim_seconds=self.skill_steps*self.dt, teacher_intervened=fault or hold or localizing,
                    perception_hold=hold, recovery_reason=None if recovery is None else recovery["reason"],
                    localization_hold=localizing, localization_recovered=recovered,
                    localization_recoveries=self.localization_recoveries,
                    localization_recovery_failed=recovery_failed,
                    phase_id=phase_id(np.concatenate((raw, self.cached_context))),
                    perturbation=self.last_step_perturbation, policy_scope="JUNCTION_APPROACH_TO_HANDOFF")
        obs = self._policy_observation(raw)
        self.active = not (terminated or truncated)
        if not self.active:
            self._publish((0., 0.))
            self._drain()
        return obs, float(reward), bool(terminated), bool(truncated), info
