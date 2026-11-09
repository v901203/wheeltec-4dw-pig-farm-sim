"""Training-only phase rewards and frozen-BC rehearsal; no route/safety changes."""
import json
import math
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from tasks import CENTRE_Y, angle_error


class PhaseTrainingReward(gym.Wrapper):
    """Measure progress toward the CURRENT phase, not a diagonal exit shortcut."""
    def __init__(self, env, stall_steps=150, trace_path=None):
        super().__init__(env)
        self.stall_limit = stall_steps
        self.trace = open(trace_path, "w") if trace_path else None
        self.trace_steps = 0

    def reset(self, **kwargs):
        result = self.env.reset(**kwargs)
        self.previous_command = np.zeros(2)
        self.phase_seen = set()
        self.last_phase = None
        self.best_error = None
        self.stall = 0
        return result

    @staticmethod
    def error(phase, pose, task):
        centre = np.array([0., CENTRE_Y])
        if task.name == 'left_to_right_reverse' or phase == 3:
            return float(np.linalg.norm(pose[:2]-task.goal[:2]))
        if phase == 2:
            return abs(angle_error(task.goal[2], pose[2]))
        return float(np.linalg.norm(pose[:2]-centre))

    def step(self, action):
        raw = self.env.unwrapped
        before = raw.pose.copy()
        context = raw.cached_context.copy()
        phase = int(np.argmax(context[:4]))
        eligible = bool(raw.label_valid and not raw.teacher.status()['relocalizing'])
        error_before = self.error(phase, before, raw.task)
        observation, original, done, truncated, info = self.env.step(action)
        error_after = self.error(phase, raw.pose, raw.task)
        valid = bool(eligible and not info.get('teacher_intervened') and
                     not info.get('safety_blocked'))
        gain = 2. if phase == 2 else 8.
        progress = gain*float(np.clip(error_before-error_after, -.08, .08)) if valid else 0.
        # During TURN keep the vehicle within the existing centre envelope.
        drift = (-.1*max(0., float(np.linalg.norm(raw.pose[:2]-[0., CENTRE_Y]))-.05)
                 if phase == 2 and valid else 0.)
        normalized = np.asarray(info['executed_action'])/[.2, .6]
        smooth = -.02*float(np.sum((normalized-self.previous_command)**2))
        self.previous_command = normalized
        next_phase = info['phase_id']
        transition = 0.
        if valid and next_phase > phase and next_phase not in self.phase_seen:
            transition = 2.
            self.phase_seen.add(next_phase)
        # A best-so-far error prevents oscillation from resetting stagnation.
        # Invalid perception/safety waits neither advance nor exhaust this budget.
        if phase != self.last_phase:
            self.best_error, self.stall, self.last_phase = error_before, 0, phase
        if valid:
            if error_after < self.best_error-.01:
                self.best_error, self.stall = error_after, 0
            else:
                self.stall += 1
        stalled = bool(valid and self.stall >= self.stall_limit and not done)
        terminal = 20. if info['success'] else -20. if done or stalled else 0.
        # Time-limit stays a truncation; a measured stagnation is a true failure.
        if stalled:
            done, truncated = True, False
            raw.active = False
            raw._publish((0., 0.))
            raw._drain()
        components = dict(progress=progress, turn_centre=drift, time=-.02,
                          smooth=smooth, blocked=-.1*float(info.get('safety_blocked', False)),
                          transition=transition, terminal=terminal)
        info = dict(info, no_progress=stalled, phase_stall_steps=self.stall,
                    original_reward=original, reward_components=components,
                    phase_target_error=error_after)
        if self.trace is not None:
            self.trace_steps += 1
            row = dict(step=self.trace_steps, task=raw.task.name, phase=phase,
                       next_phase=next_phase, eligible=valid, error_before=error_before,
                       error_after=error_after, requested=info.get("requested_action"),
                       executed=info["executed_action"], components=components,
                       no_progress=stalled, hold=info.get("perception_hold", False),
                       localization_hold=info.get("localization_hold", False),
                       route_state=info.get("route_state"), terminated=bool(done), truncated=bool(truncated))
            self.trace.write(json.dumps(row)+"\n")
            if self.trace_steps % 64 == 0 or done or truncated:
                self.trace.flush()
        return observation, sum(components.values()), done, truncated, info

    def close(self):
        try:
            if self.trace is not None:
                self.trace.close()
        finally:
            self.env.close()


class AnchoredPPO(PPO):
    """PPO update followed by one small actor-only frozen-BC rehearsal update.

    This is an auxiliary rehearsal step, not a KL trust-region guarantee.
    Reference and auxiliary optimizer are attached at runtime and never serialized.
    """
    def _excluded_save_params(self):
        return super()._excluded_save_params()+['bc_reference', 'bc_optimizer']

    def attach_bc(self, policy, learning_rate=1e-5, coefficient=.1):
        self.bc_reference = policy
        policy.set_training_mode(False)
        for parameter in policy.parameters():
            parameter.requires_grad_(False)
        actor = list(self.policy.mlp_extractor.policy_net.parameters())+list(self.policy.action_net.parameters())
        self.bc_optimizer = torch.optim.Adam(actor, lr=learning_rate)
        self.bc_coefficient = coefficient

    def train(self):
        super().train()
        reference = getattr(self, 'bc_reference', None)
        if reference is None:
            return
        # Respect critic warmup's frozen actor.
        actor = self.bc_optimizer.param_groups[0]['params']
        if not any(parameter.requires_grad for parameter in actor):
            return
        batch = next(self.rollout_buffer.get(self.batch_size))
        with torch.no_grad():
            target = reference.get_distribution(batch.observations).distribution.mean
        prediction = self.policy.get_distribution(batch.observations).distribution.mean
        loss = torch.nn.functional.mse_loss(prediction, target)
        self.bc_optimizer.zero_grad()
        (self.bc_coefficient*loss).backward()
        torch.nn.utils.clip_grad_norm_(actor, self.max_grad_norm)
        self.bc_optimizer.step()
        self.logger.record('train/bc_anchor_mse', loss.item())
