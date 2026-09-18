"""Full-route training with one PPO decision per Gym transition.

An action drives one corridor frame, then the FSM runs until the next corridor
decision or episode end. FSM commands are never recorded as policy actions.
Discounting is per policy decision, not per simulated second.
"""

from navigation import collision_detected, scan_features
from pig_pen_env import PigPenEnv, _pose, _stamp


class PatrolTrainingEnv(PigPenEnv):
    ENDPOINT_REWARD = 10.0
    RETURN_REWARD = 20.0
    COMPLETE_REWARD = 100.0
    FAILURE_PENALTY = 20.0

    def __init__(self, **kwargs):
        super().__init__(mode="patrol", **kwargs)
        self._reported_status = None

    def _report(self, info):
        status = (info.get("state"), info.get("junction"), info.get("branches_done"))
        if status != self._reported_status:
            print(f"{getattr(self, 'log_prefix', '')}Patrol: state={status[0]} junction={status[1]} "
                  f"returned={status[2]}/{info.get('branches_total', 6)}", flush=True)
            self._reported_status = status

    def _advance_to_policy(self):
        """Consume each snapshot once; return only where an RL action is used."""
        frames = 0
        while True:
            scan, odom = self._latest_snapshot
            features = scan_features(scan, self.cfg)
            if collision_detected(features, self.cfg):
                result = PigPenEnv._step(self, [0.0, 0.0], snapshot=(scan, odom), arbitrate=False)
                return (*result, frames)
            command = self.controller.command(features, _pose(odom), _stamp(scan))
            info = self.controller.info()
            self._report(info)
            if command is None:
                self._last_obs = features.observation
                return self._last_obs.copy(), 0.0, False, False, info, frames
            previous_steps = self._steps
            result = PigPenEnv._step(self, command, snapshot=(scan, odom), arbitrate=False)
            frames += self._steps - previous_steps
            if result[2] or result[3]:
                return (*result, frames)

    def reset(self, *, seed=None, options=None):
        _, info = super().reset(seed=seed, options=options)
        self._reported_status = None
        try:
            obs, _, terminated, truncated, patrol_info, frames = self._advance_to_policy()
        except BaseException:
            self._ros.publish_cmd(0.0, 0.0)
            self._episode_done = True
            raise
        if terminated or truncated:
            raise RuntimeError(f"Patrol could not reach the first corridor: {patrol_info}")
        info.update(patrol_info, mode="patrol_train", fsm_steps=frames, physical_steps=self._steps)
        self._policy_steps = 0
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        if info.get("failure_reason") == "sensor_fault":
            reward = -self.FAILURE_PENALTY
            info.update(mode="patrol_train", collision=False, physical_steps=self._steps)
        return obs, reward, terminated, truncated, info

    def _step(self, action):
        before = self.controller.info()
        steps_before = self._steps
        obs, reward, terminated, truncated, info = PigPenEnv._step(
            self, action, snapshot=self._latest_snapshot, arbitrate=False)
        frames = 0
        if not (terminated or truncated):
            obs, _, terminated, truncated, next_info, frames = self._advance_to_policy()
            info.update(next_info)
        # Credits route outcomes to the preceding policy decision. Automatic
        # forward motion/turning never supplies the corridor velocity reward.
        reward += self.ENDPOINT_REWARD * (info["endpoints_reached"] - before["endpoints_reached"])
        reward += self.RETURN_REWARD * (info["branches_done"] - before["branches_done"])
        if info.get("collision"):
            reward = -100.0
        elif info.get("success"):
            reward += self.COMPLETE_REWARD
        elif terminated or truncated:
            reward -= self.FAILURE_PENALTY
        self._policy_steps += 1
        info.update(mode="patrol_train", controller="rl", fsm_steps=frames,
                    physical_steps=self._steps, transition_frames=self._steps-steps_before,
                    policy_steps=self._policy_steps)
        if terminated or truncated:
            print(f"{getattr(self, 'log_prefix', '')}Patrol episode: reason={info.get('failure_reason')} "
                  f"success={info.get('success')} returned={info['branches_done']}/{info['branches_total']} "
                  f"policy_steps={self._policy_steps} physical_steps={self._steps}", flush=True)
        return obs, float(reward), terminated, truncated, info
