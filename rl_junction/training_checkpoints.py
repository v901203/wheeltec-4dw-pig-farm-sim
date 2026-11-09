"""Sequential validation on the existing simulator, without a second ROS node."""
import json
from pathlib import Path
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from learning_common import HistoryObservation, save_model


def validation_score(rows):
    """Protect the weakest task first; speed breaks ties only among successes."""
    rates = [sum(r['success'] for r in rows if r['task'] == task) /
             sum(r['task'] == task for r in rows) for task in sorted({r['task'] for r in rows})]
    times = [r['seconds'] for r in rows if r['success']]
    return (min(rates), float(np.mean(rates)), -sum(r['collision'] for r in rows),
            -sum(r['interventions'] for r in rows), -float(np.mean(times)) if times else -1e9)


class ValidationCheckpoints(BaseCallback):
    def __init__(self, run, spec, raw_env, interval=5120, episodes=3, seed=810000, save_function=None):
        super().__init__()
        self.run, self.spec, self.raw = Path(run), spec, raw_env
        self.save_function = save_function
        self.interval, self.episodes, self.seed = interval, episodes, seed
        self.last_step, self.best = None, None
        self.eval_tasks = spec["tasks"]

    def _on_step(self):
        return True

    def _on_rollout_start(self):
        if self.last_step is None or self.num_timesteps-self.last_step >= self.interval:
            self.evaluate()

    def _on_training_end(self):
        if self.last_step != self.num_timesteps:
            self.evaluate()

    def evaluate(self):
        step = self.num_timesteps
        destination = self.run/'checkpoints'/f'step_{step:09d}'
        destination.parent.mkdir(parents=True, exist_ok=True)
        (self.save_function or save_model)(self.model, destination, self.spec)
        # Reuse the sole simulator controller. Bypass training Monitor/diagnostics,
        # round-robin and history buffers, so validation never enters PPO statistics.
        env = HistoryObservation(self.raw, self.spec['history'])
        old_task, old_rng = self.raw.fixed_task, self.raw.np_random
        rows = []
        trace = destination.with_suffix(".steps.jsonl").open("w")
        try:
            for i, task in enumerate(self.eval_tasks):
                self.raw.fixed_task = task
                for episode in range(self.episodes):
                    obs, _ = env.reset(seed=self.seed+episode*len(self.eval_tasks)+i)
                    interventions, reward_sum, episode_step = 0, 0., 0
                    print(f"Validation START step={step} task={task} episode={episode+1}", flush=True)
                    while True:
                        action, _ = self.model.predict(obs, deterministic=True)
                        obs, reward, done, truncated, info = env.step(action)
                        episode_step += 1
                        record = dict(training_step=step, task=task, episode=episode+1, step=episode_step,
                            action=np.asarray(action).tolist(), requested=info.get("requested_action"),
                            executed=info.get("executed_action"), geometry_valid=info.get("geometry_valid"),
                            perception_hold=info.get("perception_hold"), blocked=info.get("safety_blocked"),
                            exit_depth=info.get("exit_depth_m"), exit_lateral=info.get("exit_lateral_m"),
                            exit_heading=info.get("exit_heading_error"), state=info.get("route_state"),
                            reward=float(reward), success=bool(info["success"]),
                            done=bool(done), truncated=bool(truncated))
                        trace.write(json.dumps(record)+"\n")
                        trace.flush()
                        if episode_step % 50 == 0 or done or truncated:
                            print("Validation progress: "+json.dumps(record), flush=True)
                        reward_sum += reward
                        interventions += int(info.get('safety_blocked', False) or info.get('teacher_intervened', False))
                        if done or truncated:
                            break
                    rows.append(dict(task=task, seed=self.seed+episode*len(self.eval_tasks)+i,
                        success=bool(info['success']), collision=bool(info['collision']),
                        seconds=info['sim_seconds'], interventions=interventions,
                        reward=reward_sum, truncated=bool(truncated), state=info['route_state']))
                    destination.with_suffix(".partial.json").write_text(json.dumps(dict(complete=False,
                        timesteps=step, episodes=rows), indent=2))
        finally:
            trace.close()
            self.raw.fixed_task, self.raw.np_random = old_task, old_rng
            # Validation changes physical state: discard the partial training episode
            # and start with fresh observations/history. Never resume stale observations.
            self.model._last_obs = self.training_env.reset()
            self.model._last_episode_starts = np.ones((self.training_env.num_envs,), dtype=bool)
        score = validation_score(rows)
        report = dict(timesteps=step, score=score, episodes=rows,
            note='Fixed validation seeds for selection; independent held-out evaluation still required. Training partial episode reset at rollout boundary.')
        report_path = destination.with_suffix('.evaluation.json')
        report_path.write_text(json.dumps(report, indent=2))
        if self.best is None or score > self.best:
            (self.save_function or save_model)(self.model, self.run/'best_model', self.spec)
            (self.run/'best_evaluation.json').write_text(json.dumps(report, indent=2))
            self.best = score
        with (self.run/'validation.jsonl').open('a') as stream:
            stream.write(json.dumps(report)+'\n')
        self.last_step = step
        for task in self.eval_tasks:
            data = [r for r in rows if r['task'] == task]
            self.logger.record(f'validation/{task}/success_rate', float(np.mean([r['success'] for r in data])))
        self.logger.record('time/total_timesteps', step)
        self.logger.dump(step=step)
        print(f'Validation at {step}: score={score}; best={self.best}', flush=True)
