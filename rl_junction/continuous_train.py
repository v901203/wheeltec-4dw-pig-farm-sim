#!/usr/bin/env python3
"""Explicit offline BC conversion and isolated continuous-junction PPO."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import signal
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
from learning_common import check_model as check_legacy, model_spec, history_windows, HistoryObservation
from continuous_common import contract, check_model, save_model, convert_observations
from continuous_env import ContinuousJunctionEnv
from phase_training import AnchoredPPO
from train import TaskRoundRobin
from training_checkpoints import ValidationCheckpoints


class ContinuousValidation(ValidationCheckpoints):
    def evaluate(self):
        initial = self.last_step is None
        super().evaluate()
        if initial and self.best[0] == 0.:
            raise RuntimeError("Converted BC has zero successes in at least one task; inspect validation before PPO")


class Progress(BaseCallback):
    def __init__(self, run, spec):
        super().__init__(); self.run, self.spec, self.rollouts = run, spec, 0
    def _on_step(self): return True
    def _on_rollout_start(self):
        warming = self.rollouts < 2
        for module in (self.model.policy.mlp_extractor.policy_net, self.model.policy.action_net):
            for parameter in module.parameters(): parameter.requires_grad_(not warming)
        self.model.policy.log_std.requires_grad_(not warming)
        save_model(self.model, self.run/'latest', self.spec)
        (self.run/'progress.json').write_text(json.dumps(dict(timesteps=self.num_timesteps, rollouts=self.rollouts)))
        self.rollouts += 1
    def _on_training_end(self):
        for parameter in self.model.policy.parameters(): parameter.requires_grad_(True)


class Trace(gym.Wrapper):
    def __init__(self, env, path):
        super().__init__(env); self.stream = path.open('w'); self.count = 0
    def step(self, action):
        result = self.env.step(action)
        self.count += 1
        self.stream.write(json.dumps(dict(step=self.count, reward=result[1], terminated=result[2],
                                         truncated=result[3], info=result[4]))+'\n')
        if self.count % 64 == 0 or result[2] or result[3]: self.stream.flush()
        return result
    def close(self):
        try: self.stream.close()
        finally: self.env.close()


def offline_bc(args, run):
    source = check_legacy(args.init_model)
    spec = model_spec(source)
    manifest = json.loads(args.split.read_text())
    arrays = {}
    for split, files in manifest['splits'].items():
        xx, yy, ww = [], [], []
        for entry in files:
            path = Path(entry['path'])
            if hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
                raise ValueError('Demonstration hash mismatch')
            with np.load(path, allow_pickle=False) as data:
                mask = data['label_valid'].astype(bool)
                converted = convert_observations(data['observations'], entry['task'])
                xx.append(history_windows(converted, spec['history'])[mask])
                yy.append(data['actions'][mask])
                # Preserve phase-balanced training while removing phase inputs.
                phases = data['phase_ids'][mask]
                weights = np.zeros(len(phases))
                for phase in np.unique(phases):
                    weights[phases == phase] = 1./np.sum(phases == phase)/len(np.unique(phases))
                ww.append(weights)
        arrays[split] = (torch.tensor(np.concatenate(xx)).float(), torch.tensor(np.concatenate(yy)).float(),
                         torch.tensor(np.concatenate(ww)).double())
    model = PPO.load(source, device='cpu')
    actor = list(model.policy.mlp_extractor.policy_net.parameters())+list(model.policy.action_net.parameters())
    optimizer = torch.optim.Adam(actor, lr=1e-4)
    best = float('inf')
    spec = dict(spec, control_mode='continuous_junction', conversion_source=str(source))
    for epoch in range(args.epochs):
        x,y,w = arrays['train']
        model.policy.set_training_mode(True)
        for batch in torch.multinomial(w,len(x),replacement=True).split(256):
            prediction = model.policy.get_distribution(x[batch]).distribution.mean
            loss = torch.nn.functional.mse_loss(prediction,y[batch])
            optimizer.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(actor,1.); optimizer.step()
        model.policy.set_training_mode(False)
        with torch.no_grad():
            x,y,_ = arrays['validation']
            mse = torch.nn.functional.mse_loss(model.policy.get_distribution(x).distribution.mean,y).item()
        print(f'BC epoch={epoch+1} validation_mse={mse:.6f}', flush=True)
        with (run/'bc_metrics.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(epoch=epoch+1, validation_mse=mse))+'\n')
        if mse < best:
            best = mse
            # PPO's critic and Adam moments are not transferred across objectives.
            model.num_timesteps = 0
            model.policy.optimizer.state.clear()
            save_model(model,run/'bc_model',spec)
    (run/'completion.json').write_text(json.dumps(dict(complete=True, validation_mse=best,
        source_split=str(args.split), note='Converted old FSM demonstrations still contain stop-and-turn behavior; closed-loop validation required.')))


def train(args, run):
    spec = check_model(args.init_model)
    raw = ContinuousJunctionEnv(max_steps=300, available_tasks=spec['tasks'])
    env = Monitor(HistoryObservation(TaskRoundRobin(Trace(raw,run/'steps.jsonl'),spec['tasks']),spec['history']),
                  str(run/'episodes.csv'), info_keywords=('task','success','collision','no_progress'))
    model = None
    try:
        model = AnchoredPPO.load(args.init_model, env=env, device='cpu', seed=args.seed,
            n_steps=256,batch_size=64,n_epochs=4,learning_rate=1e-5,clip_range=.1,target_kl=.01)
        reference = PPO.load(args.init_model,device='cpu')
        model.attach_bc(reference.policy,1e-5,.1)
        model.set_logger(configure(str(run/'metrics'),['stdout','csv','json']))
        validation = ContinuousValidation(run,spec,raw,interval=2560,episodes=args.eval_episodes,
                                          seed=910000,save_function=save_model)
        model.learn(args.timesteps, reset_num_timesteps=False,callback=[Progress(run,spec),validation])
        save_model(model,run/'model',spec)
        (run/'completion.json').write_text(json.dumps(dict(complete=True,timesteps=model.num_timesteps)))
    except BaseException as exc:
        if model is not None: save_model(model,run/'interrupted_model',spec)
        failure_file = 'interrupted.json' if isinstance(exc, KeyboardInterrupt) else 'infrastructure_error.json'
        (run/failure_file).write_text(json.dumps(dict(complete=False,type=type(exc).__name__,error=str(exc))))
        raise
    finally:
        env.close()
        if model is not None: model.logger.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['bc','ppo'],required=True)
    parser.add_argument('--init-model',type=Path,required=True)
    parser.add_argument('--split',type=Path)
    parser.add_argument('--epochs',type=int,default=40)
    parser.add_argument('--timesteps',type=int,default=5120)
    parser.add_argument('--eval-episodes',type=int,default=3)
    parser.add_argument('--seed',type=int,default=74000)
    parser.add_argument('--output',type=Path,default=Path('logs/rl_junction_continuous'))
    args=parser.parse_args()
    if min(args.epochs,args.timesteps,args.eval_episodes)<1: parser.error('Counts must be positive')
    if args.mode=='bc' and not args.split: parser.error('BC conversion requires the original split manifest')
    torch.manual_seed(args.seed); np.random.seed(args.seed); torch.set_num_threads(1)
    run=args.output/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run.mkdir(parents=True)
    print(f'Run directory: {run}',flush=True)
    (run/'settings.json').write_text(json.dumps(dict(vars(args),contract=contract()),default=str,indent=2))
    def stop(*_): raise KeyboardInterrupt
    old=signal.signal(signal.SIGTERM,stop)
    try:
        if args.mode=='bc': offline_bc(args,run)
        else: train(args,run)
    finally: signal.signal(signal.SIGTERM,old)

if __name__=='__main__': main()
