#!/usr/bin/env python3
"""Short deterministic continuous BC diagnostics with decision-time raw scans."""
import argparse
import json
from pathlib import Path
from datetime import datetime
from copy import deepcopy
import numpy as np
import torch
from stable_baselines3 import PPO
from continuous_common import check_model
from continuous_env import ContinuousJunctionEnv
from learning_common import HistoryObservation
from navigation import command_clearance_masks


def encode(value):
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    raise TypeError(type(value).__name__)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--max-steps',type=int,default=150)
    p.add_argument('--seed',type=int,default=910000)
    p.add_argument('--output',type=Path,default=Path('logs/rl_junction_continuous/diagnosis'))
    args=p.parse_args()
    spec=check_model(args.model);torch.set_num_threads(1)
    run=args.output/datetime.now().strftime('%Y%m%d_%H%M%S_%f');run.mkdir(parents=True)
    (run/'settings.json').write_text(json.dumps(dict(model=str(args.model),seed=args.seed,
        max_steps=args.max_steps,manifest=json.loads(args.model.with_suffix('.json').read_text())),indent=2))
    raw=ContinuousJunctionEnv(max_steps=args.max_steps,available_tasks=spec['tasks'])
    env=HistoryObservation(raw,spec['history']);model=PPO.load(args.model,device='cpu')
    summary=[]
    print(f'Diagnostic output: {run}',flush=True)
    try:
        with (run/'steps.jsonl').open('w') as trace:
            for i,task in enumerate(spec['tasks']):
                raw.fixed_task=task;obs,_=env.reset(seed=args.seed+i)
                frames=[];rows=[]
                for step in range(args.max_steps):
                    action=model.predict(obs,deterministic=True)[0]
                    features=raw._features();before_pose=raw.pose.copy();before_context=raw.cached_context.copy()
                    before_valid=raw.geometry_valid
                    frames.append(np.asarray(raw.scan.ranges,dtype=np.float32))
                    angle_min,angle_increment=raw.scan.angle_min,raw.scan.angle_increment
                    policy_command=np.asarray(action)*[.2,.6]
                    if task=='left_to_right_reverse': policy_command[0]=-.1*(action[0]+1.)
                    masks,limits=command_clearance_masks(policy_command,features,raw.cfg,
                        abs(raw.odom.twist.twist.linear.x))
                    hits=[dict(guard=guard,count=int(mask.sum()),nearest_m=float(features.ranges[mask].min()),
                               angles_deg=np.degrees(features.angles[mask])[:6])
                          for guard,mask in masks.items() if mask.any()]
                    obs,reward,done,truncated,info=env.step(action)
                    row=dict(task=task,seed=args.seed+i,step=step+1,before_geometry_valid=before_valid,
                        before_context=before_context,diagnostic_pose_before=before_pose,
                        diagnostic_pose_after=raw.pose.copy(),policy_action=action,policy_command=policy_command,
                        raw_command_clearance_hits=hits,clearance_limits=limits,info=info,
                        tracker_failure=raw.continuous_tracker.failure,
                        tracker_update=deepcopy(raw.continuous_tracker.update_diagnostics),reward=reward,
                        done=done,truncated=truncated)
                    trace.write(json.dumps(row,default=encode)+'\n');trace.flush();rows.append(row)
                    if step%25==0 or done or truncated:
                        print(f'{task} step={step+1} valid={before_valid} policy={policy_command} '
                              f'executed={info["executed_action"]} blocked={info["safety_blocked"]} '
                              f'depth={info["exit_depth_m"]:.2f} tracker={raw.continuous_tracker.failure}',flush=True)
                    if done or truncated: break
                np.savez_compressed(run/f'{task}_scans.npz',ranges=np.stack(frames),
                    angle_min=angle_min,angle_increment=angle_increment,
                    poses=np.stack([r['diagnostic_pose_before'] for r in rows]))
                summary.append(dict(task=task,steps=len(rows),success=info['success'],
                    geometry_holds=sum(not r['before_geometry_valid'] for r in rows),
                    blocked_steps=sum(r['info']['safety_blocked'] for r in rows),
                    diagnostic_start_pose=rows[0]['diagnostic_pose_before'],diagnostic_final_pose=raw.pose.copy(),
                    exit_depth_m=info['exit_depth_m'],exit_heading_error=info['exit_heading_error']))
                (run/'summary.json').write_text(json.dumps(summary,default=encode,indent=2))
    finally: env.close()

if __name__=='__main__': main()
