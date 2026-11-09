"""Explicit contract and conversion for continuous-control observations."""
import hashlib
import json
from pathlib import Path
import numpy as np
from learning_common import contract as legacy_contract


def contract():
    base = dict(legacy_contract())
    base.update(schema='pig_pen_continuous_junction_v1',
                policy_scope='continuous slowdown-to-corridor-handoff, no internal FSM gates',
                continuous_env_sha256=hashlib.sha256(Path(__file__).with_name('continuous_env.py').read_bytes()).hexdigest(),
                continuous_common_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                context='reserved_zero4,centre_xy/2m,exit_heading/pi,validity2,reverse_task')
    return base


def save_model(model, path, spec):
    path = Path(path).with_suffix('.zip')
    model.save(path)
    path.with_suffix('.json').write_text(json.dumps(dict(environment=contract(), policy=spec), indent=2))


def check_model(path):
    path = Path(path)
    metadata = json.loads(path.with_suffix('.json').read_text())
    if metadata['environment'] != contract():
        raise ValueError('Expected a current continuous-control model; old FSM-policy weights require explicit BC conversion')
    return metadata['policy']


def convert_observations(observations, task):
    """Convert only recorded causal context, never insert world pose into BC."""
    x = np.asarray(observations, dtype=np.float32).copy()
    frames = x.reshape(-1, 161)
    phases = np.argmax(frames[:,151:155], axis=1)
    early = (phases < 2) & (frames[:,159] > .5)
    turn = {'main_to_left':.5, 'right_to_main':-.5, 'left_to_right_reverse':0.}[task]
    frames[early,157] = (frames[early,157]+turn+1.) % 2.-1.
    frames[:,151:155] = 0.
    frames[:,160] = float(task == 'left_to_right_reverse')
    return x
