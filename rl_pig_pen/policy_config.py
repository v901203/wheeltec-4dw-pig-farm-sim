"""Shared spaces and PPO architecture for online RL and offline imitation."""

import gymnasium as gym
import numpy as np

from navigation import MAX_ANG, MAX_LIN, N_LIDAR

PATROL_CONTEXT_DIM = 11  # 8-state one-hot, wall angle, wall confidence, reverse intent
PATROL_OBSERVATION_DIM = N_LIDAR + 2 + PATROL_CONTEXT_DIM

PPO_PARAMS = dict(
    n_steps=2048, batch_size=512, n_epochs=10, gamma=0.99,
    gae_lambda=0.95, clip_range=0.2, ent_coef=0.01,
    learning_rate=3e-4, verbose=1, policy_kwargs=dict(net_arch=[256, 256]),
)


def policy_spaces(*, allow_reverse=False, patrol=False):
    """Policy spaces for the 38-D corridor or 49-D LiDAR-only patrol policy."""
    dimension = PATROL_OBSERVATION_DIM if patrol else N_LIDAR + 2
    low = np.zeros(dimension, dtype=np.float32)
    low[N_LIDAR] = -1.0
    if patrol:
        # context[8] is wall_parallel_error / (pi/2), the rest is [0, 1].
        low[N_LIDAR + 2 + 8] = -1.0
    action_low = -MAX_LIN if allow_reverse else 0.0
    return (gym.spaces.Box(low, np.ones(dimension, dtype=np.float32)),
            gym.spaces.Box(np.array([action_low, -MAX_ANG], dtype=np.float32),
                           np.array([MAX_LIN, MAX_ANG], dtype=np.float32)))
