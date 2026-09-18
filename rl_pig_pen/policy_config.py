"""Shared spaces and PPO architecture for online RL and offline imitation."""

import gymnasium as gym
import numpy as np

from navigation import MAX_ANG, MAX_LIN, N_LIDAR

PPO_PARAMS = dict(
    n_steps=2048, batch_size=512, n_epochs=10, gamma=0.99,
    gae_lambda=0.95, clip_range=0.2, ent_coef=0.01,
    learning_rate=3e-4, verbose=1, policy_kwargs=dict(net_arch=[256, 256]),
)


def policy_spaces():
    low = np.zeros(N_LIDAR + 2, dtype=np.float32)
    low[N_LIDAR] = -1.0
    return (gym.spaces.Box(low, np.ones(N_LIDAR + 2, dtype=np.float32)),
            gym.spaces.Box(np.array([0, -MAX_ANG], dtype=np.float32),
                           np.array([MAX_LIN, MAX_ANG], dtype=np.float32)))
