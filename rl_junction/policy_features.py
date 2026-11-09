"""Fixed unit scaling inside the policy, shared by BC training and inference."""
import math
import torch
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from policy_context import OBS_SIZE, RAW_OBS_SIZE


class LocalGeometryFeatures(BaseFeaturesExtractor):
    """Emphasize centimetre-level geometry without changing sensor observations.

    A buffer travels with the saved SB3 model, so inference and subsequent PPO
    use precisely the same transformation as BC. No ground-truth pose is used.
    """
    def __init__(self, observation_space, geometry_gain=1.):
        size = observation_space.shape[0]
        if size % OBS_SIZE or not math.isfinite(geometry_gain) or geometry_gain <= 0:
            raise ValueError("Expected stacked junction frames and a finite positive gain")
        super().__init__(observation_space, features_dim=size)
        scale = torch.ones(size)
        for offset in range(0, size, OBS_SIZE):
            # Context layout: phase4, centre x/y, heading error, validity2, reverse1.
            scale[offset+RAW_OBS_SIZE+4:offset+RAW_OBS_SIZE+7] = geometry_gain
        self.register_buffer("scale", scale)

    def forward(self, observations):
        return observations.flatten(start_dim=1)*self.scale
