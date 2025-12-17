# Do no edit the config file. Use the UI.

from dataclasses import dataclass
import numpy as np

@dataclass
class AgentConfig:
    agent_speed: float = 3.0

    heading_bias_mean: float = 0.0            # rad
    heading_bias_std: float = np.deg2rad(0.05) # rad (initial compass offset)

    heading_noise_std: float = np.deg2rad(0.5) # rad per step
    stride_noise_std: float = 0.02             # fraction

    scan_interval: float = 100.0  # mm (Distance between Sun Scans)

@dataclass
class CRWConfig:
    turn_std: float = np.deg2rad(15.0)

@dataclass
class HomingConfig:
    home_threshold: float = 5.0

@dataclass
class EnvironmentConfig:
    food_detection_radius: float = 10.0
    home_detection_radius: float = 20.0
    n_food_items: int = 15
    food_spawn_min_radius: float = 200.0
    food_spawn_max_radius: float = 400.0
