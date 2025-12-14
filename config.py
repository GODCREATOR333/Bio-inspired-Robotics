# config.py
from dataclasses import dataclass
import numpy as np

@dataclass
class AgentConfig:
    agent_speed: float = 3.0
    bias_mean: float = 1.5
    bias_std: float = np.deg2rad(5.0)
    drift_std: float = np.deg2rad(5.5)

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
