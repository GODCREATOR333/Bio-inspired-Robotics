# homing_policy.py
import numpy as np


class VectorHoming:
    def __init__(self, step_length=1.0, home_threshold=1.0):
        self.step_length = step_length
        self.home_threshold = home_threshold

    def reset(self):
        pass

    def step(self, pos):
        """
        pos : perceived position (x, y)
        """
        v = -pos
        dist = np.linalg.norm(v)

        if dist < self.home_threshold:
            return 0.0, 0.0, True

        direction = v / dist
        dx, dy = self.step_length * direction
        return dx, dy, False