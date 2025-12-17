import numpy as np


class VectorHoming:
    def __init__(self, step_length, home_threshold):
        self.step_length = step_length
        self.home_threshold = home_threshold

    def reset(self):
        pass

    def step(self, pos):
        v = -pos
        dist = np.linalg.norm(v)
        if dist < self.home_threshold or dist < self.step_length:
            return 0.0, 0.0, True
        
         # Safety catch for "Runaway" agents
        if dist > 10000: # 10 meters
             return 0.0, 0.0, True # Force Stop (Give Up)

        direction = v / dist
        dx, dy = self.step_length * direction
        return dx, dy, False