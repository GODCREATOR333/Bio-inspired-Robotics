# correlated_random_walk.py
import numpy as np


class CorrelatedRandomWalk:
    def __init__(
        self,
        step_length,
        turn_std,  # angular noise (radians)
        rng=None,
    ):
        """
        step_length : fixed distance per step
        turn_std    : std dev of turning angle (controls persistence)
        rng         : np.random.Generator (optional)
        """
        self.step_length = step_length
        self.turn_std = turn_std
        self.rng = rng if rng is not None else np.random.default_rng()

        self.heading = 0.0  # radians


    def reset(self, heading=None):
        """
        Reset internal CRW state.
        """
        if heading is None:
            self.heading = self.rng.uniform(0, 2 * np.pi)
        else:
            self.heading = heading


    def step(self):
        """
        One CRW step.
        Returns (dx, dy)
        """
        # correlated turn
        dtheta = self.rng.normal(0.0, self.turn_std)
        self.heading += dtheta

        dx = self.step_length * np.cos(self.heading)
        dy = self.step_length * np.sin(self.heading)

        return dx, dy
