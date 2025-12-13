import random
import math
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLScatterPlotItem
import numpy as np

def random_point_outside_radius(min_radius, max_radius):
        while True:
            x = random.uniform(-max_radius, max_radius)
            y = random.uniform(-max_radius, max_radius)

            if math.sqrt(x*x + y*y) >= min_radius:
                return x, y


class TrailManager:
    """
    Manages two trails: true trail (real ant position) and simulated trail (ant's perceived path)
    """
    def __init__(self):
        # Start with empty data 
        self.true_trail_data = np.zeros((0, 3), dtype=float)
        self.sim_trail_data  = np.zeros((0, 3), dtype=float)

        # Create GLScatterPlotItems
        self.true_trail = GLScatterPlotItem(
            pos=self.true_trail_data,
            color=(0.6, 0.4, 0, 0.8),   # darker brown
            size=4,                    # dot size
            pxMode=True               # express size in pixels -> consistent size
        )

        self.sim_trail = GLScatterPlotItem(
            pos=self.sim_trail_data,
            color=(0.5, 0.3, 0.9, 0.8),  # purple-ish
            size=4,
            pxMode=True
)

    def update(self, true_pos, sim_pos):
        true_pos = np.array([true_pos[0], true_pos[1], 0.0])
        sim_pos  = np.array([sim_pos[0],  sim_pos[1],  0.0])

        self.true_trail_data = np.vstack([self.true_trail_data, true_pos])
        self.sim_trail_data  = np.vstack([self.sim_trail_data,  sim_pos])

        self.true_trail.setData(pos=self.true_trail_data)
        self.sim_trail.setData(pos=self.sim_trail_data)

    def reset(self):
        """Clear both trails and reset GLLinePlotItems."""
        self.true_trail_data = np.zeros((0, 3), dtype=float)
        self.sim_trail_data = np.zeros((0, 3), dtype=float)

        self.true_trail.setData(pos=self.true_trail_data)
        self.sim_trail.setData(pos=self.sim_trail_data)
