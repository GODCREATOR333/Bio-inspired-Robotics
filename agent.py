import numpy as np
from stl import mesh
import pyqtgraph.opengl as gl
from PyQt5 import QtGui
from geometry import create_circle


# What is there in this file ?
# This file is supposed to be only concerned with the tunable parameters to experiment with .
# Noise parameters
# Geometry
# State/position of the agent/ant



class Agent_Model:
    def __init__(self, stl_path,agent_cfg, scale=0.01):
        """
        :param stl_path: Path to STL file
        :param scale: Scaling factor to shrink the model
        :param bias_mean: mean of Gaussian bias (applied to x,y)
        :param bias_std:  std of Gaussian bias (applied to x,y)
        :param drift_std: std of Gaussian *incremental* drift per move (random walk)
        """

        self.agent_speed = agent_cfg.agent_speed
        self.bias_mean   = agent_cfg.bias_mean
        self.bias_std    = agent_cfg.bias_std
        self.drift_std   = agent_cfg.drift_std


        # Sampleable noise state
        self.bias = np.zeros(2, dtype=float)     # fixed bias (x,y)
        self.drift = np.zeros(2, dtype=float)    # accumulated drift (x,y)

        # Store scale factor
        self.scale_factor = scale
        self.detection_circle = None

        # --- Load STL ---
        self.raw_mesh = mesh.Mesh.from_file(stl_path)

        # --- Center vertices around origin ---
        self.vertices = np.array(self.raw_mesh.vectors.reshape(-1, 3), dtype=np.float32)
        center = self.vertices.mean(axis=0)
        self.vertices -= center  # center the mesh

        # --- Faces ---
        self.faces = np.arange(len(self.vertices)).reshape(-1, 3)

        # --- Dynamic (true) state ---
        self.position = np.array([0.0, 0.0, 0.0], dtype=float)  # true world position
        self.perceived_position = self.position.copy() #Perceived Position of the ant
        self.rotation = 0.0  # degrees

        # --- Perceived (noisy) state ---
        # Initially matches true position (bias + drift applied when move() is called)
        self.perceived_position = self.position.copy()

        # --- Create GLMeshItem ---
        self.mesh_item = gl.GLMeshItem(
            vertexes=self.vertices,
            faces=self.faces,
            smooth=False,
            drawEdges=True,
            edgeColor=(0, 0, 0, 1),
            color=(1, 0.5, 0.0, 1),
        )
        self.mesh_item.scale(scale, scale, scale)

        # init noise samples
        self.resample_bias()
        self.reset_drift()

    # --- Noise utilities ---
    def resample_bias(self):
        """Sample a new fixed Gaussian bias for x,y."""
        self.bias = np.random.normal(self.bias_mean, self.bias_std, size=2)

    def reset_drift(self):
        """Reset accumulated drift to zero."""
        self.drift[:] = 0.0

    def configure_noise(self, bias_mean=None, bias_std=None, drift_std=None, resample_bias=False):
        """Change noise parameters at runtime. Optionally resample bias immediately."""
        if bias_mean is not None:
            self.bias_mean = bias_mean
        if bias_std is not None:
            self.bias_std = bias_std
        if drift_std is not None:
            self.drift_std = drift_std
        if resample_bias:
            self.resample_bias()

    # Place it in the world
    def spawn(self, x=0, y=0, z=0, view=None):
        # Reset transforms and scale (keeps consistent size)
        self.mesh_item.resetTransform()
        self.mesh_item.scale(self.scale_factor, self.scale_factor, self.scale_factor)

        # Set true position and move mesh
        self.position[:] = (x, y, z)
        self.mesh_item.translate(x, y, z)

        # Reset perceived state to true + current bias (no drift)
        self.reset_drift()
        self.perceived_position = np.array([x + self.bias[0], y + self.bias[1], z], dtype=float)

        if self.detection_circle is not None:
            self.detection_circle.resetTransform()
            self.detection_circle.translate(x, y, 0)

        # Create circle only once if view is provided
        elif view is not None:
            self.detection_circle = create_circle(radius=10, x=x, y=y, z=0, color=(1,0,0,0.3))
            view.addItem(self.detection_circle)

    # Move in XY plane (true motion)
    def move(self, dx, dy):
        # Update true position
        self.position[0] += dx
        self.position[1] += dy
        self.mesh_item.translate(dx, dy, 0)

        # Update drift: random walk (Gaussian increment)
        if self.drift_std > 0.0:
            inc = np.random.normal(0.0, self.drift_std, size=2)
            self.drift += inc

        # Compute perceived position = true + bias + drift (Z unchanged)
        px = self.position[0] + self.bias[0] + self.drift[0]
        py = self.position[1] + self.bias[1] + self.drift[1]
        pz = self.position[2]  # keep Z same (no noise)
        self.perceived_position = np.array([px, py, pz], dtype=float)

        # If detection circle exists, move it with true position
        if self.detection_circle is not None:
            self.detection_circle.translate(dx, dy, 0)

    # Rotate around Z-axis
    def rotate(self, angle_deg):
        self.rotation += angle_deg
        self.mesh_item.rotate(angle_deg, 0, 0, 1)  # Z-axis
    
    def get_true_pos(self):
        return self.position.copy()

    def get_sim_pos(self):
        return self.perceived_position.copy()
    