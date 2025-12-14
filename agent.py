import numpy as np
from stl import mesh
import pyqtgraph.opengl as gl
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
        """

        self.heading = 0.0        # true heading
        self.heading_est = 0.0    # perceived heading
        self.agent_speed = agent_cfg.agent_speed
        self.heading_bias_mean   = agent_cfg.heading_bias_mean
        self.heading_bias_std   = agent_cfg.heading_bias_std
        self.heading_noise_std   = agent_cfg.heading_noise_std
        self.stride_noise_std   = agent_cfg.stride_noise_std
        self.distance_since_last_scan = 0.0

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

    # --- Noise utilities ---
    def resample_bias(self):
        self.heading_bias = np.random.normal(
            self.heading_bias_mean,
            self.heading_bias_std
        )

    def configure_noise(
        self,
        heading_bias_mean=None,
        heading_bias_std=None,
        heading_noise_std=None,
        stride_noise_std=None,
        resample_bias=False
    ):
        if heading_bias_mean is not None:
            self.heading_bias_mean = heading_bias_mean
        if heading_bias_std is not None:
            self.heading_bias_std = heading_bias_std
        if heading_noise_std is not None:
            self.heading_noise_std = heading_noise_std
        if stride_noise_std is not None:
            self.stride_noise_std = stride_noise_std

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

        self.perceived_position = np.array([x, y, z], dtype=float)

        # Odometry init
        self.heading = 0.0
        self.heading_est = self.heading + self.heading_bias  # initial compass miscalibration

        if self.detection_circle is not None:
            self.detection_circle.resetTransform()
            self.detection_circle.translate(x, y, 0)

        # Create circle only once if view is provided
        elif view is not None:
            self.detection_circle = create_circle(radius=10, x=x, y=y, z=0, color=(1,0,0,0.3))
            view.addItem(self.detection_circle)


    def move(self, dx, dy):
        # True Position
        self.position[0] += dx
        self.position[1] += dy
        self.mesh_item.translate(dx, dy, 0)


        # True Steps
        step_angle = np.arctan2(dy, dx)
        step_dist = np.hypot(dx, dy)

        heading_noise = np.random.normal(0.0, self.heading_noise_std)
        stride_scale = np.random.normal(1.0, self.stride_noise_std)

        stride_scale = np.random.normal(1.0, self.stride_noise_std)
        step_dist_est = step_dist * stride_scale

        # Accumulate the error (History)
        self.heading_est += self.heading_bias + heading_noise 

        # Apply history to current step
        perceived_heading = step_angle + self.heading_est
        
        # Old Method 
        #perceived_heading = step_angle + self.heading_bias + self.heading_est

        #Count total steps 
        self.distance_since_last_scan += step_dist
        
        self.perceived_position[0] += step_dist_est * np.cos(perceived_heading)
        self.perceived_position[1] += step_dist_est * np.sin(perceived_heading)
        self.perceived_position[2] = self.position[2]

        self.px, self.py, self.pz = self.perceived_position

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
    

    def scan_sun(self):
        """
        Simulates the 'Stop and Scan' behavior.
        The ant looks at the polarization pattern (Sun) and resets its
        internal heading estimate to match the Truth (plus some sensor noise).
        """
        from geometry import normalize_angle
        
        # 1. Get True Heading (The Absolute Reference)
        # In a warehouse, this is the angle of ceiling lights.
        true_heading = self.heading  # This is the physics truth

        # RESET the distance counter
        self.distance_since_last_scan = 0.0
        
        # 2. Add Sensor Noise
        # Even the sun compass isn't perfect (e.g., 1 degree error)
        # We use a small noise, distinct from the walking drift.
        sun_sensor_noise = np.random.normal(0.0, np.deg2rad(0.5))
        
        measured_heading = true_heading + sun_sensor_noise
        
        # 3. RESET the internal estimate
        # The ant overwrites its bad dead-reckoning with the fresh sun reading.
        # We effectively delete the accumulated 'heading_est' error.
        
        # Current logic: perceived = true + error
        # Therefore: error = perceived - true
        # We want to force perceived to be close to true.
        
        # Re-align the estimated heading to the measured heading
        self.heading_est = measured_heading - true_heading 
        
        # Note: In your specific move() implementation, 'heading_est' *IS* the accumulated error.
        # So setting it to 0.0 (plus sensor noise) snaps the ghost ant back to parallel.
        self.heading_est = sun_sensor_noise
    

    