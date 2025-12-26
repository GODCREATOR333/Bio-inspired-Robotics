import numpy as np
from stl import mesh
import pyqtgraph.opengl as gl
from geometry import create_circle
from Extended_kalman_filter import EKF


class Agent_Model:
    def __init__(self, stl_path,agent_cfg, scale=0.01):
        self.heading = 0.0        # true heading
        self.heading_est = 0.0    # perceived heading
        self.agent_speed = agent_cfg.agent_speed
        self.heading_bias_mean   = agent_cfg.heading_bias_mean
        self.heading_bias_std   = agent_cfg.heading_bias_std
        self.heading_noise_std   = agent_cfg.heading_noise_std
        self.stride_noise_std   = agent_cfg.stride_noise_std
        self.distance_since_last_scan = 0.0
        self.scan_threshold = agent_cfg.scan_interval

        self.scale_factor = scale

        self.ant_detection_range: float = 10.0
        self.ant_detection_circle = None

        # Estimator params
        self.sun_sensor_std = agent_cfg.sun_sensor_std
        self.ekf_q_scale = agent_cfg.ekf_q_scale

        self.scale_factor = scale

       
        self.raw_mesh = mesh.Mesh.from_file(stl_path)

        
        self.vertices = np.array(self.raw_mesh.vectors.reshape(-1, 3), dtype=np.float32)
        center = self.vertices.mean(axis=0)
        self.vertices -= center  # center the mesh

        
        self.faces = np.arange(len(self.vertices)).reshape(-1, 3)

        
        self.position = np.array([0.0, 0.0, 0.0], dtype=float)  # true world position
        self.perceived_position = self.position.copy() #Perceived Position of the ant
        self.rotation = 0.0  # degrees


        # Init EKF State vector and P-Matrix
        q0 = [0.0, 0.0, 0.0]
        P0 = np.eye(3) * 1e-9  # High confidence but not 0 to keep math stable
        Q = np.diag([self.stride_noise_std**2, self.heading_noise_std**2]) * self.ekf_q_scale
        self.ekf = EKF(q0, P0, Q)   # Process noise Q matrix



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
        resample_bias=False,

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

        # REBUILD THE EKF Q-MATRIX
        # This is where the 'Pessimism/Optimism' scale is applied
        new_Q = np.diag([
            self.stride_noise_std**2, 
            self.heading_noise_std**2
        ]) * self.ekf_q_scale
        
        self.ekf.Q = new_Q

    # Place it in the world
    def spawn(self, x=0, y=0, z=0, view=None):
        # Reset transforms and scale (keeps consistent size)
        self.mesh_item.resetTransform()
        self.mesh_item.scale(self.scale_factor, self.scale_factor, self.scale_factor)

        # Set true position and move mesh
        self.position[:] = (x, y, z)
        self.perceived_position = np.array([x, y, z], dtype=float)
        self.mesh_item.translate(x, y, z)
        self.px, self.py, self.pz = self.perceived_position

        # Odometry init
        self.heading = 0.0

        
        # Reset EKF
        self.ekf.q = np.array([x, y, 0.0]).reshape(3, 1)
        self.ekf.P = np.eye(3) * 1e-9

        self.perceived_position = np.array([x, y, z], dtype=float)
        self.px, self.py, self.pz = self.perceived_position

        self.heading_est = 0.0 
        self.distance_since_last_scan = 0.0
        
        if self.ant_detection_circle is not None:
            self.ant_detection_circle.resetTransform()
            self.ant_detection_circle.translate(x, y, 0)

        # Create circle only once if view is provided
        elif view is not None:
            self.ant_detection_circle = create_circle(radius=self.ant_detection_range, x=x, y=y, z=0, color=(1,0,0,0.3))
            view.addItem(self.ant_detection_circle)


    def move(self, dx, dy):
        # --- 1. PHYSICAL REALITY (Ground Truth) ---
        self.position[0] += dx
        self.position[1] += dy
        self.mesh_item.translate(dx, dy, 0)
        
        # Update True heading logic
        true_heading_new = np.arctan2(dy, dx)
        d_theta_true = true_heading_new - self.heading
        self.heading = true_heading_new

        # --- 2. THE SENSORY REPORT (Odometry) ---
        dist_true = np.hypot(dx, dy)
        stride_noise = np.random.normal(1.0, self.stride_noise_std)
        heading_noise = np.random.normal(0.0, self.heading_noise_std)
        
        dist_felt = dist_true * stride_noise
        d_theta_felt = d_theta_true + self.heading_bias + heading_noise

        # --- 3. THE EKF PREDICTION ---
        self.ekf.predict(dist_felt, d_theta_felt)

        # --- 4. UPDATE TRACKING VARIABLES ---
        # We pull the Mean from the EKF and update px, py, pz 
        # so the TrailManager sees the new belief.
        self.perceived_position[0] = self.ekf.q[0, 0]
        self.perceived_position[1] = self.ekf.q[1, 0]
        self.perceived_position[2] = self.position[2] # z = 0

        self.px, self.py, self.pz = self.perceived_position
        
        # Distance tracker for the next sun scan
        self.distance_since_last_scan += dist_true

        if self.ant_detection_circle is not None:
            self.ant_detection_circle.translate(dx, dy, 0)

    # Rotate around Z-axis
    def rotate(self, angle_deg):
        self.rotation += angle_deg
        self.mesh_item.rotate(angle_deg, 0, 0, 1)  # Z-axis
    
    def get_true_pos(self):
        return self.position.copy()

    def get_sim_pos(self):
        return self.perceived_position.copy()
    

    def scan_sun(self):
        # 1. THE SENSOR READING (Reality + Noise)
        # The 'True' heading is what the sun is actually relative to.
        true_heading = self.heading 
        
        # We define how accurate the ant's eyes are (The W matrix)
        # Original code used 0.5 degrees. 
        sun_noise = np.random.normal(0.0, self.sun_sensor_std)
        
        measured_heading = true_heading + sun_noise

        # 2. THE EKF UPDATE (The Brain Fix)
        # We tell the EKF brain the absolute measurement and our sensor trust (R)
        self.ekf.update(measured_heading, self.sun_sensor_std**2)

        # 3. SYNC TRACKING VARIABLES
        # Because the EKF may have nudged the x and y during the heading fix,
        # we update our perceived_position so the Homing Policy stays in sync.
        self.perceived_position[0] = self.ekf.q[0, 0]
        self.perceived_position[1] = self.ekf.q[1, 0]
        self.px, self.py, self.pz = self.perceived_position

        # Reset the travel counter so the FSM knows when to scan again
        self.distance_since_last_scan = 0.0
        


    def print_agent_params(self):
        print("=" * 50)
        print("ANT PARAMETERS")
        print("=" * 50)
        print(f"heading:                    {self.heading}")
        print(f"heading_est:                {self.heading_est}")
        print(f"heading_bias_mean:          {self.heading_bias_mean}")
        print(f"agent_speed:                {self.agent_speed}")
        print(f"heading_bias_std:           {self.heading_bias_std}")
        print(f"heading_noise_std:          {self.heading_noise_std}")
        print(f"stride_noise_std:           {self.stride_noise_std}")
        print(f"distance_since_last_scan:   {self.distance_since_last_scan}")
        print(f"scan_threshold:             {self.scan_threshold}")
        print(f"scale_factor:               {self.scale_factor}")
        print(f"ant_detection_range:        {self.ant_detection_range}")
        print("\nESTIMATOR PARAMETERS")
        print("-" * 50)
        print(f"sun_sensor_std:             {self.sun_sensor_std}")
        print(f"ekf_q_scale:                {self.ekf_q_scale}")
        print(f"scale_factor:               {self.scale_factor}")
        print("=" * 50)
    