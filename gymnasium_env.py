import gymnasium as gym
from gymnasium import spaces
import numpy as np

class AntNavigationEnv(gym.Env):
    """
    Gymnasium wrapper for ant foraging task with EKF-based path integration.
    
    Task: Start at home -> Search for food -> Return home with food
    
    Observation: [dist_to_home, bearing_to_home, pos_uncertainty, heading_uncertainty, has_food]
    Action: steering angle [-1, 1] (scaled to turn capability)
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 10}

    def __init__(self, agent_instance, env_instance, max_steps=1000):
        super().__init__()
        
        # Store references to existing simulation objects
        self.agent = agent_instance
        self.world = env_instance
        self.max_steps = max_steps
        
        # Internal state tracking (replaces FSM)
        self.has_food = False
        self.step_count = 0
        
        # Define ACTION SPACE: continuous steering [-1.0 to 1.0]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

        # Define OBSERVATION SPACE: [dist, bearing, trace_pos, var_head, has_food]
        self.observation_space = spaces.Box(
            low=np.array([0.0, -np.pi, 0.0, 0.0, 0.0], dtype=np.float32),
            high=np.array([2000.0, np.pi, 1e6, 100.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

    def _get_obs(self):
        """Extract observation vector from EKF state."""
        q_est = self.agent.ekf.q  # [x, y, theta] - 3x1 matrix
        P = self.agent.ekf.P      # 3x3 covariance matrix
        
        # 1. Distance to home (nest at origin)
        dist = np.linalg.norm(q_est[:2])
        
        # 2. Bearing to home (angle from current heading to home direction)
        bearing = np.arctan2(-q_est[1, 0], -q_est[0, 0]) - q_est[2, 0]
        bearing = np.arctan2(np.sin(bearing), np.cos(bearing))  # Wrap to [-pi, pi]
        
        # 3. Position uncertainty (trace of position covariance)
        trace_pos = np.trace(P[:2, :2])
        
        # 4. Heading uncertainty (variance of heading)
        var_head = P[2, 2]
        
        # 5. Food status (our internal tracking)
        has_food_flag = 1.0 if self.has_food else 0.0
        
        return np.array([dist, bearing, trace_pos, var_head, has_food_flag], dtype=np.float32)

    def reset(self, seed=None, options=None):
        """Reset environment to initial state."""
        # Handle seeding for reproducibility
        super().reset(seed=seed)

        # Reset world (randomize food positions)
        self.world.reset()
        self.world.build()

        # Reset agent at nest (home)
        self.agent.spawn(x=0, y=0, z=2.5, view=None)
        
        # Set random initial heading
        start_heading = self.np_random.uniform(0, 2 * np.pi)
        self.agent.heading = start_heading
        self.agent.ekf.q[2, 0] = start_heading
        
        # Reset internal state
        self.has_food = False
        self.step_count = 0

        # Get initial observation
        observation = self._get_obs()
        info = {
            "has_food": False,
            "food_found_step": None,
            "episode_phase": "searching"
        }

        return observation, info

    def step(self, action):
        """Execute one step in the environment."""
        self.step_count += 1
        
        # 1. Map action to steering angle
        max_turn_rad = np.radians(30)  # 30 degrees max turn per step
        steering_turn = action[0] * max_turn_rad
        
        # 2. Calculate movement deltas
        step_speed = self.agent.agent_speed
        new_heading = self.agent.heading + steering_turn
        dx = step_speed * np.cos(new_heading)
        dy = step_speed * np.sin(new_heading)
        
        # 3. Execute movement (updates position, heading, and EKF)
        self.agent.move(dx, dy)

        # 4. Check for food collision (only if not already carrying food)
        if not self.has_food:
            found, items_to_remove = self.world.check_food_collision(self.agent.get_true_pos())
            if found:
                self.has_food = True
                # Note: items_to_remove ignored in headless mode

        # 5. Get new observation
        observation = self._get_obs()
        dist_to_home = observation[0]
        uncertainty = observation[2]

        # 6. Calculate reward
        reward = -0.1  # Small time penalty to encourage efficiency
        
        # Phase-dependent rewards
        if not self.has_food:
            # SEARCH PHASE: Encourage exploration
            # Small penalty for being too close to home (encourage leaving nest)
            if dist_to_home < 50:
                reward -= 0.01
        else:
            # RETURN PHASE: Encourage returning home efficiently
            reward -= 0.01 * dist_to_home  # Penalty proportional to distance
            reward -= 0.001 * uncertainty  # Penalty for being uncertain
        
        # 7. Check termination conditions
        terminated = False
        truncated = False
        
        # Get true distance to home
        true_dist = np.linalg.norm(self.agent.position[:2])
        ant_radius = 10.0
        home_radius = self.world.cfg.home_detection_radius
        combined_threshold = ant_radius + home_radius

        # SUCCESS: Returned home with food
        if self.has_food and true_dist < combined_threshold:
            reward += 1000.0  # Large reward for completing the task
            terminated = True

        # FAILURE: Timeout
        if self.step_count >= self.max_steps:
            reward -= 100.0
            truncated = True

        # FAILURE: Went too far from home (safety boundary)
        if true_dist > 1000.0:
            reward -= 200.0
            truncated = True

        # 8. Metadata
        info = {
            "true_pos": self.agent.position[:2].copy(),
            "belief_pos": self.agent.ekf.q[:2, 0].copy(),
            "has_food": self.has_food,
            "is_success": terminated and self.has_food,
            "true_dist": true_dist,
            "belief_dist": dist_to_home,
            "step_count": self.step_count,
            "episode_phase": "returning" if self.has_food else "searching"
        }

        return observation, reward, terminated, truncated, info

    def close(self):
        """Clean up resources."""
        pass
