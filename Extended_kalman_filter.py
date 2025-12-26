import numpy as np

class EKF:
    def __init__(self, q0, P0, Q):
        """
        q0 : Initial state [x, y, theta] (3x1)
        P0 : Initial covariance (3x3)
        Q  : Process noise covariance [dist_noise, angle_noise] (2x2)
        """
        self.q = np.array(q0, dtype=float).reshape(3, 1)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)

    def predict(self, dist, d_theta):
        """
        dist    : Estimated distance moved (from odometry)
        d_theta : Estimated change in heading (from internal compass/gyro)
        """
        # 1. Update the State (The Mean)
        # We use the CURRENT heading to move, then apply the turn
        theta = self.q[2, 0]
        
        self.q[2, 0] += d_theta
        # Normalize
        self.q[2, 0] = np.arctan2(np.sin(self.q[2, 0]), np.cos(self.q[2, 0]))

        new_theta = self.q[2, 0]
        self.q[0, 0] += dist * np.cos(new_theta)
        self.q[1, 0] += dist * np.sin(new_theta)

        # 2. Update the Covariance (The Doubt)
        # We need the Jacobian F_x (Sensitivity of next state to current state)
        # F = [ 1, 0, -dist*sin(theta) ]
        #     [ 0, 1,  dist*cos(theta) ]
        #     [ 0, 0,  1               ]
        F = np.array([
            [1, 0, -dist * np.sin(theta)],
            [0, 1,  dist * np.cos(theta)],
            [0, 0, 1]
        ])

        # We need the Jacobian F_v (Sensitivity of state to input noise)
        # G = [ cos(theta), 0 ]
        #     [ sin(theta), 0 ]
        #     [ 0,          1 ]
        G = np.array([
            [np.cos(theta), 0],
            [np.sin(theta), 0],
            [0, 1]
        ])

        # P = F*P*F' + G*Q*G' 
        # (This is the "Sandwich" that stretches the doubt based on geometry)
        self.P = F @ self.P @ F.T + G @ self.Q @ G.T


    def update(self, z, R):
        """
        z : Absolute heading measurement from the sun sensor (radians)
        R : Measurement noise covariance (Variance of the sun sensor)
        """
        # 1. THE MEASUREMENT LENS (H)
        # We only observe theta (the 3rd element of our state q)
        H = np.array([[0, 0, 1]])

        # 2. THE SURPRISE (Innovation y)
        # y = Measured Angle - Estimated Angle
        # We use atan2(sin, cos) to handle the 360-degree wrap-around correctly
        estimated_theta = self.q[2, 0]
        y = z - estimated_theta
        y = np.arctan2(np.sin(y), np.cos(y)) 

        # 3. THE TRUST FACTOR (Kalman Gain K)
        # S is the 'Total Uncertainty' (Internal Doubt + Sensor Noise)
        S = H @ self.P @ H.T + R
        
        # K = P * H' * inv(S)
        # K tells us how to distribute the heading 'surprise' into x, y, and theta
        K = self.P @ H.T / S

        # 4. THE SNAP (Update State)
        # We nudge the whole state [x, y, theta] using the Surprise and Gain
        self.q = self.q + K * y
        
        # Re-normalize heading after the snap
        self.q[2, 0] = np.arctan2(np.sin(self.q[2, 0]), np.cos(self.q[2, 0]))

        # 5. THE PINCH (Update Covariance)
        # P = (I - KH) * P
        # This is where the doubt shrinks/collapses
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P