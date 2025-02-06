import numpy as np

class KalmanFilter:
    def __init__(self, Q: float, R: float, delta_t: float):
        # Q: Process Noise Covariance, R: Measurement Noise Covariance
        self.Q = np.array([[Q, 0, 0],
                           [0, Q, 0],
                           [0, 0, Q]])
        self.B = np.array([[(delta_t * delta_t)/2, delta_t, 1]])
        self.R = np.array([[R]])  
        self.delta_t = delta_t
        self.F = np.array([[1, delta_t, 0],
                           [0, 1, delta_t],
                           [0, 0, 1]])
        self.P = np.eye(3)
        self.H = np.array([[0,0,1]])  
        self.fin = np.array([[1,0,0]])

        self.z_hat = np.array([[0]])  
        self.x_hat = np.array([[0], [0], [0]]) 
        self.speed_hat = 0

    def update_filter(self, imu: float) -> float:
        self.x_hat_minus = np.dot(self.F, self.x_hat) + np.dot(self.B, imu)
        self.P_minus = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        S = np.dot(np.dot(self.H, self.P_minus), self.H.T) + self.R  
        self.K = np.dot(np.dot(self.P_minus, self.H.T), np.linalg.inv(S)) 
        z = imu
        z = np.array([0, 0, z]) 
        self.v_k = z - np.dot(self.H, self.x_hat_minus)
        self.x_hat = self.x_hat_minus + np.dot(self.K, self.v_k)  
        self.P = np.dot(np.eye(3) - np.dot(self.K, self.H), self.P_minus)  
        self.z_hat = np.dot(self.H, self.x_hat)  
        return np.dot(self.fin, self.x_hat)
