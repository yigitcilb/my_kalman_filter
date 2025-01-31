import numpy as np

class KalmanFilter:
    def __init__(self, Q: float, R: float, delta_t: float):
        #Q (Process Noise Covariance Matrix), R (Measurement Noise Covariance Matrix)
        self.Q = np.array([[Q, 0, 0],
                           [0, Q ,0],
                           [0, 0, Q]])
        self.R = np.eye(3) * R
        self.delta_t = delta_t
        self.F = np.array([[1, delta_t, 0],
                          [0, 1, delta_t],
                          [0, 0, 1]])
        self.P = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
        self.H = np.array([[1, 0, 0]])
        self.z_hat = np.array([0, 0, 0])
        self.speed = 0
        self.speed_hat = 0
        self.x_hat = np.array([0, 0, 0])  

    def update_filter(self, imu: float)  -> float:
        #input the measured imu data in 1D and get output the estimated one.
        self.speed = self.speed_hat + imu * self.delta_t
        ghost = self.z_hat[0]
        z = self.z_hat[0] + self.speed * self.delta_t

        z = np.array([z, 0, 0])
        self.x_hat_minus = np.dot(self.F, self.x_hat)
        self.P_minus = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        self.v_k = z - np.dot(self.H, self.x_hat_minus)
        self.K = np.dot(np.dot(self.P_minus, self.H.T), np.linalg.inv(np.dot(np.dot(self.H, self.P_minus), self.H.T) + self.R))
        self.P = np.dot(np.eye(3) - np.dot(self.K, self.H), self.P_minus)
        self.x_hat = self.x_hat_minus + np.dot(self.K, self.v_k)

        self.z_hat = np.dot(self.H, self.x_hat)
        self.speed_hat = (self.z_hat[0]- ghost)/self.delta_t

        return self.z_hat[0]