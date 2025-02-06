import numpy as np

class KalmanFilter:
    def __init__(self, Q: float, R: float, delta_t: float):
        # Q: Process Noise Covariance, R: Measurement Noise Covariance
        self.Q = np.array([[Q, 0, 0],
                           [0, Q, 0],
                           [0, 0, Q]])
        self.B = np.array([[(delta_t**2)/2], [delta_t], [1]])
        self.R = np.array([[R, 0, 0],
                           [0, R, 0],
                           [0, 0, R]]) 
        self.delta_t = delta_t
        self.F = np.array([[1, delta_t, 0],
                           [0, 1, delta_t],
                           [0, 0, 1]])
        self.P = np.eye(3) * 1000
        self.H = np.array([[1,delta_t,0], [0,1,delta_t], [0,0,1]])

        self.fin = np.array([[1], [0], [0]]) 
        self.pos_hat = 0

        self.z_hat = np.array([[0], [0],[0]])  
        self.x_hat = np.array([[0], [0], [0]]) 
        self.speed_hat = 0

    def update_filter(self, imu: float) -> float:

        self.x_hat_minus = np.dot(self.F, self.x_hat) + np.dot(self.B, imu)
        self.P_minus = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        S = np.dot(np.dot(self.H, self.P_minus), self.H.T) + self.R  
        self.K = np.dot(np.dot(self.P_minus, self.H.T), np.linalg.inv(S)) 
        z = np.array([[self.pos_hat + imu*(self.delta_t**2)/2],[self.speed_hat + imu* self.delta_t], [imu]])
        self.v_k = z - np.dot(self.H, self.x_hat_minus)
        self.x_hat = self.x_hat_minus + np.dot(self.K, self.v_k) 
        Y = np.eye(3) - np.dot(self.K, self.H)
        self.P = np.dot(np.dot(Y, self.P_minus), Y.T)  + np.dot(np.dot(self.K, self.R), self.K.T)
        self.z_hat = np.dot(self.H, self.x_hat)  
        print(self.x_hat)
        self.speed_hat = self.x_hat[1,0]
        self.pos_hat = self.x_hat[0,0]
        return self.pos_hat
