from filterpy.kalman import KalmanFilter
import numpy as np


def create_kalman_filter():
    kf = KalmanFilter(dim_x=4, dim_z=2)
    
    dt = 1

    # State transition matrix
    kf.F = np.array([[1, 0, dt, 0],
                     [0, 1, 0, dt],
                     [0, 0, 1, 0 ],
                     [0, 0, 0, 1 ]])
    
    # Measurement function: we only observe x and y
    kf.H = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0]])

    # Initial state uncertainty
    kf.P = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0 ],
                     [0, 0, 0, 1 ]])
                     
    # Measurement noise
    kf.R = np.array([[1, 0],
                     [0, 1]]) 
    
    # Process noise
    kf.Q = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0 ],
                     [0, 0, 0, 1 ]])
    
    return kf