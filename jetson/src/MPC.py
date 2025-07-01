import cvxpy as cp
import numpy as np
import testing.yolov1.hsv3 as tracking
import arduino_connection_test
import time
from scipy.signal import cont2discrete

class MPC:
    def __init__(self, arduinoThread: arduino_connection_test.ArduinoConnection, tracker: tracking.BallTracker):

        g = 9.81
        a = 5 * g / 7  # ≈ 7.007

        A = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0]
        ])

        B = np.array([
            [0, 0],
            [a, 0],
            [0, 0],
            [0, a]
        ])

        dt = 0.02  # sample time (same as your loop)

        self.Ad, self.Bd, _, _, _ = cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), dt)

        self.N = 20  # horizon
        self.x = cp.Variable((4, self.N + 1))
        self.u = cp.Variable((2, self.N))

        self.Q = np.diag([1, 0.1, 1, 0.1])
        self.R = np.diag([0.01, 0.01])

    def step(self, state, state_ref):
        x_init = state
        x_ref = state_ref

        cost = 0
        constraints = [self.x[:, 0] == x_init]

        for k in range(self.N):
            cost += cp.quad_form(self.x[:, k] - x_ref, self.Q) + cp.quad_form(u[:, k], R)
            constraints += [self.x[:, k + 1] == self.Ad @ self.x[:, k] + self.Bd @ self.u[:, k]]
            constraints += [cp.abs(self.u[:, k]) <= np.deg2rad(1.8)]  # angle limits

        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        return self.u[:, 0]