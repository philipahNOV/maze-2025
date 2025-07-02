import cvxpy as cp
import numpy as np
#import testing.yolov1.hsv3 as tracking
#import arduino_connection_test
import time
from scipy.signal import cont2discrete

class MPC_controller:
    def __init__(self):

        g = 9.81
        #a = 5 * g / 7  # ≈ 7.007
        a = 5.5 # from experiments

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

        dt = 0.04  # sample time (same as your loop)

        self.Ad, self.Bd, _, _, _ = cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), dt)

        self.N = 15  # horizon
        self.x = cp.Variable((4, self.N + 1))
        self.u = cp.Variable((2, self.N))

        self.Q = np.diag([1, 0.1, 1, 0.1])*10
        self.R = np.diag([0.01, 0.01])

        
        self.x0 = cp.Parameter(4)
        self.x_ref = cp.Parameter(4)

        #Q_terminal = self.Q * 10

        # Build cost and constraints once
        cost = 0
        constraints = [self.x[:, 0] == self.x0]  # dynamic initial state

        for k in range(self.N):
            cost += cp.quad_form(self.x[:, k] - self.x_ref, self.Q)
            cost += cp.quad_form(self.u[:, k], self.R)
            constraints += [self.x[:, k + 1] == self.Ad @ self.x[:, k] + self.Bd @ self.u[:, k]]
            constraints += [cp.abs(self.u[:, k]) <= 0.0314]  # ~1.8°

        cost += cp.quad_form(self.x[:, self.N+1] - self.x_ref)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def step(self, state, state_ref):
        start_time = time.time()
        self.x0.value = state
        self.x_ref.value = state_ref

        self.problem.solve(solver=cp.OSQP, warm_start=True)
        print(f"Solve time: {(time.time()-start_time)*1000} ms")

        if self.problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            print("[MPC] Solve failed:", self.problem.status)
            return None

        return self.u.value[:, 0]