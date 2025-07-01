import numpy as np
import matplotlib.pyplot as plt
import MPC

# --- Configuration ---
px_to_m = 0.28 / 1200  # ≈ 0.000233 m/px
dt = 0.015
N_sim = 2000  # total simulation steps
reach_thresh = 0.002  # meters (≈ 8mm) to switch to next waypoint

# Define waypoints in pixels: [x, _, y, _]
waypoints_px = [
    [200, 0, 200, 0],
    [400, 0, 300, 0],
    [600, 0, 200, 0],
    [800, 0, 300, 0],
    [1000, 0, 200, 0],
    [1000, 0, 400, 0],
    [800, 0, 500, 0],
    [600, 0, 400, 0],
    [400, 0, 500, 0],
    [200, 0, 400, 0]
]
waypoints = [np.array(wp) * px_to_m for wp in waypoints_px]
num_waypoints = len(waypoints)

# --- Initialization ---
x = waypoints[0].copy()
trajectory = [x.copy()]
current_wp_index = 1
mpc_controller = MPC.MPC_controller()

# --- Simulation Loop ---
for t in range(N_sim):
    if current_wp_index >= num_waypoints:
        break

    x_ref = waypoints[current_wp_index]

    # Compute control
    u = mpc_controller.step(x, x_ref)
    if u is None:
        print("MPC solve failed at step", t)
        break

    # Apply system dynamics + noise
    noise = np.random.normal(0, 0.002, size=4)
    x = mpc_controller.Ad @ x + mpc_controller.Bd @ u + noise

    trajectory.append(x.copy())

    # Check distance to current waypoint (using only position: x[0], x[2])
    pos_error = np.linalg.norm(x[[0, 2]] - x_ref[[0, 2]])
    if pos_error < reach_thresh:
        print(f"Reached waypoint {current_wp_index} at step {t}")
        current_wp_index += 1

# --- Visualization ---
trajectory = np.array(trajectory)
waypoints_arr = np.array([wp[[0, 2]] for wp in waypoints])  # x and y positions

plt.figure(figsize=(8, 6))
plt.plot(trajectory[:, 0], trajectory[:, 2], 'o-', label='Ball trajectory')
plt.plot(waypoints_arr[:, 0], waypoints_arr[:, 1], 'rx--', label='Waypoints')
plt.plot(trajectory[0, 0], trajectory[0, 2], 'go', label='Start')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Simulated Ball Path Following with MPC')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
