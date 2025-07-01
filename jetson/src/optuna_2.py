import random
import time
import numpy as np

def evaluate_controller(controller, waypoints, start=(604, 950), timeout=12):
    from path_following import PathFollower

    path_array = [(x, y) for y, x in waypoints]
    path_follower = PathFollower(path_array, controller)

    start_time = time.time()
    reached = False
    last_pos = None

    while time.time() - start_time < timeout:
        pos = controller.tracker.get_position()
        if pos:
            last_pos = pos
            path_follower.follow_path(pos)

        dist = np.linalg.norm(np.array(path_array[-1]) - np.array(last_pos or (0, 0)))
        if dist < controller.pos_tol:
            reached = True
            break

        time.sleep(0.01)

    duration = time.time() - start_time
    penalty = 0 if reached else 1000
    error = np.linalg.norm(np.array(path_array[-1]) - np.array(last_pos or (0, 0)))
    return duration + error + penalty


def pid_tuning_dual_axis(controller, waypoints, n_trials=50, start=(604,950)):
    results = []
    best_score = float('inf')
    best_params = None

    for i in range(n_trials):
        if i < 15 or not results:
            kp_x = random.uniform(0.00001, 0.00005)
            kd_x = random.uniform(0.00005, 0.00015)
            ki_x = random.uniform(0.0, 0.0004)

            kp_y = random.uniform(0.00001, 0.00005)
            kd_y = random.uniform(0.00005, 0.00015)
            ki_y = random.uniform(0.0, 0.0004)
        else:
            base = min(results, key=lambda x: x[6])
            def clipn(mu, factor=0.4, lo=0.000001, hi=0.001):
                return np.clip(np.random.normal(mu, factor * mu), lo, hi)

            kp_x, kd_x, ki_x = clipn(base[0]), clipn(base[1]), clipn(base[2])
            kp_y, kd_y, ki_y = clipn(base[3]), clipn(base[4]), clipn(base[5])

        controller.set_pid_parameters([
            "pass", "pass", kp_x, kp_y, kd_x, kd_y, ki_x, ki_y, "pass", "pass"
        ])

        score = evaluate_controller(controller, waypoints, start=start)
        results.append((kp_x, kd_x, ki_x, kp_y, kd_y, ki_y, score))

        if score < best_score:
            best_score = score
            best_params = (kp_x, kd_x, ki_x, kp_y, kd_y, ki_y)
            print(f"[{i}] New Best")
        print(f"[{i}] kp_x={kp_x:.6f} kd_x={kd_x:.6f} ki_x={ki_x:.6f} | kp_y={kp_y:.6f} kd_y={kd_y:.6f} ki_y={ki_y:.6f} → Score: {score:.2f}")

    print("\n--- Best Dual-Axis PID ---")
    names = ["kp_x", "kd_x", "ki_x", "kp_y", "kd_y", "ki_y"]
    for n, v in zip(names, best_params):
        print(f"{n} = {v:.6f}")
    print(f"Best Score: {best_score:.2f}")
