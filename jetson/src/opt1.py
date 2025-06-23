import testing.yolov1.hsv2 as tracking
import time
import cv2
import positionController
import arduino_connection
import optuna
import numpy as np
import threading

def tune_pid_with_optuna(controller: positionController.Controller, ref_point, n_trials=15):
    def objective(trial):
        # PID gain search space (narrowed around known good values)
        controller.kp_x = trial.suggest_float("kp_x", 5e-5, 2e-4, log=True)
        controller.kd_x = trial.suggest_float("kd_x", 3e-5, 2e-4, log=True)
        controller.ki_x = trial.suggest_float("ki_x", 0.0, 5e-5)

        controller.kp_y = trial.suggest_float("kp_y", 5e-5, 2e-4, log=True)
        controller.kd_y = trial.suggest_float("kd_y", 3e-5, 2e-4, log=True)
        controller.ki_y = trial.suggest_float("ki_y", 0.0, 5e-5)

        # Reset controller state
        controller.e_x_int = 0
        controller.e_y_int = 0
        controller.prevError = None
        controller.prevTime = None
        controller.prevVelError = (0, 0)

        total_error = 0
        overshoot_count = 0
        max_overshoot = 0
        control_smoothness = 0

        prev_tilt = (0.0, 0.0)
        start_time = time.time()
        timeout = 6  # seconds per trial
        dt = 0.05
        steps = 0

        while time.time() - start_time < timeout:
            controller.posControl(ref_point)
            pos = controller.tracker.get_position()

            if pos:
                error_vec = np.array(ref_point) - np.array(pos)
                error = np.linalg.norm(error_vec)
                total_error += error
                steps += 1

                # Overshoot penalty if error was previously low and now grows
                if error < controller.pos_tol and error > 2:
                    overshoot_count += 1
                    max_overshoot = max(max_overshoot, error)

            try:
                tilt = controller.get_current_tilt()  # Tuple: (tilt_x, tilt_y)
            except:
                tilt = prev_tilt  # Fallback if method not defined

            tilt_delta = np.linalg.norm(np.array(tilt) - np.array(prev_tilt))
            control_smoothness += tilt_delta
            prev_tilt = tilt

            time.sleep(dt)

        if steps == 0:
            return float('inf')

        avg_error = total_error / steps
        avg_smoothness = control_smoothness / steps
        overshoot_penalty = overshoot_count * 10 + max_overshoot

        # Final objective
        return avg_error + 0.5 * avg_smoothness + overshoot_penalty

    study = optuna.create_study(direction="minimize")
    study.optimize(objective, n_trials=n_trials)

    print("[OPTUNA] Best parameters found:")
    for k, v in study.best_params.items():
        print(f"  {k}: {v:.8f}")

    best = study.best_params
    controller.kp_x = best["kp_x"]
    controller.kd_x = best["kd_x"]
    controller.ki_x = best["ki_x"]
    controller.kp_y = best["kp_y"]
    controller.kd_y = best["kd_y"]
    controller.ki_y = best["ki_y"]

    return study

def show_camera(tracker, stop_event):
    """ Continuously show camera frame until stop_event is set. """
    while not stop_event.is_set():
        frame = tracker.frame
        if frame is not None:
            for label, data in tracker.tracked_objects.items():
                pos = data["position"]
                if pos:
                    color = (0, 255, 0) if label == "ball" else (0, 0, 255)
                    cv2.circle(frame, pos, 8, color, -1)
                    cv2.circle(frame, (770, 330), 5, (0, 0, 255), -1)
                    cv2.putText(frame, label, (pos[0] + 10, pos[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.imshow("Ball & Marker Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break
        time.sleep(0.03)  # ~30 FPS


def main():
    mode = input("Choose mode: [1] Run Controller  [2] Tune PID with Optuna\n> ").strip()

    def initialize_component(component, name, retries=5, delay=2):
        for attempt in range(retries):
            try:
                comp_instance = component()
                print(f"{name} initialized on attempt {attempt + 1}")
                return comp_instance
            except Exception as e:
                print(f"Failed to initialize {name} on attempt {attempt + 1}: {e}")
                time.sleep(delay)
        raise Exception(f"Failed to initialize {name} after {retries} attempts")

    try:
        arduino_thread = initialize_component(arduino_connection.ArduinoConnection, "ArduinoConnection")
        time.sleep(10)
    except Exception as e:
        print(e)
        exit(1)

    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
    tracker.start()

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    controller = positionController.Controller(arduino_thread, tracker)
    time.sleep(5)
    controller.horizontal()

    # Start camera display thread
    stop_event = threading.Event()
    camera_thread = threading.Thread(target=show_camera, args=(tracker, stop_event))
    camera_thread.start()

    if mode == "2":
        ref_point = (770, 330)
        try:
            tune_pid_with_optuna(controller, ref_point)
        finally:
            stop_event.set()
            camera_thread.join()
            tracker.stop()
            cv2.destroyAllWindows()
        return

    print("[INFO] Tracking started. Press 'q' to quit.")
    try:
        start_time = time.time()
        while True:
            ball_pos = tracker.get_position()
            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            if time.time() - start_time < 30:
                controller.posControl((770, 330))
            else:
                controller.posControl((770, 330))

            if stop_event.is_set():
                break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        stop_event.set()
        camera_thread.join()
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Tracker stopped.")

if __name__ == "__main__":
    main()
