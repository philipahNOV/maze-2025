import testing.yolov1.hsv2 as tracking
import time
import cv2
import positionController
import arduino_connection
import optuna
import numpy as np

def tune_pid_with_optuna(controller: positionController.Controller, ref_point, n_trials=15):
    def objective(trial):
        # Suggest gains
        controller.kp_x = trial.suggest_float("kp_x", 1e-6, 1e-2, log=True)
        controller.kd_x = trial.suggest_float("kd_x", 1e-6, 1e-2, log=True)
        controller.ki_x = trial.suggest_float("ki_x", 0.0, 1e-4, log=False)
        controller.kp_y = trial.suggest_float("kp_y", 1e-6, 1e-3, log=True)
        controller.kd_y = trial.suggest_float("kd_y", 1e-6, 1e-3, log=True)
        controller.ki_y = trial.suggest_float("ki_y", 0.0, 1e-4, log=False)

        # Reset controller state
        controller.e_x_int = 0
        controller.e_y_int = 0
        controller.prevError = None
        controller.prevTime = None
        controller.prevVelError = (0, 0)

        total_error = 0
        start_time = time.time()
        timeout = 6  # seconds per trial

        while time.time() - start_time < timeout:
            controller.posControl(ref_point)
            pos = controller.tracker.get_position()
            if pos:
                error = np.linalg.norm(np.array(ref_point) - np.array(pos))
                total_error += error
                if error < controller.pos_tol:
                    break
            time.sleep(0.05)

        return total_error

    study = optuna.create_study(direction="minimize")
    study.optimize(objective, n_trials=n_trials)

    print("[OPTUNA] Best parameters found:")
    for k, v in study.best_params.items():
        print(f"  {k}: {v}")

    best = study.best_params
    controller.kp_x = best["kp_x"]
    controller.kd_x = best["kd_x"]
    controller.ki_x = best["ki_x"]
    controller.kp_y = best["kp_y"]
    controller.kd_y = best["kd_y"]
    controller.ki_y = best["ki_y"]

    return study

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

    if mode == "2":
        ref_point = (770, 330)
        tune_pid_with_optuna(controller, ref_point)
        return

    print("[INFO] Tracking started. Press 'q' to quit.")

    try:
        start_time = time.time()
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            for label, data in tracker.tracked_objects.items():
                pos = data["position"]
                if pos:
                    color = (0, 255, 0) if label == "ball" else (0, 0, 255)
                    cv2.circle(frame, pos, 8, color, -1)
                    cv2.circle(frame, (770, 330), 5, (0, 0, 255), -1)
                    cv2.circle(frame, (770, 330), 5, (0, 0, 255), -1)
                    cv2.putText(frame, label, (pos[0]+10, pos[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            ball_pos = tracker.get_position()
            cv2.imshow("Ball & Marker Tracking", frame)
            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            if time.time() - start_time < 30:
                controller.posControl((770, 330))
            else:
                controller.posControl((770, 330))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Tracker stopped.")

if __name__ == "__main__":
    main()
