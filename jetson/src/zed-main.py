import time
import cv2
from ZED.tracker import BallTracker
from ZED.camera import ZEDCamera

def run_tracker():
    tracker = BallTracker(model_path="testing/yolov1/best.pt")
    tracker.start()
    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")
    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            for label, data in tracker.tracked_objects.items():
                pos = data["position"]
                if pos:
                    color = (0, 255, 0) if label == "ball" else (0, 0, 255)
                    cv2.circle(frame, pos, 8, color, -1)
                    cv2.putText(frame, label, (pos[0] + 10, pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            orientation = tracker.get_orientation()
            if orientation:
                text = f"Orientation: [{orientation[0]:.2f}, {orientation[1]:.2f}]"
                cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow("Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        tracker.stop()
        cv2.destroyAllWindows()

def show_menu():
    print("""
    === ZED SYSTEM MENU ===
    1. Track Ball
    2. Run PID (TBD)
    3. Calibrate Camera (TBD)
    0. Exit
    """)

def main():
    camera = ZEDCamera()
    try:
        camera.init_camera()
        print("[INFO] ZED camera initialized.")
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        return 

    while True:
        show_menu()
        choice = input("Select option: ")
        if choice == "1":
            run_tracker(camera)
        elif choice == "2":
            print("[TODO] PID control module")
        elif choice == "3":
            print("[TODO] Camera calibration")
        elif choice == "0":
            break
        else:
            print("Invalid choice.")

    camera.close()
    print("[INFO] Camera closed. Goodbye.")


if __name__ == "__main__":
    main()
