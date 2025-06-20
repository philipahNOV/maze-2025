import time
import cv2
from ZED.tracker import BallTracker
from ZED.camera import ZEDCamera

INITIAL_FRAME_WINDOW = "Initial Frame"

def run_tracker(camera):
    try:
        tracker = BallTracker(model_path="testing/yolov1/best.pt")
        tracker.start()
        print("[INFO] Waiting for YOLO initialization...")
        while not tracker.initialized:
            time.sleep(0.1)
    except Exception as e:
        print(f"[ERROR] Failed to start tracker: {e}")
        return

    print("[INFO] Tracking started. Press 'q' to quit.")
    try:
        while True:
            frame = camera.grab_frame()
            if frame is None:
                print("Failed to grab frame from ZED camera.")
                break

            tracker.process_frame(frame)

            for label, data in tracker.tracked_objects.items():
                pos = data["position"]
                if pos:
                    color = (0, 255, 0) if label == "ball" else (0, 0, 255)
                    cv2.circle(frame, (int(pos[0]), int(pos[1])), 8, color, -1)
                    cv2.putText(frame, label, (int(pos[0]) + 10, int(pos[1])),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            orientation = camera.get_orientation()
            if orientation:
                text = f"Orientation: [{orientation[0]:.2f}, {orientation[1]:.2f}]"
                cv2.putText(frame, text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow("Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"[ERROR] during tracking: {e}")
    finally:
        if 'tracker' in locals():
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
    try:
        camera = ZEDCamera()
        camera.init_camera()
        print("[INFO] ZED camera initialized.")
        grab = camera.grab_frame()
        if grab is None:
            print("[ERROR] Failed to grab initial frame from ZED camera.")
            return
        cv2.imshow(INITIAL_FRAME_WINDOW, grab)
        cv2.imshow("Initial Frame", grab)
        key = cv2.waitKey(0)
        if key == ord('q'):
            cv2.destroyAllWindows()
            camera.close()
            print("[INFO] Camera closed.")
            return

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
    print("[INFO] Camera closed.")

if __name__ == "__main__":
    main()
