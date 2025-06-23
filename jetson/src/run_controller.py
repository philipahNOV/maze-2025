import testing.yolov1.hsv2 as tracking
import time
import cv2
import positionController
import arduino_connection

def main():

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

    print("[INFO] Tracking started. Press 'q' to quit.")

    controller = positionController.Controller(arduino_thread, tracker)
    time.sleep(5)
    controller.horizontal()
    
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
                    cv2.circle(frame, (770-150, 330-150), 5, (0, 0, 255), -1)
                    cv2.circle(frame, (770+150, 330+150), 5, (0, 0, 255), -1)
                    cv2.putText(frame, label, (pos[0]+10, pos[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            ball_pos = tracker.get_position()
            cv2.imshow("Ball & Marker Tracking", frame)
            if not ball_pos:
                print("No ball found (run_controller)")
                continue
            
            #print(f"Ball position: {ball_pos}", end="\r")

            #--- Control ---

            if time.time() - start_time < 30:
                controller.posControl((770-150, 330-150))
            else:
                controller.posControl((770+150, 330+150))

            #---------------


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
