import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController
import arduino_connection
import lowPassFilter
import path

def main():

    def plot_waypoints(frame, path_obj: path.Path):
        for n in range(path_obj.length):
            cv2.circle(frame, path_obj[n], 5, (0, 0, 255), -1)


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
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)
    path_array = [(50, 50), (150, 50), (250, 50), (350, 50), (450, 50), (450, 150), (450, 250), (450, 350), (450, 450)]
    path_obj = path.Path(path_array=path_array)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")

    controller = positionController.Controller(arduino_thread, tracker)
    time.sleep(1)
    controller.horizontal()
    time.sleep(2)
    
    try:
        start_time = time.time()
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            plot_waypoints(frame, path_obj)

            #for label, data in tracker.tracked_objects.items():
            #    pos = data["position"]
            #    if pos:
            #        color = (0, 255, 0) if label == "ball" else (0, 0, 255)
            #        cv2.circle(frame, pos, 8, color, -1)
            #        cv2.circle(frame, (770-150, 330-150), 5, (0, 0, 255), -1)
            #        cv2.circle(frame, (770+150, 330+150), 5, (0, 0, 255), -1)
            #        cv2.putText(frame, label, (pos[0]+10, pos[1]), 
            #                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)

            cv2.circle(frame, ball_pos, 8, (0, 255, 0), -1)
            #cv2.circle(frame, (770-150, 330-150), 5, (0, 0, 255), -1)
            #cv2.circle(frame, (770+150, 330+150), 5, (0, 0, 255), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow("Ball & Marker Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ball_pos:
                print("No ball found (run_controller)")
                continue
            
            #print(f"Ball position: {ball_pos}", end="\r")

            #--- Control ---

            #if time.time() - start_time < 30:
                #controller.posControl((770-150, 330-150))
            #else:
                #controller.posControl((770+150, 330+150))

            #---------------


    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Tracker stopped.")

if __name__ == "__main__":
    main()
