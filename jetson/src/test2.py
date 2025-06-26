import testing.yolov1.hsv2 as tracking
import time
import cv2



def main():
    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
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
                    cv2.putText(frame, label, (pos[0]+10, pos[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            ball_pos = tracker.get_position()
            if ball_pos:
                print(f"Ball position: {ball_pos}", end="\r")

            tracker.get_orientation()

            cv2.imshow("Ball & Marker Tracking", frame)
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
