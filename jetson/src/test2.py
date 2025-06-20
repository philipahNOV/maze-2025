import testing.yolov1.hsv2 as tracking
import cv2
import time

def main():
    tracker = tracking.BallTracker(model_path="best.pt")
    tracker.start()
    print("BallTracker started. Press 'q' to quit.")

    try:
        while True:
            frame = tracker.frame
            pos = tracker.get_position()
            if frame is not None:
                # Draw the tracked position
                if pos is not None:
                    cv2.circle(frame, pos, 10, (0, 255, 0), 2)
                    cv2.putText(frame, f"Ball: {pos}", (pos[0]+15, pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.imshow("Ball Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.01)
    finally:
        tracker.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()