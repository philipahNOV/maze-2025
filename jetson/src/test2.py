import testing.yolov1.hsv2 as tracking
import cv2
import time

def main():
    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
    print("Starting BallTracker tracking loop. Press Ctrl+C to stop.")
    try:
        tracker._tracking_loop()
    except KeyboardInterrupt:
        print("Tracking stopped by user.")
        tracker.stop()

if __name__ == "__main__":
    main()