import testing.yolov1.hsv2 as tracking
from time import sleep

def main():
    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
    tracker.start()
    
    try:
        while True:
            position = tracker.get_position()
            if position:
                print("Ball position:", position)
            else:
                print("Ball not detected")
            sleep(0.1) 
    except KeyboardInterrupt:
        print("\nStopping tracker...")
        tracker.stop()

if __name__ == "__main__":
    main()