import testing.yolov1.hsv2 as tracking

ballTracker = tracking.BallTracker("testing/yolov1/best.pt")

try:
    ballTracker.init_camera()
    frame = ballTracker.grab_frame()
    if frame is None:
        print("Failed to grab frame from camera.")
        exit(1)
    
    ballTracker.start()
    print("start")

    while True:
        pos = ballTracker.get_position()
        if pos is None:
            print("No position detected.")
            break

        print(f"Ball position: x={pos[0]}, y={pos[1]}")

except KeyboardInterrupt:
    print("Keyboard interrupt detected. Stopping the tracker.")

    print(ballTracker.get_position())

finally:
    ballTracker.stop()
    print("Tracker stopped.")