import testing.yolov1.hsv2 as tracking

ballTracker = tracking.BallTracker("testing/yolov1/best.pt")
ballTracker.start()
while True:
    print(ballTracker.get_position())