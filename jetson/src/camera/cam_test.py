from cam_loop import CameraThread
import cv2
import time

# Create and start the camera thread
camthread = CameraThread()
camthread.start()

# Display frames in real-time
try:
    while True:
        last_frame = camthread.get_latest_frame()
        if last_frame is not None:
            cv2.imshow("Live Frame", last_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.01)  # Add a small delay to prevent high CPU usage

except KeyboardInterrupt:
    pass
finally:
    # Stop the camera thread and close OpenCV windows
    camthread.stop()
    cv2.destroyAllWindows()
