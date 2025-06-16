import threading
import cv2
class DisplayThread(threading.Thread):
    """A thread for displaying frames from a camera for debugging purposes."""

    def __init__(self, camera_thread):
        threading.Thread.__init__(self)
        self.camera_thread = camera_thread
        self.running = True

    def run(self):
        while self.running:
            frame = self.camera_thread.get_latest_frame()
            if frame is not None:
                cv2.imshow('Camera Feed', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop()

    def stop(self):
        """Stop the display thread."""
        self.running = False
        cv2.destroyAllWindows()