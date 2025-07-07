import time
import threading


class BlinkRed(threading.Thread):
    def __init__(self, arduino_thread):
        super().__init__(daemon=True)
        self.arduino_thread = arduino_thread
        self._stop_event = threading.Event()

    def run(self):
        red = (255, 0, 0)
        white = (255, 255, 255)
        while not self._stop_event.is_set():
            self.arduino_thread.send_color(*red)
            time.sleep(0.2)
            self.arduino_thread.send_color(*white)
            time.sleep(2.3)

    def stop(self):
        self._stop_event.set()
