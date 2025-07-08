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
            time.sleep(4)

    def stop(self):
        self._stop_event.set()


class LookForBall:
    def __init__(self, tracking_service, on_ball_found=None):
        self.tracking_service = tracking_service
        self.on_ball_found = on_ball_found
        self._stop_event = threading.Event()

    def start_ball_check(self):
        thread = threading.Thread(target=self._check_loop, daemon=True)
        thread.start()

    def _check_loop(self):
        print("[LookForBall] Started checking...")
        while not self._stop_event.is_set():
            pos = self.tracking_service.get_ball_position()
            if pos is not None:
                print(f"[LookForBall] Ball found at {pos}")
                if self.on_ball_found:
                    self.on_ball_found()
                break
            time.sleep(0.1)

    def stop(self):
        self._stop_event.set()
