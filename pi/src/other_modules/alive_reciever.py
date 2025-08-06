import threading
import time

class ImAliveThread(threading.Thread):
    def __init__(self, app):
        super().__init__(daemon=True)
        self.last_alive_time = None
        self.connection_lost = False
        self.app = app

    def run(self):
        time.sleep(5)  # Initial delay to allow other components to start
        while True:
            elapsed = time.time() - (self.last_alive_time if self.last_alive_time else 0)
            if elapsed > 5 and not self.connection_lost:
                self.connection_lost = True
                self.app.show_frame("InfoScreen")
                print("[ImAliveThread] Connection lost")
            else:
                if self.connection_lost:
                    self.connection_lost = False
            time.sleep(0.3)

