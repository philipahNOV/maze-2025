import threading
import time

class ImAliveThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.last_alive_time = None
        self.connection_lost = False

    def run(self):
        time.sleep(5)  # Initial delay to allow other components to start
        while True:
            elapsed = time.time() - (self.last_alive_time if self.last_alive_time else 0)
            if elapsed > 5:
                self.connection_lost = True
                print("[ImAliveThread] Connection lost, publishing to MQTT.")
            else:
                if self.connection_lost:
                    self.connection_lost = False
            time.sleep(0.3)

