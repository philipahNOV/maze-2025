import threading
import time

class ImAliveThread(threading.Thread):
    def __init__(self, app):
        super().__init__(daemon=True)
        self.last_alive_time = None
        self.connection_lost = False
        self.app = app

    def run(self):
        time.sleep(1)  # Initial delay to allow other components to start
        while True:
            elapsed = time.time() - (self.last_alive_time if self.last_alive_time else 0)
            if elapsed > 5 and not self.connection_lost:
                self.connection_lost = True
                self.app.show_frame("ConnectionLostScreen")
                time.sleep(5)  # Wait before trying to recover
                print("[ImAliveThread] Connection lost")
                self.app.mqtt_client.client.publish("jetson/recovery", "recover")
                self.app.restart_program()
            elif elapsed <= 5:
                if self.connection_lost:
                    self.connection_lost = False
            time.sleep(0.3)

