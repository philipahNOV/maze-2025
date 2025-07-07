from enum import Enum
from mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
import rl2
import pos2
from camera.tracker_service import TrackerService

class State(Enum):
    BOOTING = "booting"
    MAIN_SCREEN = "main_screen"
    INFO_SCREEN = "info_screen"
    NAVIGATION_SCREEN = "navigation_screen"
    LOCATING_SCREEN = "locating_screen"



class FSM():
    def __init__(self, tracking_service: TrackerService, arduino_thread: ArduinoConnection):
        self.state = State.BOOTING
        self.tracking_service = tracking_service
        self.arduino_thread = arduino_thread
        self.controller = pos2.Controller(arduino_thread, tracking_service)


    def take_action(self, action):
        if self.state == State.BOOTING:
            pass
        elif self.state == State.MAIN_SCREEN:
            pass
        elif self.state == State.INFO_SCREEN:
            pass
        elif self.state == State.NAVIGATION_SCREEN:
            pass

    def set_state(self, state):
        try:
            self.state = state
        except Exception as e:
            print(f"Error setting state: {e}")