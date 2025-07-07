from enum import Enum, auto
from mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
from camera.tracker_service import TrackerService
import pos2


class SystemState(Enum):
    BOOTING = auto()
    MAIN_SCREEN = auto()
    INFO_SCREEN = auto()
    NAVIGATION = auto()
    LOCATING = auto()
    TRACKING = auto()
    STOPPED = auto()

class HMIController:
    def __init__(self, tracking_service: TrackerService, arduino_thread: ArduinoConnection):
        self.state = SystemState.BOOTING
        self.tracking_service = tracking_service
        self.arduino_thread = arduino_thread
        self.control_type = None

    def on_command(self, cmd):
        print(f"[FSM] State: {self.state.name} | Command received: {cmd}")
        if self.state == SystemState.BOOTING:
            if cmd == "booted":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")
        elif self.state == SystemState.MAIN_SCREEN:
            if cmd == "Info":
                self.state = SystemState.INFO_SCREEN
                print("[FSM] Transitioned to INFO_SCREEN")

            elif cmd == "Navigate":
                self.state = SystemState.NAVIGATION
                print("[FSM] Transitioned to NAVIGATION")

        elif self.state == SystemState.INFO_SCREEN:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN from INFO_SCREEN")

        elif self.state == SystemState.NAVIGATION:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN from NAVIGATION")
            if cmd.startswith("Locate"):
                if cmd.endswith("Safe"):
                    self.control_type = "SafeControl"
                elif cmd.endswith("Speed"):
                    self.control_type = "SpeedControl"
                self.tracking_service.start_tracker()
                self.state = SystemState.LOCATING
                print("[FSM] Transitioned to LOCATING from NAVIGATION")


        elif self.state == SystemState.LOCATING:
            if self.model.tracking_service.is_initialized:
                self.state = SystemState.TRACKING

        elif cmd == "Emergency_Stop":
            self.model.stop_all()
            self.state = SystemState.STOPPED