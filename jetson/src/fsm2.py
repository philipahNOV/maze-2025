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


class TrackerModel:
    def __init__(self, tracking_service: TrackerService, arduino_thread: ArduinoConnection):
        self.tracking_service = tracking_service
        self.arduino_thread = arduino_thread
        self.controller = pos2.Controller(arduino_thread, tracking_service)

    def start_tracking(self):
        self.tracking_service.start_tracker()

    def force_retrack(self):
        self.tracking_service.retrack()

    def stop_all(self):
        self.controller.arduinoThread.send_speed(0, 0)
        self.tracking_service.stop_tracker()


class TrackerView:
    def show_main_ui(self):
        print("[UI] Showing main menu.")

    def show_info_screen(self):
        print("[UI] Showing system info.")

    def show_navigation_ui(self):
        print("[UI] Entering navigation mode.")

    def show_tracking_ui(self):
        print("[UI] Tracker engaged.")

    def show_retracking_spinner(self):
        print("[UI] Retracking... please wait.")

    def flash_emergency_ui(self):
        print("[UI] EMERGENCY STOP triggered! Blinking red.")

    def show_boot_screen(self):
        print("[UI] Booting system...")



class HMIController:
    def __init__(self, tracking_service: TrackerService, arduino_thread: ArduinoConnection):
        self.state = SystemState.BOOTING
        self.model = TrackerModel(tracking_service, arduino_thread)
        self.view = TrackerView()

        self.view.show_boot_screen()
        self.state = SystemState.MAIN_SCREEN
        self.view.show_main_ui()

    def on_command(self, cmd):
        print(f"[FSM] State: {self.state.name} | Command received: {cmd}")

        if self.state == SystemState.MAIN_SCREEN:
            if cmd == "Info":
                self.state = SystemState.INFO_SCREEN
                self.view.show_info_screen()

            elif cmd == "Navigate":
                self.state = SystemState.NAVIGATION
                self.view.show_navigation_ui()

            elif cmd == "Control":
                self.model.start_tracking()
                self.state = SystemState.TRACKING
                self.view.show_tracking_ui()

        elif self.state == SystemState.TRACKING:
            if cmd == "Retrack":
                self.model.force_retrack()
                self.state = SystemState.LOCATING
                self.view.show_retracking_spinner()

            elif cmd == "Stop":
                self.model.stop_all()
                self.state = SystemState.STOPPED
                self.view.flash_emergency_ui()

        elif self.state == SystemState.INFO_SCREEN:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                self.view.show_main_ui()

        elif self.state == SystemState.NAVIGATION:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                self.view.show_main_ui()

        elif self.state == SystemState.LOCATING:
            if self.model.tracking_service.is_initialized:
                self.state = SystemState.TRACKING
                self.view.show_tracking_ui()

        elif cmd == "Emergency_Stop":
            self.model.stop_all()
            self.state = SystemState.STOPPED
            self.view.flash_emergency_ui()