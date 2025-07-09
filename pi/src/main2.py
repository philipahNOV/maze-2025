import tkinter as tk

from other_modules.ui.final_ui.booting import BootScreen
from other_modules.ui.final_ui.controlling import ControllingScreen
from other_modules.ui.final_ui.maze_navigation import NavigationScreen
from other_modules.ui.final_ui.info import InfoScreen
from other_modules.ui.final_ui.locating_ball import LocatingScreen
from other_modules.ui.final_ui.main_screen import MainScreen
from other_modules.ui.final_ui.path_finding_auto import AutoPathScreen
from other_modules.ui.final_ui.path_finding_custom import CustomPathScreen

import subprocess
from other_modules.mqtt_client_2 import MQTTClientPi

import signal
import sys
import os


class MainApp(tk.Tk):
    def __init__(self, mqtt_client):
        super().__init__()
        self.title("NOV maze 2025")
        self.geometry("1024x600")
        self.attributes("-fullscreen", True)
        self.current_screen = None
        self.nov_red = "#EE3229"
        self.nov_grey = "#60666C"
        self.nov_background = "#D9D9D9"

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.image_path = os.path.abspath(os.path.join(self.script_dir, '..', 'data'))
        self.logo_path = os.path.join(self.image_path, 'logo.png')
        self.background_path = os.path.join(self.image_path, 'background.png')
        self.blank_image_path = os.path.join(self.image_path, 'blank_image.png')

        self.current_frame = "BootScreen"
        self.mqtt_client = mqtt_client

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        boot = BootScreen(parent=container, controller=self, mqtt_client=self.mqtt_client)
        self.frames["BootScreen"] = boot
        boot.grid(row=0, column=0, sticky="nsew")
        self.show_frame("BootScreen")

        for F in (
            NavigationScreen, InfoScreen, LocatingScreen,
            MainScreen, AutoPathScreen, CustomPathScreen, ControllingScreen
        ):
            frame = F(parent=container, controller=self, mqtt_client=self.mqtt_client)
            self.frames[F.__name__] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        print("Frames initialized:", self.frames)

<<<<<<< HEAD
    def get_current_screen(self):
        return self.current_screen  # Returns the current active screen
=======
        self.show_frame("BootScreen")

>>>>>>> 052c323ecc5ffbade684a01f7fcacc0682b453aa

    def show_frame(self, page_name):
        print("Attempting to show frame:", page_name)
        self.current_screen = page_name
        frame = self.frames[page_name]

        def raise_and_show():
            frame.tkraise()
            frame.focus_set()
            self.update_idletasks()
            self.update()
            self.after(10, lambda: frame.event_generate("<Configure>"))
            if hasattr(frame, "show"):
                frame.show()

        # Slight delay gives X server a chance to sync and load fonts properly
        self.after(30, raise_and_show)

    def on_close(self):
        try:
            self.mqtt_client.shut_down()
        except Exception as e:
            print(f"Error stopping MQTT client: {e}")
        self.destroy()
        sys.exit(0)

def signal_handler(sig, frame):
    print('Signal received:', sig)
    if sig == signal.SIGINT or sig == signal.SIGTSTP: # type: ignore
        app.on_close()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTSTP, signal_handler) # type: ignore

def main():
    global app
    mqtt_client = MQTTClientPi()
    app = MainApp(mqtt_client)
    mqtt_client.set_app(app)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()    


if __name__ == "__main__":
    main()
