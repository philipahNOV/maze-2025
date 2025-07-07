import tkinter as tk
from other_modules.ui.start_screen import Screen1
from other_modules.ui.boot_screen_1 import BootScreen1
from other_modules.ui.tuning_screen import Tuning
from other_modules.ui.control_screen import ControlScreen

import subprocess
from other_modules.mqtt_client import MQTTClientPi

import signal
import sys
import os


class MainApp(tk.Tk):
    def __init__(self, mqtt_client):
        super().__init__()
        self.title("Raspberry Pi Application")
        self.geometry("1024x600") # This is the wrong reselution for the pi screen
        self.attributes("-fullscreen", True)
        self.current_screen = None
        self.nov_red = "#EE3229"
        self.nov_grey = "#60666C"
        self.nov_background = "#D9D9D9"

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.image_path = os.path.abspath(os.path.join(self.script_dir, '..', 'data'))
        self.logo_path = os.path.join(self.image_path, 'logo.png')
        self.background_path = os.path.join(self.image_path, 'background.png')

        self.current_frame = "BootScreen1"

        self.mqtt_client = mqtt_client
        # Container to hold all the frames
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (BootScreen1, Screen1, Tuning, ControlScreen):
            frame = F(parent=container, controller=self, mqtt_client=self.mqtt_client)
            self.frames[F.__name__] = frame
            frame.grid(row=0, column=0, sticky="nsew")
        
        print("Frames initialized:", self.frames)

        self.show_frame("BootScreen1")


    def get_current_screen(self):
        return self.current_screen  # Returns the current active screen

    def show_frame(self, page_name):
        """Show a frame for the given page name"""
        print("Attempting to show frame:", page_name)
        self.current_screen = page_name
        frame = self.frames[page_name]
        frame.tkraise()

        # If the frame has a custom `show()` method, call it
        if hasattr(frame, "show"):
            frame.show()

    def on_close(self):
        try:
            self.mqtt_client.shut_down()
        except Exception as e:
            print(f"Error stopping MQTT client: {e}")
        self.destroy()
        sys.exit(0)

    def restart_program(self):
        print("Restart requested...")
        try:
            self.mqtt_client.shut_down()
        except Exception as e:
            print(f"Error stopping MQTT client: {e}")

        # Launch a new process first
        python = sys.executable
        script = os.path.abspath(sys.argv[0])
        print(f"Launching new process: {python} {script}")
        subprocess.Popen([python, script], start_new_session=True)
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
