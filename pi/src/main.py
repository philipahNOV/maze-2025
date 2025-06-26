import tkinter as tk
#from other_modules.ui.screen1 import Screen1
from other_modules.ui.start_screen import Screen1
from other_modules.ui.screen2 import Screen2
from other_modules.ui.screen3 import Screen3
from other_modules.ui.testscreen import ScreenTemplate
from other_modules.ui.loading_path import LoadingPath
from other_modules.ui.need_path import NeedPath
from other_modules.ui.elManuel import elManuel
#from other_modules.ui.boot_screen import BootScreen
from other_modules.ui.boot_screen_1 import BootScreen

from other_modules.mqtt_client import MQTTClientPi
from other_modules.data_handler import DataHandler

import signal
import sys
import os
import time


class MainApp(tk.Tk):
    def __init__(self, mqtt_client):
        super().__init__()
        self.title("Raspberry Pi Application")
        self.geometry("800x480") # This is the wrong reselution for the pi screen
        self.attributes("-fullscreen", True)
        self.current_screen = None

        #self.data_handler = DataHandler(self.get_current_screen)

        self.current_frame = "BootScreen"

        self.mqtt_client = mqtt_client
        # Container to hold all the frames
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (BootScreen, Screen1):
            frame = F(parent=container, controller=self, mqtt_client=self.mqtt_client)
            self.frames[F.__name__] = frame
            frame.grid(row=0, column=0, sticky="nsew")
        
        print("Frames initialized:", self.frames)

        self.show_frame("BootScreen")

        #self.mqtt_client.screen2_instance = self.frames["Screen2"]

    def get_current_screen(self):
        return self.current_screen  # Returns the current active screen

    def show_frame(self, page_name):
        """Show a frame for the given page name"""
        print("Attempting to show frame:", page_name)
        self.current_screen = page_name
        frame = self.frames[page_name]
        frame.tkraise()

    def on_close(self):
        #self.frames["Screen3"].stop_update_camera_feed_thread()
        #self.data_handler.close_resources()
        try:
            self.mqtt_client.shut_down()
        except Exception as e:
            print(f"Error stopping MQTT client: {e}")
        self.destroy()
        sys.exit(0)

    def restart_program(self):
        print("Restart requested...")
        # Gracefully shut down but don't exit
        try:
            self.mqtt_client.shut_down()
        except Exception as e:
            print(f"Error stopping MQTT client: {e}")

        # Schedule restart after window is closed
        self.after(100, self._do_restart)
        self.destroy()  # This exits mainloop()

    def _do_restart(self):
        print("Re-executing main.py...")
        python = sys.executable
        os.execl(python, python, *sys.argv)

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
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()    


if __name__ == "__main__":
    main()
