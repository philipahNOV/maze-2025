import tkinter as tk

from other_modules.ui.final_ui.booting import BootScreen
from other_modules.ui.final_ui.controlling import ControllingScreen
from other_modules.ui.final_ui.maze_navigation import NavigationScreen
from other_modules.ui.final_ui.info import InfoScreen
from other_modules.ui.final_ui.locating_ball import LocatingScreen
from other_modules.ui.final_ui.main_screen import MainScreen
from other_modules.ui.final_ui.path_finding_auto import AutoPathScreen
from other_modules.ui.final_ui.path_finding_custom import CustomPathScreen
from other_modules.mqtt_client import MQTTClientPi
from other_modules.ui.final_ui.human_mode import HumanScreen
from other_modules.ui.final_ui.practice import PracticeScreen
from other_modules.ui.final_ui.play_alone import PlayAloneScreen
from other_modules.ui.final_ui.play_alone_start import PlayAloneStartScreen
from other_modules.ui.final_ui.play_vs_ai import PlayVsAIScreen
from other_modules.ui.final_ui.leaderboard import LeaderboardScreen

import signal
import sys
import os
from pathlib import Path
import yaml


class MainApp(tk.Tk):
    def __init__(self, mqtt_client, config):
        super().__init__()
        self.title("NOV maze 2025")
        self.geometry("1024x600")
        #self.attributes("-fullscreen", True)
        self.overrideredirect(True)
        self.geometry("1024x600")
        self.resizable(False, False)
        self.current_screen = None
        self.nov_red = "#EE3229"
        self.nov_grey = "#60666C"
        self.nov_background = "#D9D9D9"
        self.reset_jetson_on_exit = False
        self.config = config

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.image_path = os.path.abspath(os.path.join(self.script_dir, '..', 'data'))
        self.logo_path = os.path.join(self.image_path, 'logo.png')
        self.background_path = os.path.join(self.image_path, 'background.png')
        self.blank_image_path = os.path.join(self.image_path, 'blank_image.png')
        self.check_true_path = os.path.join(self.image_path, 'Check_true.png')
        self.check_false_path = os.path.join(self.image_path, 'Check_false.png')
        self.touch_path = os.path.join(self.image_path, 'touch.png')
        self.loading_animation_path = os.path.join(self.image_path, 'loading_animation')
        self.pathfinding_animation_path = os.path.join(self.image_path, 'pathfinding_animation')
        self.xbox_controller_image_path = os.path.join(self.image_path, 'xbox_controller.png')

        self.current_frame = "BootScreen"
        self.mqtt_client = mqtt_client
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (BootScreen, NavigationScreen, InfoScreen, LocatingScreen, MainScreen, AutoPathScreen, CustomPathScreen, ControllingScreen, HumanScreen, PracticeScreen,
                  PlayAloneScreen, PlayAloneStartScreen, PlayVsAIScreen, LeaderboardScreen):
            frame = F(parent=container, controller=self, mqtt_client=self.mqtt_client)
            frame.grid(row=0, column=0, sticky="nsew")
            frame.lower()
            self.frames[F.__name__] = frame
        
        print("Frames initialized:", self.frames)

        self.show_frame("BootScreen")

    def get_current_screen(self):
        return self.current_screen 

    def show_frame(self, page_name):
        print("Attempting to show frame:", page_name)
        self.current_screen = page_name
        frame = self.frames[page_name]
        frame.tkraise()

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

        python = sys.executable
        script = os.path.abspath(sys.argv[0])
        print(f"Launching new process: {python} {script}")
        #subprocess.Popen([python, script], start_new_session=True)
        #sys.exit(0)
        os.execv(python, [python, script])

def load_config():
    project_root = Path(__file__).resolve().parents[2]  # up from src → jetson → maze-2025
    config_path = project_root / "config.yaml"

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found at {config_path}")

    with config_path.open('r') as file:
        config = yaml.safe_load(file)
    return config

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
    config = load_config()
    app = MainApp(mqtt_client, config)
    mqtt_client.set_app(app)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()    


if __name__ == "__main__":
    main()