import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class HumanScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        try:
            self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        except Exception as e:
            print("Error loading background image:", e)
            self.background_image = None

        self.create_widgets()

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")

    def on_button_click_restart(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def create_widgets(self):
        self.update()

        if self.background_image:
            self.bg_label = tk.Label(self, image=self.background_image)
            self.bg_label.image = self.background_image  # keep reference
            self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.exit_button = tk.Button(
            self,
            text="✖",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_exit
        )
        self.exit_button.place(x=964, y=10, width=50, height=50)

        self.restart_button = tk.Button(
            self,
            text="⟲",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_restart
        )
        self.restart_button.place(x=904, y=10, width=50, height=50)

        self.back_button = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 28),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back
        )
        self.back_button.place(x=744, y=10, width=150, height=50)

        self.title = tk.Label(
            self,
            text="HUMAN MODE",
            font=("Jockey One", 55),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=320, y=100)

        self.play_ai_button = tk.Button(
            self,
            text="PLAY VS ROBOT",
            font=("Jockey One", 25),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.play_vs_ai
        )
        self.play_ai_button.place(x=391, y=235, width=243, height=74)

        self.play_alone_button = tk.Button(
            self,
            text="PLAY ALONE",
            font=("Jockey One", 25),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.play_alone
        )
        self.play_alone_button.place(x=391, y=320, width=243, height=74)

        self.practice_button = tk.Button(
            self,
            text="PRACTICE",
            font=("Jockey One", 25),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.practice_mode
        )
        self.practice_button.place(x=391, y=405, width=243, height=74)

        self.leaderboard_button = tk.Button(
            self,
            text="LEADERBOARD",
            font=("Jockey One", 25),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.show_leaderboard
        )
        self.leaderboard_button.place(x=391, y=490, width=243, height=74)


    def play_vs_ai(self):
        print("[HumanScreen] Play vs AI selected")
        self.mqtt_client.client.publish("jetson/command", "PlayVsAI")
        self.controller.show_frame("PlayVsAIScreen")

    def play_alone(self):
        self.mqtt_client.client.publish("jetson/command", "PlayAlone")
        self.controller.show_frame("PlayAloneScreen")

    def practice_mode(self):
        self.mqtt_client.client.publish("jetson/command", "Practice")
        self.controller.show_frame("PracticeScreen")

    def show_leaderboard(self):
        self.mqtt_client.client.publish("jetson/command", "Leaderboard")
        self.controller.show_frame("LeaderboardScreen")
    
    def show(self):
        self.focus_set()
        self.update_idletasks()