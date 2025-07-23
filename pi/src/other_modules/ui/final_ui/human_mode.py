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

    def on_button_click_info(self):
        self.mqtt_client.client.publish("jetson/command", "Info")
        self.controller.show_frame("InfoScreen")

    def create_widgets(self):
        self.update()

        if self.background_image:
            bg_label = tk.Label(self, image=self.background_image)
            bg_label.image = self.background_image
            bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        # --- Title ---
        tk.Label(
            self,
            text="HUMAN MODE",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=350, y=40)

        # --- Top Buttons ---
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

        self.info_button = tk.Button(
            self,
            text="📄",
            font=("Jockey One", 26),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_info
        )
        self.info_button.place(x=844, y=10, width=50, height=50)

        # --- Center Buttons ---
        button_config = {
            "font": ("Jockey One", 24),
            "fg": "white",
            "bg": "#60666C",
            "activebackground": "#4B4C4C",
            "activeforeground": "#DFDFDF",
            "width": 20,
            "height": 2,
            "borderwidth": 0,
            "highlightthickness": 0,
        }

        tk.Button(self, text="PLAY VS AI", command=self.play_vs_ai, **button_config).place(x=355, y=150)
        tk.Button(self, text="PLAY VS FRIEND", command=self.play_vs_friend, **button_config).place(x=355, y=230)
        tk.Button(self, text="PRACTICE", command=self.practice_mode, **button_config).place(x=355, y=310)
        tk.Button(self, text="LEADERBOARD", command=self.show_leaderboard, **button_config).place(x=355, y=390)

        tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 20),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back
        ).place(x=10, y=10, width=120, height=50)

    def play_vs_ai(self):
        print("[HumanScreen] Play vs AI selected")

    def play_vs_friend(self):
        print("[HumanScreen] Play vs Friend selected")

    def practice_mode(self):
        self.mqtt_client.client.publish("jetson/command", "Practice")
        self.controller.show_frame("PracticeScreen")

    def show_leaderboard(self):
        print("[HumanScreen] Leaderboard requested")