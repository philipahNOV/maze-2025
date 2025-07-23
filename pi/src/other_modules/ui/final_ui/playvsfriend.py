import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainApp

class PlayVsFriendScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.selected_player_count = None

        self.create_widgets()

    def create_widgets(self):
        self.update()

        bg_label = tk.Label(self, image=self.background_image)
        bg_label.image = self.background_image
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        # Title
        tk.Label(
            self,
            text="PLAY VS FRIEND",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=300, y=30)

        # Number of Players Label
        tk.Label(
            self,
            text="Number of Players:",
            font=("Jockey One", 24),
            bg="#D9D9D9"
        ).place(x=300, y=140)

        # Dropdown for Player Count
        self.player_count_var = tk.StringVar(value="")  # Initially blank
        self.dropdown = ttk.Combobox(
            self,
            textvariable=self.player_count_var,
            values=[""] + [str(i) for i in range(2, 7)],
            state="readonly",
            font=("Jockey One", 24),
            width=10,
            justify="center"
        )
        self.dropdown.place(x=580, y=140)
        self.dropdown.bind("<<ComboboxSelected>>", self.on_player_count_selected)

        # Start button (disabled by default)
        self.start_button = tk.Button(
            self,
            text="START",
            font=("Jockey One", 24),
            fg="white",
            bg="#A0A0A0",
            activeforeground="white",
            activebackground="#4B4C4C",
            borderwidth=0,
            highlightthickness=0,
            state="disabled",
            command=self.start_game
        )
        self.start_button.place(x=420, y=440, width=200, height=60)

        # Back button
        self.back_button = tk.Button(
            self,
            text="←",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back
        )
        self.back_button.place(x=844, y=10, width=50, height=50)

    def on_player_count_selected(self, event=None):
        selected = self.player_count_var.get()
        if selected.isdigit():
            self.selected_player_count = int(selected)
            self.start_button.config(state="normal", bg="#60666C")
        else:
            self.selected_player_count = None
            self.start_button.config(state="disabled", bg="#A0A0A0")

    def start_game(self):
        if self.selected_player_count:
            player_names = [f"Player {i+1}" for i in range(self.selected_player_count)]
            print(f"Starting game with players: {player_names}")
            # Future: Send names via MQTT or transition to gameplay screen

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")