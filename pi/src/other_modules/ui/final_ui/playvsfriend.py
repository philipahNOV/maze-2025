import tkinter as tk
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
        self.player_buttons = []

        self.create_widgets()

    def create_widgets(self):
        self.update()

        # Background
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

        # Player count label
        tk.Label(
            self,
            text="Select Number of Players:",
            font=("Jockey One", 24),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=300, y=140)

        # Player count buttons (2 to 6)
        for i in range(2, 7):
            btn = tk.Button(
                self,
                text=str(i),
                font=("Jockey One", 22),
                width=3,
                bg="#BBBBBB",
                fg="black",
                activebackground="#4B4C4C",
                activeforeground="white",
                borderwidth=0,
                highlightthickness=0,
                command=lambda n=i: self.select_player_count(n)
            )
            btn.place(x=300 + (i - 2) * 70, y=200, width=60, height=60)
            self.player_buttons.append(btn)

        # Start button (initially disabled)
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

    def select_player_count(self, count):
        self.selected_player_count = count
        self.start_button.config(state="normal", bg="#60666C")

        # Highlight selected button
        for btn in self.player_buttons:
            if btn["text"] == str(count):
                btn.config(bg="#60666C", fg="white")
            else:
                btn.config(bg="#BBBBBB", fg="black")

    def start_game(self):
        if self.selected_player_count:
            player_names = [f"Player {i+1}" for i in range(self.selected_player_count)]
            print(f"Starting game with players: {player_names}")
            # Future: send this list via MQTT or transition to gameplay screen

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")