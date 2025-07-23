import tkinter as tk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainApp


class PlayAloneScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.name = ""

        self.create_widgets()

    def create_widgets(self):
        self.update()

        # Background
        bg_label = tk.Label(self, image=self.background_image)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        # Title
        tk.Label(
            self,
            text="PLAY ALONE",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=330, y=30)

        # Name display
        self.name_label = tk.Label(
            self,
            text="ENTER YOUR NAME:",
            font=("Jockey One", 24),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.name_label.place(x=320, y=110)

        self.name_entry_display = tk.Label(
            self,
            text=self.name,
            font=("Jockey One", 28),
            bg="white",
            fg="#222",
            bd=2,
            relief="sunken",
            width=20,
            anchor="center"
        )
        self.name_entry_display.place(x=270, y=160, height=50)

        # Keyboard layout
        keys = [
            list("ABCDEFGHIJKL"),
            list("MNOPQRSTUVWXYZ"),
            ["SPACE", "BACK", "CLEAR"]
        ]

        for row_idx, row in enumerate(keys):
            for col_idx, key in enumerate(row):
                btn = tk.Button(
                    self,
                    text=key,
                    font=("Jockey One", 18),
                    bg="#444",
                    fg="white",
                    activebackground="#666",
                    width=4,
                    height=2,
                    command=lambda k=key: self.key_press(k)
                )
                btn.place(x=130 + col_idx * 60, y=240 + row_idx * 65)

        # Start button
        self.start_button = tk.Button(
            self,
            text="START",
            font=("Jockey One", 24),
            fg="white",
            bg="#A0A0A0",
            activebackground="#4B4C4C",
            activeforeground="white",
            borderwidth=0,
            highlightthickness=0,
            state="disabled",
            command=self.start_game
        )
        self.start_button.place(x=400, y=470, width=200, height=60)

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

    def key_press(self, key):
        if key == "BACK":
            self.name = self.name[:-1]
        elif key == "CLEAR":
            self.name = ""
        elif key == "SPACE":
            self.name += " "
        else:
            self.name += key

        self.name_entry_display.config(text=self.name)

        # Enable START button if name is valid
        if self.name.strip():
            self.start_button.config(state="normal", bg="#60666C")
        else:
            self.start_button.config(state="disabled", bg="#A0A0A0")

    def start_game(self):
        player_name = self.name.strip()
        self.controller.player_name = player_name
        print(f"Starting game as '{player_name}'")

        # Notify Jetson of player name (Jetson will pair name with time)
        self.mqtt_client.client.publish("jetson/player_name", player_name)

        # Transition to game
        self.mqtt_client.client.publish("jetson/command", "StartGame")

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")
