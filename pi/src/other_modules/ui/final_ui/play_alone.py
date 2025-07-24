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

        bg_label = tk.Label(self, image=self.background_image)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        tk.Label(
            self,
            text="PLAY ALONE",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=330, y=30)

        tk.Label(
            self,
            text="ENTER YOUR NAME:",
            font=("Jockey One", 24),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=320, y=110)

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

        keys = [
            list("ABCDEFGHIJKLMNO"),
            list("PQRSTUVWXYZÆØÅ"),
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
                btn.place(x=20 + col_idx * 70, y=240 + row_idx * 85)

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
            command=self.on_button_click_start_game
        )
        self.start_button.place(x=400, y=470, width=200, height=60)

        self.back_button = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back
        )
        self.back_button.place(x=857, y=10, width=150, height=50)

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

        if self.name.strip():
            self.start_button.config(state="normal", bg="#60666C")
        else:
            self.start_button.config(state="disabled", bg="#A0A0A0")

    def on_button_click_start_game(self):
        player_name = self.name.strip()
        if not player_name:
            print("[PlayAloneScreen] Start clicked with no name.")
            return

        self.controller.player_name = player_name
        print(f"[PlayAloneScreen] Starting game as '{player_name}'")

        self.mqtt_client.client.publish("jetson/player_name", player_name)
        self.mqtt_client.client.publish("jetson/command", "StartGame")
        self.controller.show_frame("PlayAloneStartScreen")

    def on_button_click_back(self):
        self.name = ""
        self.name_entry_display.config(text=self.name)
        self.start_button.config(state="disabled", bg="#A0A0A0")
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")

    def show(self):
        self.focus_set()
        self.name = ""
        self.name_entry_display.config(text=self.name)
        self.start_button.config(state="disabled", bg="#A0A0A0")