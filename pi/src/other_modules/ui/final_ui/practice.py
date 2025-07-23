import tkinter as tk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class PracticeScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.create_widgets()

    def create_widgets(self):
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.image = self.background_image
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        # Title text
        self.title = tk.Label(
            self,
            text="PRACTICE MODE",
            font=("Jockey One", 42),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=300, y=100)

        # Description
        self.description = tk.Label(
            self,
            text="Move around freely using the joystick",
            font=("Jockey One", 24),
            fg="#333333",
            bg="#D9D9D9"
        )
        self.description.place(x=250, y=180)

        # Back button
        self.back_button = tk.Button(
            self,
            text="← BACK",
            font=("Jockey One", 24),
            fg="white",
            bg="#60666C",
            activebackground="#4B4C4C",
            borderwidth=0,
            highlightthickness=0,
            command=self.on_back
        )
        self.back_button.place(x=30, y=20, width=140, height=50)

    def on_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")