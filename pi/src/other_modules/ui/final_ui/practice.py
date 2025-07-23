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
        self.update()
        bg_label = tk.Label(self, image=self.background_image)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        tk.Label(
            self,
            text="PRACTICE MODE",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=320, y=50)

        tk.Label(
            self,
            text="Move the ball using the joystick\nFreely explore the maze.",
            font=("Jockey One", 22),
            justify="center",
            bg="#D9D9D9",
            fg="#333"
        ).place(x=270, y=200)

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
            command=self.on_back
        ).place(x=10, y=10, width=120, height=50)

    def on_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")

    def show(self):
        self.update_idletasks()

    def hide(self):
        self.pack_forget()