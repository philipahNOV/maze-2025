import tkinter as tk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING
import os

if TYPE_CHECKING:
    from main import MainApp


class PracticeScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.xbox_controller_image_path = os.path.join(self.controller.image_path, 'xbox_controller.png')
        controller_img = Image.open(self.xbox_controller_image_path)
        original_size = controller_img.size
        new_size = (int(original_size[0] * 0.75), int(original_size[1] * 0.75))
        controller_img_resized = controller_img.resize(new_size, Image.Resampling.LANCZOS)
        self.controller_image = ImageTk.PhotoImage(controller_img_resized)
        self.create_widgets()

    def create_widgets(self):
        self.update()
        bg_label = tk.Label(self, image=self.background_image)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        controller_label = tk.Label(self, image=self.controller_image, bg="#D9D9D9")
        controller_label.image = self.controller_image  # anti garbage collection
        controller_label.place(relx=0.52, rely=0.5, anchor="center")

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
        ).place(x=864, y=10, width=150, height=50)

    def on_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")

    def show(self):
        self.focus_set()
        self.update_idletasks()

    def hide(self):
        self.pack_forget()