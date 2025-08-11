import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class ConnectionLostScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.create_widgets()

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def add_essential_buttons(self):
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

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.add_essential_buttons()

        self.title = tk.Label(
            self,
            text="PROGRAM CRASHED",
            font=("Jockey One", 45),  # 1.5x of original 30
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=512, y=200, anchor="n")  # shifted down to keep spacing similar

        self.under_title = tk.Label(
            self,
            text="ATTEMPTING TO RECOVER IN 5 SECONDS",
            font=("Jockey One", 30),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.under_title.place(x=512, y=280, anchor="n")

        self.under_title = tk.Label(
            self,
            text="PLEASE WAIT...",
            font=("Jockey One", 30),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.under_title.place(x=512, y=340, anchor="n")

    def show(self):
        pass
    
    def hide(self):
        self.pack_forget()