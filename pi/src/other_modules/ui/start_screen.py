import tkinter as tk
from PIL import Image, ImageTk
from tkinter import font as tkfont

import time


class Screen1(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.image = ImageTk.PhotoImage(Image.open('../data/start_screen.png'))

        # Layout the widgets including the logo
        self.create_widgets()

    def on_button_click(self):
        self.mqtt_client.client.publish("jetson/command", "test")
        #self.controller.show_frame("elManuel")
        print("Sent command: 1.0")

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.example_button = tk.Button(
            self,
            text="START",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click,
        )
        self.example_button.place(x=390, y=385, width=243, height=74)  # Absolute placement        

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
