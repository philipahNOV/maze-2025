import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp
import cv2
from PIL import ImageDraw, ImageFont
import math


class AutoPathScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.waiting_phase = 0

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()
        self.update_image()  # Start updating the image

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("NavigationScreen")

    def update_image(self):
        if self.mqtt_client.img is not None:
            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.image_label.imgtk = imgtk
            self.image_label.config(image=imgtk)
            self.status_label.place_forget()
        else:
            blank_image = Image.open(self.controller.blank_image_path).convert("RGB")
            imgtk = ImageTk.PhotoImage(image=blank_image)
            self.image_label.imgtk = imgtk
            self.image_label.config(image=imgtk)
            waiting_text = "FINDING PATH" + "." * (math.floor(self.waiting_phase / 2) + 1)
            self.waiting_phase = (self.waiting_phase + 1) % 6
            self.status_label.config(text=waiting_text)
            self.status_label.place(x=100, y=230, width=300, height=50)
        self.after(200, self.update_image)  # update every 200 ms

    def add_essential_buttons(self):
        self.exit_button = tk.Button(
            self,
            text="✖",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",              # Red exit button
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.controller.on_close  # or self.controller.destroy
        )
        self.exit_button.place(x=964, y=10, width=50, height=50) 

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.image_label = tk.Label(self)
        self.image_label.place(x=30, y=75, width=450, height=450)
        self.add_essential_buttons()

        self.back_button = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 20),
            fg="white",
            bg="#EE3229",           
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back,
        )
        self.back_button.place(x=804, y=10, width=150, height=50)

        self.status_label = tk.Label(
            self,
            text="",
            font=("Jockey One", 60),
            fg="white",
            bg="#4D4D4D",  # Match background or make transparent if needed
            anchor="w",    # Left aligned
            justify="left"
        )


    def show(self):
        """Make this frame visible"""
        self.mqtt_client.img = None  # Reset image to trigger loading state
        self.waiting_phase = 0  # Reset waiting phase for loading state

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
