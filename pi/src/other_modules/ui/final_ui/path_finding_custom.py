import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp
import cv2


class CustomPathScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.scale_ratio = 0.8
        self.true_width = 730
        self.true_height = 710
        self.offset_x = 390
        self.offset_y = 10

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()
        #self.update_image()  # Start updating the image

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("NavigationScreen")

    def on_canvas_click(self, event):
        x, y = event.x, event.y
        x = self.true_width - int(x / self.scale_ratio) + self.offset_x
        y = self.true_height - int(y / self.scale_ratio) + self.offset_y
        print(f"[CustomPathScreen] Clicked at pixel: ({x}, {y})")

        # Optional: show a small circle at the clicked location
        r = 3
        self.canvas.create_oval(x-r, y-r, x+r, y+r, fill="red", outline="")

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
        self.canvas = tk.Canvas(self, width=int(self.true_width * self.scale_ratio), height=int(self.true_height * self.scale_ratio))
        self.canvas.place(x=0, y=0, relwidth=1, relheight=1)
        
        blank_image_pil = Image.open(self.controller.blank_image_path).convert("RGB")
        scaled_pil = blank_image_pil.resize(
            (int(self.true_width * self.scale_ratio), int(self.true_height * self.scale_ratio)),
            Image.Resampling.LANCZOS
        )
        blank_image = ImageTk.PhotoImage(scaled_pil)

        self.image = blank_image  # Keep a reference to prevent garbage collection
        self.image_id = self.canvas.create_image(0, 0, anchor="nw", image=self.image)

        self.canvas.bind("<Button-1>", self.on_canvas_click)
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

    def show(self):
        """Make this frame visible"""

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
