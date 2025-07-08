import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp
import cv2
from PIL import ImageDraw, ImageFont


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
        else:
            # Load blank image
            blank_image = Image.open(self.controller.blank_image_path).convert("RGB")
            draw = ImageDraw.Draw(blank_image)

            # Cycle through "Waiting." -> "Waiting.." -> "Waiting..."
            waiting_text = "FINDING PATH" + "." * (self.waiting_phase + 1)
            self.waiting_phase = (self.waiting_phase + 1) % 3

            # Load a font (optional: provide TTF path or use default)
            try:
                font = ImageFont.truetype("Jockey One", 36)
            except:
                font = ImageFont.load_default()

            # Draw text centered
            text_width, text_height = draw.textsize(waiting_text, font=font)
            x = (blank_image.width - text_width) // 2
            y = (blank_image.height - text_height) // 2
            draw.text((x, y), waiting_text, font=font, fill=(0, 0, 0))

            imgtk = ImageTk.PhotoImage(image=blank_image)
            self.image_label.imgtk = imgtk
            self.image_label.config(image=imgtk)
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


    def show(self):
        """Make this frame visible"""
        self.mqtt_client.img = None  # Reset image to trigger loading state
        self.waiting_phase = 0  # Reset waiting phase for loading state

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
