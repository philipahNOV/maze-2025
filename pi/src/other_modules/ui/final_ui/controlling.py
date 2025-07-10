import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class ControllingScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()
        self.update_image()  # Start updating the image

    def on_button_click_template(self):
        self.mqtt_client.client.publish("jetson/command", "Template")

    def update_image(self):
        if self.mqtt_client.img is not None:
            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            #img = img.resize((320, 240), Image.Resampling.LANCZOS)
            imgtk = ImageTk.PhotoImage(image=img)
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
        self.image_label.place(x=287, y=50, width=450, height=450)
        self.add_essential_buttons()

        self.template_button = tk.Button(
            self,
            text="TEMPLATE",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_template,
        )
        self.template_button.place(x=690, y=150, width=243, height=74)

        self.template_title = tk.Label(
            self,
            text="Template",
            font=("Jockey One", 40),   # or any font you prefer
            fg="#EE3229",                # text color
            bg="#D9D9D9"                 # background (or match your image if needed)
        )
        self.template_title.place(x=410, y=315)

    def show(self):
        """Make this frame visible"""

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
