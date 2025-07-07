import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class MainScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()

    def on_button_click_disco(self):
        pass

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
        self.add_essential_buttons()

        self.navigation_button = tk.Button(
            self,
            text="MAZE NAVIGATION",
            font=("Jockey One", 25),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=lambda: self.controller.show_frame("NavigationScreen")
        )
        self.navigation_button.place(x=391, y=235, width=243, height=74)

        self.info_button = tk.Button(
            self,
            text="INFO",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=lambda: self.controller.show_frame("InfoScreen"),
        )
        self.info_button.place(x=391, y=320, width=243, height=74)

        self.disco_button = tk.Button(
            self,
            text="DISCO",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_disco,
        )
        self.disco_button.place(x=391, y=405, width=243, height=74)

        self.title = tk.Label(
            self,
            text="WELCOME",
            font=("Jockey One", 55),   # or any font you prefer
            fg="#1A1A1A",                # text color
            bg="#D9D9D9"                 # background (or match your image if needed)
        )
        self.title.place(x=400, y=100)

    def show(self):
        """Make this frame visible"""

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
