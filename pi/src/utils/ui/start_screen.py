import tkinter as tk
from PIL import Image, ImageTk
from tkinter import font as tkfont
import time

class Screen1(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.image = ImageTk.PhotoImage(Image.open('../img/start_screen.png'))
        self.create_widgets()

    def on_button_click_elevator(self):
        self.mqtt_client.client.publish("jetson/command", "Elevator")

    def on_button_click_control(self):
        self.mqtt_client.client.publish("jetson/command", "Control")

    def on_button_click_horizontal(self):
        self.mqtt_client.client.publish("jetson/command", "Horizontal")

    def on_button_click_stop(self):
        self.mqtt_client.client.publish("jetson/command", "Idle")

    def on_button_click_stop_control(self):
        self.mqtt_client.client.publish("jetson/command", "Stop_control")

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.example_button = tk.Button(
            self,
            text="ELEVATOR",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_elevator,
        )
        #self.example_button.place(x=390, y=385, width=243, height=74)  # (Centered under logo)
        self.example_button.place(x=690, y=150, width=243, height=74)
        
        self.example_button = tk.Button(
            self,
            text="CONTROL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_control,
        )
        self.example_button.place(x=690, y=235, width=243, height=74)

        self.example_button = tk.Button(
            self,
            text="STOP",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_stop_control,
        )
        self.example_button.place(x=933, y=235, width=70, height=74)

        self.example_button = tk.Button(
            self,
            text="HORIZONTAL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_horizontal,
        )
        self.example_button.place(x=690, y=320, width=243, height=74)

        self.example_button = tk.Button(
            self,
            text="STOP BALL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_stop,
        )
        self.example_button.place(x=690, y=405, width=243, height=74)

        self.title_label = tk.Label(
            self,
            text="Dev Tools",
            font=("Jockey One", 40),   # or any font you prefer
            fg="#EE3229",                # text color
            bg="#D9D9D9"                 # background (or match your image if needed)
        )
        self.title_label.place(x=410, y=340)        

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
