import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class NavigationScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.loop = tk.BooleanVar(value=False)
        self.custom = tk.BooleanVar(value=False)

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()

    def on_toggle_loop(self):
        pass

    def on_toggle_custom(self):
        pass

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")

    def on_button_click_horizontal(self):
        self.mqtt_client.client.publish("jetson/command", "Horizontal")

    def on_button_click_elevator(self):
        self.mqtt_client.client.publish("jetson/command", "Elevator")

    def on_button_click_safe(self):
        self.controller.show_frame("LocatingScreen")
        self.controller.frames["LocatingScreen"].custom = self.custom.get()
        if self.loop.get():
            self.mqtt_client.client.publish("jetson/command", "Locate,loop,safe")
        else:
            self.mqtt_client.client.publish("jetson/command", "Locate,dont_loop,safe")

    def on_button_click_speed(self):
        self.controller.show_frame("LocatingScreen")
        self.controller.frames["LocatingScreen"].custom = self.custom.get()
        if self.loop.get():
            self.mqtt_client.client.publish("jetson/command", "Locate,loop,speed")
        else:
            self.mqtt_client.client.publish("jetson/command", "Locate,dont_loop,speed")

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
            command=self.on_button_click_back
        )
        self.back_button.place(x=804, y=10, width=150, height=50)

        self.get_ball_button = tk.Button(
            self,
            text="GET BALL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_elevator
        )
        self.get_ball_button.place(x=30, y=235, width=243, height=74)

        self.horizontal_button = tk.Button(
            self,
            text="HORIZONTAL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_horizontal
        )
        self.horizontal_button.place(x=30, y=320, width=243, height=74)

        self.safe_button = tk.Button(
            self,
            text="SAFE CONTROL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_safe
        )
        self.safe_button.place(x=751, y=235, width=243, height=74)

        self.speed_button = tk.Button(
            self,
            text="SPEED CONTROL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_speed
        )
        self.speed_button.place(x=751, y=320, width=243, height=74)

        self.loop_toggle = tk.Checkbutton(
            self,
            text="LOOP",
            font=("Jockey One", 20),
            variable=self.loop,
            onvalue=True,
            offvalue=False,
            bg="#D9D9D9",
            activebackground="#D9D9D9",
            fg="#1A1A1A",
            selectcolor="#60666C",  # This changes the indicator fill color
            anchor='w',
            justify='left',
            command=self.on_toggle_loop
        )
        self.loop_toggle.place(x=760, y=175, width=200, height=50)  # Adjust position as needed

        self.custom_toggle = tk.Checkbutton(
            self,
            text="CUSTOM GOAL",
            font=("Jockey One", 20),
            variable=self.custom,
            onvalue=True,
            offvalue=False,
            bg="#D9D9D9",
            activebackground="#D9D9D9",
            fg="#1A1A1A",
            selectcolor="#60666C",  # This changes the indicator fill color
            anchor='w',
            justify='left',
            command=self.on_toggle_custom
        )
        self.custom_toggle.place(x=760, y=115, width=200, height=50)  # Adjust position as needed



    def show(self):
        """Make this frame visible"""

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
