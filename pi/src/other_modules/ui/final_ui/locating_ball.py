import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main2 import MainApp


class LocatingScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.custom = False

        self.image_paths = []
        for i in range(1, 27):
            self.image_paths.append(os.path.join(self.controller.loading_animation_path, f"{i}.png"))
        self.images = [
            ImageTk.PhotoImage(
                Image.open(path).resize(
                    (100, 100),
                    Image.Resampling.LANCZOS
                )
            )
            for path in self.image_paths
        ]
        self.image_index = 0

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()
        self.cycle_images()  # Start cycling images

    def cycle_images(self):
        # Change to next image
        self.image_index = (self.image_index + 1) % len(self.images)
        self.image_label.configure(image=self.images[self.image_index])
        self.after(1000//20, self.cycle_images)  # Change image every 0.05 seconds

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")

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
        self.image_label.place(x=462, y=400, width=100, height=100)
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

        self.title = tk.Label(
            self,
            text="LOCATING BALL",
            font=("Jockey One", 55),   # or any font you prefer
            fg="#1A1A1A",                # text color
            bg="#D9D9D9"                 # background (or match your image if needed)
        )
        self.title.place(x=320, y=180)

        self.under_title = tk.Label(
            self,
            text="PLACE BALL IN ELEVATOR",
            font=("Jockey One", 30),   # or any font you prefer
            fg="#1A1A1A",                # text color
            bg="#D9D9D9"                 # background (or match your image if needed)
        )
        self.under_title.place(x=480, y=290)

    def check_for_ball_2(self):
        if self.mqtt_client.ball_found:
            if self.custom:
                self.mqtt_client.client.publish("jetson/command", "CustomPath")
                self.mqtt_client.ball_found = False
                self.controller.show_frame("CustomPathScreen")
            else:
                self.mqtt_client.client.publish("jetson/command", "AutoPath")
                self.mqtt_client.ball_found = False
                self.controller.show_frame("AutoPathScreen")
        else:
            self.after(200, self.check_for_ball)  # Check again after 0.2 seconds

    def check_for_ball(self):
        if self.mqtt_client.ball_found:
            self.mqtt_client.client.publish("jetson/command", "BallFound")
            self.mqtt_client.ball_found = False
            self.controller.show_frame("CustomPathScreen")
        else:
            self.after(200, self.check_for_ball)  # Check again after 0.2 seconds

    def show(self):
        """Make this frame visible"""
        self.mqtt_client.ball_found = False
        self.check_for_ball()  # Start checking for the ball

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
