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
        self.disable_buttons()

    def on_button_click_retry(self):
        self.controller.show_frame("LocatingScreen")
        self.mqtt_client.client.publish("jetson/command", "Locate")
        self.mqtt_client.img = None
        self.disable_buttons()

    def on_button_click_start(self):
        self.mqtt_client.client.publish("jetson/command", "Start")
        self.controller.show_frame("ControllingScreen")

    def on_button_click_restart(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def enable_buttons(self):
        self.start_button.config(state="normal", bg="#EE3229", activebackground="#B82F27", fg="white", activeforeground="#DFDFDF")
        self.retry_button.config(state="normal", bg="#EE3229", activebackground="#B82F27", fg="white", activeforeground="#DFDFDF")

    def disable_buttons(self):
        self.start_button.config(state="disabled", bg="#723D3A", activebackground="#331E1D", fg="#9E9E9E", activeforeground="#7A7A7A")
        self.retry_button.config(state="disabled", bg="#723D3A", activebackground="#331E1D", fg="#9E9E9E", activeforeground="#7A7A7A")

    def check_for_timeout(self):
        if self.mqtt_client.timeout:
            self.controller.show_frame("MainScreen")
            self.mqtt_client.client.publish("jetson/command", "timeout")
            self.mqtt_client.timeout = False
        else:
            self.after(1000, self.check_for_timeout)

    def update_image(self):
        if self.mqtt_client.img is not None:
            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.image_label.imgtk = imgtk
            self.image_label.config(image=imgtk)
            self.status_label.place_forget()
            self.enable_buttons()
        else:
            blank_image = Image.open(self.controller.blank_image_path).convert("RGB")
            imgtk = ImageTk.PhotoImage(image=blank_image)
            self.image_label.imgtk = imgtk
            self.image_label.config(image=imgtk)
            waiting_text = "FINDING PATH" + "." * (math.floor(self.waiting_phase / 2) + 1)
            self.waiting_phase = (self.waiting_phase + 1) % 6
            self.status_label.config(text=waiting_text)
            self.status_label.place(x=120, y=250, width=300, height=50)
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
            command=self.on_button_click_exit  # or self.controller.destroy
        )
        self.exit_button.place(x=964, y=10, width=50, height=50) 

        self.exit_button = tk.Button(
            self,
            text="⟲",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_restart
        )
        self.exit_button.place(x=904, y=10, width=50, height=50)

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
        self.back_button.place(x=754, y=10, width=150, height=50)

        self.status_label = tk.Label(
            self,
            text="",
            font=("Jockey One", 35),
            fg="white",
            bg="#4D4D4D",  # Match background or make transparent if needed
            anchor="w",    # Left aligned
            justify="left"
        )

        self.retry_button = tk.Button(
            self,
            text="RETRY",
            font=("Jockey One", 20),
            fg="#9E9E9E",
            bg="#723D3A",           
            activebackground="#331E1D",
            activeforeground="#7A7A7A",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            state="disabled",  # Initially disabled
            command=self.on_button_click_retry,
        )
        self.retry_button.place(x=600, y=300, width=150, height=50)

        self.start_button = tk.Button(
            self,
            text="START ROBOT",
            font=("Jockey One", 20),
            fg="#9E9E9E",
            bg="#723D3A",           
            activebackground="#331E1D",
            activeforeground="#7A7A7A",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            state="disabled",  # Initially disabled
            command=self.on_button_click_start,
        )
        self.start_button.place(x=770, y=300, width=200, height=50)


    def show(self):
        """Make this frame visible"""
        self.mqtt_client.img = None  # Reset image to trigger loading state
        self.waiting_phase = 0  # Reset waiting phase for loading state
        #self.check_for_timeout()

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
