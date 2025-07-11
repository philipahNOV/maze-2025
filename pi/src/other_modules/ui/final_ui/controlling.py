import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main2 import MainApp
import cv2


class ControllingScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self._last_timeout_state = False  # Track the last timeout state
        self.loop = tk.BooleanVar(value=False)

        self.scale_ratio = 0.8
        self.true_width = 730
        self.true_height = 710

        self.loop_on_img = ImageTk.PhotoImage(Image.open(controller.check_true_path))
        self.loop_off_img = ImageTk.PhotoImage(Image.open(controller.check_false_path))

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()
        self.update_image()  # Start updating the image
        self.check_for_timeout()  # Start checking for timeout

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")

    def on_button_click_restart(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def on_button_click_exit(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def check_for_timeout(self):
        if self.mqtt_client.timeout and not self._last_timeout_state:
            print("[DEBUG] New timeout detected — transitioning to MainScreen.")
            self.controller.show_frame("MainScreen")
            self._last_timeout_state = True
            self.mqtt_client.timeout = False
            self.mqtt_client.client.publish("jetson/command", "timeout")

        elif not self.mqtt_client.timeout:
            self._last_timeout_state = False

        self.after(1000, self.check_for_timeout)

    def on_toggle_loop(self):
        current = self.loop.get()
        self.loop.set(not current)
        if self.loop.get():
            self.loop_button.config(image=self.loop_on_img)
            self.mqtt_client.client.publish("jetson/command", "LoopOn")
        else:
            self.loop_button.config(image=self.loop_off_img)
            self.mqtt_client.client.publish("jetson/command", "LoopOff")

    def update_image(self):
        if self.mqtt_client.img is not None:
            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            img_scaled = img.resize(
                (int(self.true_width * self.scale_ratio), int(self.true_height * self.scale_ratio)),
                Image.Resampling.LANCZOS
            )
            imgtk = ImageTk.PhotoImage(image=img_scaled)
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
        self.exit_button.place(x=125, y=10, width=50, height=50)

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.image_label = tk.Label(self)
        self.image_label.place(x=150, y=16, width=self.true_width * self.scale_ratio, height=self.true_height * self.scale_ratio)
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

        self.loop_button = tk.Button(
            self,
            image=self.loop_off_img,
            bd=0,
            command=self.on_toggle_loop,
            bg="#FFFFFF",
            activebackground="#FFFFFF"
        )
        self.loop_button.place(x=45, y=300, width=60, height=60)

        self.loop_label = tk.Label(
            self,
            text="LOOP",
            font=("Jockey One", 16),
            fg="#EE3229",
            bg="#D9D9D9"
        )
        self.loop_label.place(x=50, y=265)

    def show(self):
        """Make this frame visible"""
        self._last_timeout_state = self.mqtt_client.timeout
        self.check_for_timeout()

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
