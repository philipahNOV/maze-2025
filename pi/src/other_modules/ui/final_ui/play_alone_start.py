import tkinter as tk
from PIL import Image, ImageTk
import os
import cv2
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainApp

class PlayAloneStartScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.scale_ratio = controller.config['camera'].get('maze_image_scale', 0.8)
        self.true_width = controller.config['camera'].get('maze_width', 730)
        self.true_height = controller.config['camera'].get('maze_height', 640)

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.background_label = tk.Label(self, image=self.background_image)
        self.background_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.create_widgets()
        self.update_image()

    def create_widgets(self):
        tk.Label(
            self,
            text="Timer starts when the joystick is moved and actuators begin",
            font=("Jockey One", 18),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=200, y=20)

        self.canvas = tk.Canvas(self, width=int(self.true_width * self.scale_ratio),
                                height=int(self.true_height * self.scale_ratio))
        self.canvas.place(x=150, y=80)

        placeholder = ImageTk.PhotoImage(Image.new("RGB", (1, 1), (0, 0, 0)))
        self.image = placeholder
        self.image_id = self.canvas.create_image(0, 0, anchor="nw", image=self.image)

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
        self.back_button.place(x=744, y=10, width=150, height=50)

        self.add_essential_buttons()

    def update_image(self):
        if self.mqtt_client.img is not None:
            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
        else:
            img = Image.open(self.controller.blank_image_path).convert("RGB")

        img_scaled = img.resize(
            (int(self.true_width * self.scale_ratio), int(self.true_height * self.scale_ratio)),
            Image.Resampling.LANCZOS
        )
        self.image = ImageTk.PhotoImage(img_scaled)
        self.canvas.itemconfig(self.image_id, image=self.image)

        self.after(200, self.update_image)


    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")

    def add_essential_buttons(self):
        exit_btn = tk.Button(
            self,
            text="✖",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_exit
        )
        exit_btn.place(x=964, y=10, width=50, height=50)

        restart_btn = tk.Button(
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
        restart_btn.place(x=904, y=10, width=50, height=50)

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def on_button_click_restart(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def hide(self):
        self.pack_forget()