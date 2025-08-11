
import tkinter as tk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp
import os

class BootScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.image_paths = [
            os.path.join(controller.image_path, 'boot_image_1.png'),
            os.path.join(controller.image_path, 'boot_image_2.png'),
            os.path.join(controller.image_path, 'boot_image_3.png')
        ]
        self.images = [ImageTk.PhotoImage(Image.open(path)) for path in self.image_paths]
        self.image_index = 0

        self.animation_image_paths = []
        for i in range(1, 5):
            self.animation_image_paths.append(os.path.join(self.controller.booting_animation_path, f"{i}.png"))
        self.animation_images = [
            ImageTk.PhotoImage(
                Image.open(path).convert("RGBA").resize(
                    (200, 200),
                    Image.Resampling.LANCZOS
                )
            )
            for path in self.animation_image_paths
        ]
        self.animation_image_index = 0

        self.create_widgets()
        self.cycle_images()
        self.check_pi_state()
        self.mqtt_client.initiate_handshake()
        self.cycle_animation_images()

    def cycle_animation_images(self):
        self.image_label.configure(image=self.animation_images[self.animation_image_index])
        self.animation_image_index = (self.animation_image_index + 1) % len(self.animation_images)
        self.after(int(1000 / 30), lambda: self.after_idle(self.cycle_animation_images))

    def on_button_click_exit(self):
        if self.mqtt_client.handshake_complete and self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def on_button_click_restart(self):
        if self.mqtt_client.handshake_complete:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def create_widgets(self):
        self.bg_label = tk.Label(self, image=self.images[self.image_index])
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.image_label = tk.Label(self)
        self.image_label.place(x=412, y=350, width=200, height=200)

        self.exit_button = tk.Button(
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

    def cycle_images(self):
        self.image_index = (self.image_index + 1) % len(self.images)
        self.bg_label.configure(image=self.images[self.image_index])
        self.after(500, self.cycle_images)

    def check_pi_state(self):
        if self.mqtt_client.handshake_complete:
            self.mqtt_client.client.publish("jetson/command", "booted")
            self.controller.show_frame("MainScreen")
            self.controller.start_alive_reciever()
        else:
            self.after(200, self.check_pi_state)

    def show(self):
        #self.pack(expand=True, fill=tk.BOTH)
        self.check_pi_state()

    def hide(self):
        self.pack_forget()
        self.grid_remove()