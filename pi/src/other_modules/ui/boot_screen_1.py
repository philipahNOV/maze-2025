
import tkinter as tk
from PIL import Image, ImageTk
from main import MainApp
import os

class BootScreen(tk.Frame):
    def __init__(self, parent, controller: MainApp, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        # Image cycling state
        self.image_paths = [
            os.path.join(controller.image_path, 'boot_image_1.png'),
            os.path.join(controller.image_path, 'boot_image_2.png'),
            os.path.join(controller.image_path, 'boot_image_3.png')
        ]
        self.images = [ImageTk.PhotoImage(Image.open(path)) for path in self.image_paths]
        self.image_index = 0

        # Layout the widgets
        self.create_widgets()

        # Start the periodic image change
        self.cycle_images()

        # Start the periodic check
        self.check_pi_state()

        # Begin MQTT handshake
        self.mqtt_client.initiate_handshake()

    def create_widgets(self):
        # Initialize with the first image
        self.bg_label = tk.Label(self, image=self.images[self.image_index])
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

    def cycle_images(self):
        # Change to next image
        self.image_index = (self.image_index + 1) % len(self.images)
        self.bg_label.configure(image=self.images[self.image_index])
        self.after(500, self.cycle_images)  # Change image every 0.5 seconds

    def check_pi_state(self):
        if self.mqtt_client.pi_state == "0.0":
            self.controller.show_frame("Screen1")
        else:
            self.after(200, self.check_pi_state)  # Check again after 0.2 seconds

    def show(self):
        #self.pack(expand=True, fill=tk.BOTH)
        self.check_pi_state()  # Ensure periodic check restarts

    def hide(self):
        self.pack_forget()
        self.grid_remove()
