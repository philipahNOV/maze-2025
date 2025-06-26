
import tkinter as tk
from PIL import Image, ImageTk

class BootScreen(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        # Layout the widgets
        self.create_widgets()

        # Start the periodic check
        self.check_pi_state()

        # Begin MQTT handshake
        self.mqtt_client.initiate_handshake()

    def create_widgets(self):
        # Load and display the background boot screen image
        self.bg_image = Image.open("../../../data/boot_image.png")
        self.bg_photo = ImageTk.PhotoImage(self.bg_image)
        self.bg_label = tk.Label(self, image=self.bg_photo)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

    def check_pi_state(self):
        if self.mqtt_client.pi_state == "0.0":
            self.controller.show_frame("Screen1")
        else:
            self.after(1000, self.check_pi_state)  # Check again after 1 second

    def show(self):
        self.pack(expand=True, fill=tk.BOTH)
        self.check_pi_state()  # Ensure periodic check restarts

    def hide(self):
        self.pack_forget()
        self.grid_remove()
