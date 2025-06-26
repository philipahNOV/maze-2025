import tkinter as tk
import time
import threading
import paho.mqtt.client as mqtt
import base64
import numpy as np
import cv2

class NeedPath(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.goto_screen3_button = None  # Ensure this is defined here

        # Layout the widgets
        self.create_widgets()

    def create_widgets(self):
        # Add back button in the top left corner
        back_button = tk.Button(self, text="← Back", command=lambda: self.go_back_button_logic("0.0"))
        back_button.grid(row=0, column=0, sticky="nw", padx=(10, 0), pady=(10, 0))

        # Main instruction label
        main_label = tk.Label(self, text="Instruks:", font=('Helvetica', 25))
        main_label.grid(row=1, column=0, sticky="w", padx=(175, 20), pady=(150, 5))  # Adjusted vertical padding

        # Step labels
        steps = [
            "1. Fjern ball fra brett.",
            "2. Bytt brett.",
            "3. Trykk scan og vent.",
            "4. Legg tilbake ball.",
            "5. Trykk start."
        ]

        for index, step in enumerate(steps):
            label = tk.Label(self, text=step, font=('Helvetica', 16))
            label.grid(row=index + 2, column=0, sticky="w", padx=(150, 20), pady=(1, 1))  # Reduced vertical padding

        scan_button = tk.Button(self, text="Scan", bg="green", command=lambda: self.scan_button_action("1.1"))
        scan_button.grid(row=2, column=2, rowspan=5, sticky="nsew", padx=(10, 10), pady=(10, 10))

        # Configure column widths to allocate extra space to the right side
        self.grid_columnconfigure(0, weight=3)  # Give more weight to the text column
        self.grid_columnconfigure(1, weight=1)  # Smaller weight to the button column

        # Configure row heights to allow for compact vertical layout and central alignment
        self.grid_rowconfigure(0, weight=0)  # Adjust weight for more compact layout
        self.grid_rowconfigure(1, weight=0)  # Adjust weight for more compact layout
        for i in range(2, 7):  # Only the rows with steps
            self.grid_rowconfigure(i, weight=0, pad=1)  # Minimal padding to keep text closer and reduce stretch

    def go_back_button_logic(self, command):
        self.mqtt_client.client.publish("jetson/command", command)
        self.controller.show_frame("Screen1")
        print("tried to send command: " + command)

    def scan_button_action(self, command):
        self.mqtt_client.client.publish("jetson/command", command)
        time_slept = 0.0

        print("should be in state 1.1 now")
        self.controller.show_frame("LoadingPath")
       

    def show(self):
        """Make this frame visible and start monitoring state"""
        self.grid()  # Use grid instead of pack
        

    def hide(self):
        """Hide this frame"""
        self.grid_remove()  
