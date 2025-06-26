import tkinter as tk
import time
import threading
import paho.mqtt.client as mqtt
import base64
import numpy as np
import cv2

class LoadingPath(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        # Layout the widgets
        self.create_widgets()
        self.monitor_state()

    def create_widgets(self):
        # Main instruction label
        main_label = tk.Label(self, text="Instruks:", font=('Helvetica', 25))
        main_label.grid(row=1, column=0, sticky="w", padx=(175, 20), pady=(150, 5))  # Adjusted vertical padding

        back_button = tk.Button(self, text="← Cancel", command=lambda: self.go_back_button_logic())
        back_button.grid(row=0, column=0, sticky="nw", padx=(10, 0), pady=(10, 0))

        # Step labels
        steps = [
            "Loading board"
        ]

        for index, step in enumerate(steps):
            label = tk.Label(self, text=step, font=('Helvetica', 16))
            label.grid(row=index + 2, column=0, sticky="w", padx=(150, 20), pady=(1, 1))  # Reduced vertical padding

        # Configure column widths to allocate extra space to the right side
        self.grid_columnconfigure(0, weight=3)  # Give more weight to the text column
        self.grid_columnconfigure(1, weight=1)  # Smaller weight to the button column

        # Configure row heights to allow for compact vertical layout and central alignment
        self.grid_rowconfigure(0, weight=0)  # Adjust weight for more compact layout
        self.grid_rowconfigure(1, weight=0)  # Adjust weight for more compact layout
        for i in range(2, 7):  # Only the rows with steps
            self.grid_rowconfigure(i, weight=0, pad=1)  # Minimal padding to keep text closer and reduce stretch

    
    def go_back_button_logic(self):
        if self.mqtt_client.jetson_path == "None":
            command = "1.0"
            self.mqtt_client.client.publish("jetson/command", command)
            self.controller.show_frame("NoPath")
        else:
            command = "1.2"
            self.mqtt_client.client.publish("jetson/command", command)
            self.controller.show_frame("Screen2")
       
        print("tried to send command: " + command)

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)
        

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
        self.grid_remove()  

    def monitor_state(self):
        print("LoadingPath: monitor_state() called")
        def check_state():
            while self.controller.get_current_screen() != "LoadingPath":
                time.sleep(1)
                while self.controller.get_current_screen() == "LoadingPath":
                    if self.mqtt_client.pi_state == "1.2":
                        self.controller.show_frame("Screen2")
                    elif self.mqtt_client.pi_state == "0.0":
                        self.controller.show_frame("Screen1")
                    time.sleep(0.2)
        
        # Starting the thread
        threading.Thread(target=check_state, daemon=True).start()
