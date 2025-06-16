import tkinter as tk
from tkinter import Frame, Label, Button
import threading
import time
from PIL import Image, ImageTk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
class Screen3(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.stop_thread = False
        self.data = {'ball_pos_x': [], 'ball_pos_y': [], 'target_pos_x': [], 'target_pos_y': []}  # Store data for the graph


        # Layout the widgets
        self.create_widgets()
        self.start_update_camera_feed_thread()

    def start_update_camera_feed_thread(self):
        self.video_thread = threading.Thread(target=self.update_camera_feed)
        self.video_thread.start()

    def stop_update_camera_feed_thread(self):
        self.video_thread.join()
    

    def create_widgets(self):
        # Configure grid for entire frame
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=2)
        self.grid_rowconfigure(1, weight=1)  # Adjusted to push content to row 1

        # Create back button
        back_button = Button(self, text="← Back", command=lambda: self.go_back_screen_3_button("1.2"))
        back_button.grid(row=0, column=0, sticky="nw", padx=10, pady=10)  # Positioned at top left

        # Create left and right frames
        left_frame = Frame(self, bg='white')
        right_frame = Frame(self, bg='grey')
        left_frame.grid(row=1, column=0, sticky="nsew")
        right_frame.grid(row=1, column=1, sticky="nsew")

     
        # Adding camera feed and exit button to the right frame
        self.camera_feed = Label(right_frame, text="Camera Feed", bg='black', fg='white')
        exit_button = Button(right_frame, text="Exit Application", command=lambda: self.exit_application("0.0"))
        self.camera_feed.pack(fill='both', expand=True, padx=5, pady=5)
        exit_button.pack(pady=20)

    def go_back_screen_3_button(self, command):
        self.mqtt_client.client.publish("jetson/command", command)
        while self.mqtt_client.pi_state != command:
            time.sleep(0.1)
        self.controller.show_frame("Screen2")
        print("tried to send command: " + command)
    

    def update_camera_feed(self):
        try:
            print("Entered update_camera_feed")
            print(f"Jetson state: {self.mqtt_client.pi_state}")
            while self.mqtt_client.pi_state != "1.3" and not self.stop_thread:
                time.sleep(1)
            while self.mqtt_client.pi_state == "1.3" and not self.stop_thread:
                if self.mqtt_client.img is not None:
                    image_data = np.array(self.mqtt_client.img)
                    img = Image.fromarray(image_data)
                    img_tk = ImageTk.PhotoImage(image=img)
                    self.camera_feed.configure(image=img_tk) # type: ignore
                    self.camera_feed.image = img_tk  # type: ignore # keep a reference to prevent garbage collection
                

                time.sleep(0.1)  # Add a slight delay to avoid overloading the GUI thread
        except Exception as e:
            print(f"Error in update_camera_feed: {e}")

    def exit_application(self, command):
        self.mqtt_client.client.publish("jetson/command", command)
        self.controller.show_frame("Screen1")

    def stop_video_thread(self):
        """Stop the video thread"""
        print("Stopping video thread")
        self.video_thread.join()

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
