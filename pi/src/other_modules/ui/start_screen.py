import tkinter as tk
from PIL import Image, ImageTk
from tkinter import font as tkfont

import time


class Screen1(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.motor_speed = 100
        self.dir = None

        self.image = ImageTk.PhotoImage(Image.open('../data/start_screen.png'))

        # Layout the widgets including the logo
        self.create_widgets()

    def on_button_click_elevator(self):
        self.mqtt_client.client.publish("jetson/command", "Elevator")

    def on_button_click_control(self):
        self.mqtt_client.client.publish("jetson/command", "Control")

    def on_button_click_horizontal(self):
        self.mqtt_client.client.publish("jetson/command", "Horizontal")

    def on_button_click_stop(self):
        self.mqtt_client.client.publish("jetson/command", "Idle")

    def on_button_click_stop_control(self):
        self.mqtt_client.client.publish("jetson/command", "Stop_control")

    def on_button_click_motor(self, dir):
        self.dir = dir
        self.mqtt_client.client.publish("jetson/command", "Motor_" + dir + "_" + str(self.motor_speed))

    def on_release(self, event):
        self.dir = None
        self.mqtt_client.client.publish("jetson/command", "Motor_stop")

    def on_speed_change(self, value):
        self.motor_speed = int(value)
        if self.dir:
            self.on_button_click_motor(self.dir)

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.example_button = tk.Button(
            self,
            text="ELEVATOR",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_elevator,
        )
        #self.example_button.place(x=390, y=385, width=243, height=74)  # (Centered under logo)
        self.example_button.place(x=690, y=150, width=243, height=74)

        self.example_button = tk.Button(
            self,
            text="CONTROL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_control,
        )
        self.example_button.place(x=690, y=235, width=243, height=74)

        self.example_button = tk.Button(
            self,
            text="STOP",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_stop_control,
        )
        self.example_button.place(x=933, y=235, width=70, height=74)

        self.example_button = tk.Button(
            self,
            text="HORIZONTAL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_horizontal,
        )
        self.example_button.place(x=690, y=320, width=243, height=74)

        self.example_button = tk.Button(
            self,
            text="STOP BALL",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#60666C",     # Match image color or use transparent if supported
            activebackground="#4B4C4C",  # Match on press
            activeforeground="#DFDFDF",
            command=self.on_button_click_stop,
        )
        self.example_button.place(x=690, y=405, width=243, height=74)

        self.title_label = tk.Label(
            self,
            text="Dev Tools",
            font=("Jockey One", 40),   # or any font you prefer
            fg="#EE3229",                # text color
            bg="#D9D9D9"                 # background (or match your image if needed)
        )
        self.title_label.place(x=410, y=315)

        self.up_button = tk.Button(
            self,
            text="⇧",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.up_button.place(x=175, y=320, width=74, height=74)
        self.up_button.bind("<ButtonRelease-1>", self.on_release)
        self.up_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("up"))

        self.down_button = tk.Button(
            self,
            text="⇩",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.down_button.place(x=175, y=480, width=74, height=74)       
        self.down_button.bind("<ButtonRelease-1>", self.on_release)
        self.down_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("down")) 

        self.left_button = tk.Button(
            self,
            text="⇦",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.left_button.place(x=95, y=400, width=74, height=74)   
        self.left_button.bind("<ButtonRelease-1>", self.on_release)    
        self.left_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("left")) 

        self.right_button = tk.Button(
            self,
            text="⇨",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.right_button.place(x=255, y=400, width=74, height=74)   
        self.right_button.bind("<ButtonRelease-1>", self.on_release)     
        self.right_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("right"))     

        self.speed_slider = tk.Scale(
            self,
            from_=0,
            to=255,
            orient="horizontal",        # or "vertical"
            length=255,
            label="Motor Speed",
            font=("Jockey One", 14),
            bg="#D9D9D9",
            fg="#000000",
            highlightthickness=0,
            troughcolor="#60666C",
            activebackground="#EE3229",
            command=self.on_speed_change  # Called when slider moves
        )
        self.speed_slider.set(100)  # Optional default value
        self.speed_slider.place(x=90, y=220)  

        self.exit_button = tk.Button(
            self,
            text="X",
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
        self.exit_button.place(x=20, y=20, width=50, height=50) 

        self.restart_button = tk.Button(
            self,
            text="RESTART",
            font=("Jockey One", 20),
            fg="white",
            bg="#EE3229",           
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            command=self.controller.restart_program
        )
        self.restart_button.place(x=80, y=80, width=100, height=50)

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
