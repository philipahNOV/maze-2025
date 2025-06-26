import tkinter as tk
from PIL import Image, ImageTk
from tkinter import font as tkfont

import time


class Tuning(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.motor_speed = 100
        self.dir = None

        self.input_frame = tk.Frame(self)
        self.input_frame.place(x=100, y=100)

        self.image = ImageTk.PhotoImage(Image.open('../data/start_screen.png'))

        # Layout the widgets including the logo
        self.create_widgets()

    #def on_button_click_elevator(self):
        #self.mqtt_client.client.publish("jetson/command", "Elevator")

    def create_input(self, parent, label, row, col):
        tk.Label(parent, text=label, font=("Jockey One", 14)).grid(row=row, column=col, sticky="e", padx=5, pady=5)
        entry = tk.Entry(parent, font=("Jockey One", 14), width=25)
        entry.grid(row=row, column=col+1, padx=5, pady=5)
        return entry
    
    def handle_submit(self):
        val1 = self.entry1.get()
        val2 = self.entry2.get()
        val3 = self.entry3.get()

        print(f"Submitted Values:\n  x offset: {val1}\n  y offset: {val2}\n  Kp x: {val3}")


    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.bg_label.lower()

        self.title_label = tk.Label(
            self,
            text="PID Tuning",
            font=("Jockey One", 40),   # or any font you prefer
            fg="#EE3229",                # text color
            bg="#D9D9D9"                 # background (or match your image if needed)
        )
        self.title_label.place(x=410, y=315)

        

        self.entry1 = self.create_input(self.input_frame, "x offset", 0, 0)
        self.entry2 = self.create_input(self.input_frame, "y offset:", 1, 0)
        self.entry3 = self.create_input(self.input_frame, "Kp x:", 2, 0)

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

        self.submit_button = tk.Button(
            self.input_frame,
            text="SUBMIT",
            font=("Jockey One", 16),
            fg="white",
            bg="#4CAF50",
            activebackground="#388E3C",
            activeforeground="white",
            command=self.handle_submit
        )
        self.submit_button.grid(row=3, column=0, columnspan=2, pady=10)

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
        self.restart_button.place(x=80, y=20, width=100, height=50)

        self.tune_pid_button = tk.Button(
            self,
            text="DEV TOOLS",
            font=("Jockey One", 20),
            fg="white",
            bg="#EE3229",           
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=lambda: self.controller.show_frame("Screen1")
        )
        self.tune_pid_button.place(x=600, y=100, width=180, height=60)

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
