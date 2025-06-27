import tkinter as tk
from PIL import Image, ImageTk
from tkinter import font as tkfont

import time


class Tuning(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.params = None
        self.params_recieved = False

        self.input_frame = tk.Frame(self)
        self.input_frame.place(x=100, y=100)

        self.image = ImageTk.PhotoImage(Image.open('../data/start_screen.png'))

        # Layout the widgets including the logo
        self.create_widgets()

    #def on_button_click_elevator(self):
        #self.mqtt_client.client.publish("jetson/command", "Elevator")

    def create_input(self, parent, label, row, col, param_index):
        tk.Label(parent, text=label, font=("Jockey One", 14)).grid(row=row, column=col, sticky="e", padx=5, pady=5)
        entry = tk.Entry(parent, font=("Jockey One", 14), width=10)
        entry.grid(row=row, column=col+1, padx=5, pady=5)

        reset_btn = tk.Button(
            parent,
            text="↺",  # or "Reset"
            font=("Jockey One", 12),
            width=2,
            command=lambda idx=param_index: self.load_params(self.params, idx)
        )
        reset_btn.grid(row=row, column=col+2, padx=2)

        return entry
    
    def poll_for_params(self):
        if self.params and not self.params_recieved:
            self.params_recieved = True
            self.load_params(self.params, -1)
        else:
            self.after(200, self.poll_for_params)  # Try again in 200ms
    
    def load_params(self, params, index):
        entries = [self.entry1,
            self.entry2,
            self.entry3,
            self.entry4,
            self.entry5,
            self.entry6,
            self.entry7,
            self.entry8
        ]
        if index == -1:
            for i in range(len(entries)):
                entries[i].insert(0, params[i])
        else:
            entries[index].insert(0, params[index])
    
    def handle_submit(self):
        entries = [self.entry1.get(),
            self.entry2.get(),
            self.entry3.get(),
            self.entry4.get(),
            self.entry5.get(),
            self.entry6.get(),
            self.entry7.get(),
            self.entry8.get()
        ]
        command_string = "PID:"
        for entry in entries:
            if entry == "":
                command_string += ",pass"
            else:
                try:
                    val = float(entry)
                    command_string += f",{val}"
                except ValueError:
                    command_string += ",pass"

        self.mqtt_client.client.publish("jetson/command", command_string)


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

        

        self.entry1 = self.create_input(self.input_frame, "x offset", 0, 0, 0)
        self.entry2 = self.create_input(self.input_frame, "y offset:", 1, 0, 1)
        self.entry3 = self.create_input(self.input_frame, "Kp x:", 2, 0, 2)
        self.entry4 = self.create_input(self.input_frame, "Kp y:", 3, 0, 3)
        self.entry5 = self.create_input(self.input_frame, "Kd x:", 4, 0, 4)
        self.entry6 = self.create_input(self.input_frame, "Kd y:", 5, 0, 5)
        self.entry7 = self.create_input(self.input_frame, "Ki x:", 6, 0, 6)
        self.entry8 = self.create_input(self.input_frame, "Ki y:", 7, 0, 7)

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
        self.submit_button.grid(row=8, column=0, columnspan=2, pady=10)

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

        self.start_screen_button = tk.Button(
            self,
            text="DEV TOOLS",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",           
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=lambda: self.controller.show_frame("Screen1")
        )
        self.start_screen_button.place(x=690, y=65, width=243, height=74)

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)
        self.mqtt_client.publish("jetson/command", "Get_pid")
        self.poll_for_params()

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
