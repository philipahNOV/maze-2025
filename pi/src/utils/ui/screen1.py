import tkinter as tk
from PIL import Image, ImageTk
from tkinter import font as tkfont

import time


class Screen1(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.logo_image = Image.open('/home/raspberrypi/nov-maze-2024/pi/data/automaze-high-resolution-logo-transparent.png')

        self.resize_image = self.logo_image.resize((500, 109))
        self.image = ImageTk.PhotoImage(self.resize_image)

        # Layout the widgets including the logo
        self.create_widgets()

    def create_widgets(self):
        self.update()
        # Displaying the logo across the top row from west to east
        self.logo_label = tk.Label(self, image=self.image) # type: ignore
        self.logo_label.image = self.image  # type: ignore # Keep a reference to prevent garbage collection
        self.logo_label.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(int(self.winfo_toplevel().winfo_height() / 5),
                                                                          int(self.winfo_toplevel().winfo_height() / 10)))  # Spans across both columns

        # Configure the column weights to ensure they expand equally
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        helv20 = tkfont.Font(family="Helvetica", size=20, weight="bold")

        # Button 1 in row 2 column 1
        goto_screen2_button1 = tk.Button(self, text="EL. Manuel", font=helv20,
                                         command=lambda: self.button_logic_manuel("2.0"), height=5, width=5)
        goto_screen2_button1.grid(row=1, column=0, sticky="nsew", padx=(20, 10), pady=10)

        # Button 2 in row 2 column 2
        goto_screen2_button2 = tk.Button(self, text="Automatico", font=helv20,
                                         command=lambda: self.button_logic("1.0"), height=5, width=5)
        goto_screen2_button2.grid(row=1, column=1, sticky="nsew", padx=(10, 20), pady=10)

        back_button = tk.Button(self, text="Quit Application", command=lambda: self.go_back_button_logic("-1"))
        back_button.grid(row=0, column=0, sticky="nw", padx=(10, 0), pady=(10, 0))

    def button_logic(self, command):
        self.mqtt_client.client.publish("jetson/command", command)
        if self.mqtt_client.jetson_path == "None":
            self.controller.show_frame("NeedPath")
        else: 
            self.controller.show_frame("Screen2")
        print("tried to send command: " + command)

    def button_logic_manuel(self, command):
        self.mqtt_client.client.publish("jetson/command", command)
        self.controller.show_frame("elManuel")
        print("tried to send command: " + command)

    def go_back_button_logic(self, command):
        self.mqtt_client.client.publish("jetson/command", command)
        self.mqtt_client.shut_down()
        self.controller.on_close()
        exit(0)
        

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
