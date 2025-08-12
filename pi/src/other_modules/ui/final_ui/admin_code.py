import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class AdminCodeScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.active_field = None

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.create_widgets()

    def on_button_click_back(self):
        self.controller.show_frame("MainScreen")

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.back_button = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 20),
            fg="white",
            bg="#EE3229",           
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back
        )
        self.back_button.place(x=744, y=10, width=150, height=50)

        self.title = tk.Label(
            self,
            text="ENTER CODE",
            font=("Jockey One", 35),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=420, y=70)
        
        self.code_entry = tk.Entry(
            self,
            font=("Jockey One", 30),
            justify="center"
        )
        self.code_entry.place(x=432, y=500, width=160, height=50)

        self.submit_offsets_button = tk.Button(
            self,
            text="ENTER",
            font=("Jockey One", 15),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.submit_offsets
        )
        self.submit_offsets_button.place(x=491, y=400, width=100, height=50)

        # Digit Buttons (keypad-style)
        button_font = ("Jockey One", 14)
        start_x, start_y = 397, 120
        btn_w, btn_h = 100, 80
        pad = 10

        digits = [
            ("1", 0, 0), ("2", 1, 0), ("3", 2, 0),
            ("4", 0, 1), ("5", 1, 1), ("6", 2, 1),
            ("7", 0, 2), ("8", 1, 2), ("9", 2, 2),
            ("-", 0, 3), ("0", 1, 3), (".", 2, 3),
            ("⌫", 1, 4), ("C", 2, 4)
        ]

        for text, col, row in digits:
            btn = tk.Button(
                self,
                text=text,
                font=button_font,
                width=2,
                command=lambda t=text: self.append_digit(t)
            )
            btn.place(x=start_x + col * (btn_w + pad), y=start_y + row * (btn_h + pad), width=btn_w, height=btn_h)

    def submit_offsets(self):
            code = self.code_entry.get()
            if code == "2000":
                self.controller.show_frame("AdminToolsScreen")
                self.mqtt_client.client.publish("jetson/command", "AdminTools")

    def append_digit(self, char):
        current = self.code_entry.get()

        if char == "C":
            self.code_entry.delete(0, tk.END)
        elif char == "⌫":
            self.code_entry.delete(len(current) - 1, tk.END)
        else:
            self.code_entry.insert(tk.END, char)

    def show(self):
        pass
    
    def hide(self):
        self.pack_forget()