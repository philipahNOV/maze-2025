import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class AdminToolsScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.active_offset_field = None
        self.config_offset_x = controller.config["controller"]["arduino"].get("x_offset", 0.015)
        self.config_offset_y = controller.config["controller"]["arduino"].get("y_offset", 0.0)

        try:
            self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        except Exception as e:
            print("Error loading background image:", e)
            self.background_image = None

        self.create_widgets()

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")

    def on_button_click_restart(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def create_widgets(self):
        self.update()

        if self.background_image:
            self.bg_label = tk.Label(self, image=self.background_image)
            self.bg_label.image = self.background_image  # keep reference
            self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.exit_button = tk.Button(
            self,
            text="✖",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_exit
        )
        self.exit_button.place(x=964, y=10, width=50, height=50)

        self.restart_button = tk.Button(
            self,
            text="⟲",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_restart
        )
        self.restart_button.place(x=904, y=10, width=50, height=50)

        self.back_button = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 28),
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
            text="ADMIN TOOLS",
            font=("Jockey One", 45),  # 1.5x of original 30
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=512, y=100, anchor="n")  # shifted down to keep spacing similar

        self.clear_leaderboard_easy = tk.Button(
            self,
            text="CLEAR EASY LEADERBOARD",
            font=("Jockey One", 15),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.clear_easy_leaderboard
        )
        self.clear_leaderboard_easy.place(x=30, y=200, width=243, height=74)

        self.clear_leaderboard_hard = tk.Button(
            self,
            text="CLEAR HARD LEADERBOARD",
            font=("Jockey One", 15),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.clear_hard_leaderboard
        )
        self.clear_leaderboard_hard.place(x=30, y=285, width=243, height=74)

        self.clear_leaderboard_hard = tk.Button(
            self,
            text="REBOOT",
            font=("Jockey One", 15),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            command=self.reboot_button_click
        )
        self.clear_leaderboard_hard.place(x=30, y=370, width=243, height=74)

        self.clear_leaderboard_hard = tk.Button(
            self,
            text="SHUTDOWN",
            font=("Jockey One", 15),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            command=self.shutdown_button_click
        )
        self.clear_leaderboard_hard.place(x=30, y=455, width=243, height=74)

        # --- X Offset Input ---
        self.x_offset_label = tk.Label(
            self,
            text="X Offset:",
            font=("Jockey One", 15),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.x_offset_label.place(x=391, y=300)
        
        self.x_offset_entry = tk.Entry(
            self,
            font=("Jockey One", 15),
            justify="center"
        )
        self.x_offset_entry.place(x=491, y=300, width=143, height=30)
        self.x_offset_entry.bind("<FocusIn>", lambda e: self.set_active_field('x'))

        # --- Y Offset Input ---
        self.y_offset_label = tk.Label(
            self,
            text="Y Offset:",
            font=("Jockey One", 15),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.y_offset_label.place(x=391, y=350)
        
        self.y_offset_entry = tk.Entry(
            self,
            font=("Jockey One", 15),
            justify="center"
        )
        self.y_offset_entry.place(x=491, y=350, width=143, height=30)
        self.y_offset_entry.bind("<FocusIn>", lambda e: self.set_active_field('y'))

        # --- Submit Button ---
        self.submit_offsets_button = tk.Button(
            self,
            text="SUBMIT",
            font=("Jockey One", 15),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#3C8D2F",
            activebackground="#327727",
            activeforeground="#DFDFDF",
            command=self.submit_offsets
        )
        self.submit_offsets_button.place(x=491, y=400, width=100, height=50)

        self.offset_info_label = tk.Label(
            self,
            text=f"Original offsets → X: {self.config_offset_x:.3f}, Y: {self.config_offset_y:.3f}",
            font=("Jockey One", 14),
            bg="#D9D9D9",
            fg="#333333"
        )
        self.offset_info_label.place(x=491, y=200, anchor="n")  # Just below the title

        self.offset_info_label = tk.Label(
            self,
            text=f"Recommended range: X: 0.01-0.02, Y: 0.00-0.005",
            font=("Jockey One", 14),
            bg="#D9D9D9",
            fg="#333333"
        )
        self.offset_info_label.place(x=491, y=250, anchor="n")  # Just below the title

        # Digit Buttons (keypad-style)
        button_font = ("Jockey One", 14)
        start_x, start_y = 740, 300
        btn_w, btn_h = 50, 40
        pad = 5

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

    def clear_easy_leaderboard(self):
        self.mqtt_client.client.publish("jetson/command", "ClearEasyLeaderboard")

    def clear_hard_leaderboard(self):
        self.mqtt_client.client.publish("jetson/command", "ClearHardLeaderboard")

    def submit_offsets(self):
            x_offset = self.x_offset_entry.get()
            y_offset = self.y_offset_entry.get()
            self.mqtt_client.client.publish("jetson/command", f"SetOffsets:{x_offset},{y_offset}")

    def reboot_button_click(self):
        self.mqtt_client.client.publish("jetson/command", "Reboot")
        self.controller.reboot_pi()

    def reboot_button_click(self):
        self.mqtt_client.client.publish("jetson/command", "Shutdown")
        self.controller.shutdown_pi()


    def append_digit(self, char):
        if self.active_offset_field == 'x':
            entry = self.x_offset_entry
        elif self.active_offset_field == 'y':
            entry = self.y_offset_entry
        else:
            return  # No field selected

        current = entry.get()

        if char == "C":
            entry.delete(0, tk.END)
        elif char == "⌫":
            entry.delete(len(current) - 1, tk.END)
        else:
            entry.insert(tk.END, char)


    def set_active_field(self, field):
        self.active_offset_field = field

    def show(self):
        self.focus_set()
        self.update_idletasks()