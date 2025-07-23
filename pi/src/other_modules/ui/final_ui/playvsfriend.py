import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainApp

class PlayVsFriendScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.player_entries = []

        self.create_widgets()

    def create_widgets(self):
        self.update()

        bg_label = tk.Label(self, image=self.background_image)
        bg_label.image = self.background_image
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        # Title
        tk.Label(
            self,
            text="PLAY VS FRIEND",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=300, y=30)

        # Player count dropdown
        tk.Label(
            self,
            text="Number of Players:",
            font=("Jockey One", 20),
            bg="#D9D9D9"
        ).place(x=340, y=120)

        self.player_count_var = tk.StringVar(value="2")
        self.dropdown = ttk.Combobox(
            self,
            textvariable=self.player_count_var,
            values=[str(i) for i in range(2, 6)],
            state="readonly",
            font=("Jockey One", 16),
            width=5
        )
        self.dropdown.place(x=580, y=125)
        self.dropdown.bind("<<ComboboxSelected>>", self.update_player_fields)

        # Frame for player name inputs
        self.input_frame = tk.Frame(self, bg="#D9D9D9")
        self.input_frame.place(x=350, y=170)

        # Start button
        self.start_button = tk.Button(
            self,
            text="START",
            font=("Jockey One", 24),
            fg="white",
            bg="#A0A0A0",
            activeforeground="white",
            activebackground="#4B4C4C",
            borderwidth=0,
            highlightthickness=0,
            state="disabled",
            command=self.start_game
        )
        self.start_button.place(x=420, y=440, width=200, height=60)

        # Back button
        self.back_button = tk.Button(
            self,
            text="←",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back
        )
        self.back_button.place(x=844, y=10, width=50, height=50)

        self.update_player_fields()  # Initialize with 2 players

    def update_player_fields(self, event=None):
        for widget in self.input_frame.winfo_children():
            widget.destroy()
        self.player_entries.clear()

        count = int(self.player_count_var.get())

        for i in range(count):
            label = tk.Label(self.input_frame, text=f"Player {i+1} Name:", font=("Jockey One", 18), bg="#D9D9D9")
            label.grid(row=i, column=0, padx=5, pady=8, sticky='e')
            entry = tk.Entry(self.input_frame, font=("Jockey One", 16), width=18)
            entry.grid(row=i, column=1, padx=5, pady=8)
            entry.bind("<KeyRelease>", self.check_all_fields)
            self.player_entries.append(entry)

        self.check_all_fields()

    def check_all_fields(self, event=None):
        all_filled = all(entry.get().strip() for entry in self.player_entries)
        if all_filled:
            self.start_button.config(state="normal", bg="#60666C")
        else:
            self.start_button.config(state="disabled", bg="#A0A0A0")

    def start_game(self):
        names = [entry.get().strip() for entry in self.player_entries]
        print(f"Starting game with players: {names}")
        # Future: Send this list via MQTT or transition to gameplay screen

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")