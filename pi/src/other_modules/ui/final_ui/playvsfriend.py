import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from utils.on_screen_keyboard import OnScreenKeyboard

class PlayVsFriendScreen(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.name_entries = []

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.create_widgets()

    def create_widgets(self):
        if self.background_image:
            self.bg_label = tk.Label(self, image=self.background_image)
            self.bg_label.image = self.background_image
            self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        tk.Label(
            self,
            text="PLAY VS FRIEND",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=300, y=30)

        self.back_button = tk.Button(
            self,
            text="←",
            font=("Jockey One", 26),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.go_back
        )
        self.back_button.place(x=844, y=10, width=50, height=50)

        # Dropdown for selecting number of players
        tk.Label(self, text="Select number of players:", font=("Jockey One", 20), bg="#D9D9D9").place(x=320, y=100)
        self.player_count_var = tk.IntVar(value=2)
        self.dropdown = ttk.Combobox(self, textvariable=self.player_count_var, values=[2, 3, 4, 5], font=("Jockey One", 18), state="readonly")
        self.dropdown.place(x=570, y=100, width=60)
        self.dropdown.bind("<<ComboboxSelected>>", lambda e: self.render_name_inputs())

        self.start_button = tk.Button(
            self,
            text="START",
            font=("Jockey One", 26),
            fg="white",
            bg="gray",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.start_game,
            state="disabled"
        )
        self.start_button.place(x=391, y=460, width=243, height=74)

        self.render_name_inputs()

    def render_name_inputs(self):
        # Clear existing entries
        for entry in self.name_entries:
            entry.destroy()
        self.name_entries.clear()

        count = self.player_count_var.get()
        for i in range(count):
            entry = tk.Entry(self, font=("Jockey One", 20), justify="center")
            entry.place(x=320, y=160 + i * 60, width=360, height=40)
            entry.bind("<Button-1>", lambda e, target=entry: self.open_keyboard(target))
            entry.bind("<KeyRelease>", lambda e: self.validate_names())
            self.name_entries.append(entry)

        self.validate_names()

    def open_keyboard(self, entry_widget):
        OnScreenKeyboard(self, entry_widget)

    def validate_names(self):
        if all(e.get().strip() for e in self.name_entries):
            self.start_button.config(state="normal", bg="#60666C")
        else:
            self.start_button.config(state="disabled", bg="gray")

    def start_game(self):
        names = [entry.get().strip() for entry in self.name_entries]
        print("Starting game with players:", names)
        # TODO: Send MQTT or transition to gameplay

    def go_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")