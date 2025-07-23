import tkinter as tk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING
from utils.on_screen_keyboard import OnScreenKeyboard

if TYPE_CHECKING:
    from main import MainApp


class PlayVsFriendScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        self.num_players_var = tk.StringVar(value="2")
        self.input_fields = []
        self.input_frame = None

        self.create_widgets()

    def create_widgets(self):
        self.update()

        if self.background_image:
            bg_label = tk.Label(self, image=self.background_image)
            bg_label.image = self.background_image
            bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        tk.Label(self, text="NUMBER OF PLAYERS:", font=("Jockey One", 24), bg="#D9D9D9", fg="#1A1A1A")\
            .place(x=360, y=100)

        dropdown = tk.OptionMenu(self, self.num_players_var, *[str(i) for i in range(2, 6)], command=self.update_input_fields)
        dropdown.config(font=("Jockey One", 20))
        dropdown.place(x=450, y=160)

        self.update_input_fields("2")

        self.start_button = tk.Button(
            self,
            text="START",
            font=("Jockey One", 24),
            fg="white",
            bg="gray",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            width=20,
            height=2,
            state="disabled",
            command=self.start_game
        )
        self.start_button.place(x=355, y=450)

        self.back_button = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 20),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_button_click_back
        )
        self.back_button.place(x=844, y=10, width=50, height=50)

    def update_input_fields(self, selected):
        count = int(selected)

        if self.input_frame:
            self.input_frame.destroy()

        self.input_frame = tk.Frame(self, bg="#D9D9D9")
        self.input_frame.place(x=300, y=240)

        self.input_fields = []
        for i in range(count):
            label = tk.Label(self.input_frame, text=f"Player {i+1}:", font=("Jockey One", 18), bg="#D9D9D9")
            label.grid(row=i, column=0, padx=10, pady=5, sticky='e')

            entry = tk.Entry(self.input_frame, font=("Jockey One", 18), width=20)
            entry.grid(row=i, column=1, padx=10, pady=5)
            entry.bind("<FocusIn>", lambda e, ent=entry: self.show_keyboard(ent))
            entry.bind("<KeyRelease>", lambda e: self.check_start_ready())

            self.input_fields.append(entry)

        self.check_start_ready()

    def check_start_ready(self):
        all_filled = all(entry.get().strip() for entry in self.input_fields)
        if all_filled:
            self.start_button.config(state="normal", bg="#60666C")
        else:
            self.start_button.config(state="disabled", bg="gray")

    def show_keyboard(self, entry):
        OnScreenKeyboard(self, entry)

    def start_game(self):
        names = [entry.get().strip() for entry in self.input_fields]
        print("Starting game with players:", names)
        # Placeholder for FSM or MQTT command to begin game logic

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")