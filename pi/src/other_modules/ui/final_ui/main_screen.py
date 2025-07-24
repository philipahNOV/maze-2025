import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class MainScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        # Store image in a persistent instance variable to prevent garbage collection
        try:
            print("Loading background from:", controller.background_path)
            self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        except Exception as e:
            print("Error loading background image:", e)
            self.background_image = None

        self.create_widgets()

    def on_button_click_disco(self):
        self.mqtt_client.client.publish("jetson/command", "Disco")

    def on_button_click_human_mode(self):
        self.mqtt_client.client.publish("jetson/command", "Human")
        self.controller.show_frame("HumanScreen")

    def on_button_click_restart(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def on_button_click_navigation(self):
        self.mqtt_client.client.publish("jetson/command", "Locate")
        self.controller.show_frame("LocatingScreen")

    def on_button_click_info(self):
        self.mqtt_client.client.publish("jetson/command", "Info")
        self.controller.show_frame("InfoScreen")

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def add_essential_buttons(self):
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

        self.exit_button = tk.Button(
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
        self.exit_button.place(x=904, y=10, width=50, height=50)

        self.info_button = tk.Button(
        self,
        text="INFO",
        font=("Jockey One", 26),
        fg="white",
        bg="#EE3229",
        activebackground="#B82F27",
        activeforeground="#DFDFDF",
        borderwidth=0,
        highlightthickness=0,
        relief="flat",
        command=self.on_button_click_info
        )
        self.info_button.place(x=734, y=10, width=150, height=50)

    def create_widgets(self):
        self.update()  # ensure layout updates

        if self.background_image:
            self.bg_label = tk.Label(self, image=self.background_image)
            self.bg_label.image = self.background_image  # <- PERSISTENT REFERENCE
            self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.add_essential_buttons()

        self.navigation_button = tk.Button(
            self,
            text="ROBOT MODE",
            font=("Jockey One", 25),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.on_button_click_navigation
        )
        self.navigation_button.place(x=391, y=235, width=243, height=74)

        self.human_button = tk.Button(
        self,
        text="HUMAN MODE",
        font=("Jockey One", 30),
        fg="white",
        borderwidth=0,
        highlightthickness=0,
        background="#60666C",
        activebackground="#4B4C4C",
        activeforeground="#DFDFDF",
        command=self.on_button_click_human_mode
        )
        self.human_button.place(x=391, y=320, width=243, height=74)


        self.disco_button = tk.Button(
            self,
            text="LIGHTS MODE",
            font=("Jockey One", 30),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.on_button_click_disco
        )
        self.disco_button.place(x=391, y=405, width=243, height=74)

        self.title = tk.Label(
            self,
            text="WELCOME",
            font=("Jockey One", 55),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=380, y=100)

    def show(self):
        self.update_idletasks()

    def hide(self):
        self.pack_forget()