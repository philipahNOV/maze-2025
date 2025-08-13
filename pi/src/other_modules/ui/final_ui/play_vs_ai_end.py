import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class PlayvsaiEndScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client


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
        #self.exit_button.place(x=964, y=10, width=50, height=50)

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
        #self.restart_button.place(x=904, y=10, width=50, height=50)

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
        self.back_button.place(x=864, y=10, width=150, height=50)

        self.title = tk.Label(
            self,
            text="BATTLE COMPLETED",
            font=("Jockey One", 45),  # 1.5x of original 30
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=512, y=200, anchor="n")  # shifted down to keep spacing similar

        self.under_title = tk.Label(
            self,
            text="",
            font=("Jockey One", 30),  # 1.5x of original 20
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.under_title.place(x=512, y=280, anchor="n")

        self.under_title2 = tk.Label(
            self,
            text="",
            font=("Jockey One", 30),  # 1.5x of original 20
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.under_title2.place(x=512, y=330, anchor="n")

        self.main_menu_button = tk.Button(
            self,
            text="MAIN MENU",
            font=("Jockey One", 25),
            fg="white",
            borderwidth=0,
            highlightthickness=0,
            background="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.show_main_menu
        )
        self.main_menu_button.place(x=391, y=400, width=243, height=74)

    def set_texts(self, winner=None, loser=None, draw=False, winner_time=None, loser_time=None):
        if draw:
            self.under_title.config(text="No winners!")
            self.under_title2.config(text="Both the player and the robot failed...")
        else:
            self.under_title.config(text=f"{winner} wins in {winner_time} seconds!")
            if loser_time == "-1.00":
                self.under_title2.config(text=f"{loser} lost because they failed.")
            else:
                self.under_title2.config(text=f"{loser} lost with time: {loser_time} seconds.")

    def show_main_menu(self):
        self.mqtt_client.client.publish("jetson/command", "human_screen")
        self.controller.show_frame("HumanScreen")

    def show(self):
        self.focus_set()
        self.update_idletasks()