import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class InfoScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.create_widgets()
        self.show_info_message()

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

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.add_essential_buttons()

        # Info text frame
        self.info_frame = tk.Frame(self, bg="#FFFFFF", bd=2, relief="sunken")
        self.info_frame.place(x=200, y=200, width=600, height=300)

        self.scrollbar = tk.Scrollbar(self.info_frame)
        self.scrollbar.pack(side="right", fill="y")

        self.info_text = tk.Text(
            self.info_frame,
            wrap="word",
            font=("Jockey One", 14),
            fg="#1A1A1A",
            bg="#F7F7F7",
            yscrollcommand=self.scrollbar.set,
            borderwidth=0,
            highlightthickness=0
        )
        self.info_text.pack(fill="both", expand=True)
        self.scrollbar.config(command=self.info_text.yview)

        # Define text tags for formatting
        self.info_text.tag_configure("title", font=("Jockey One", 20, "bold"), spacing3=10)
        self.info_text.tag_configure("subtitle", font=("Jockey One", 16, "bold"), foreground="#333")
        self.info_text.tag_configure("bullet", lmargin1=15, lmargin2=30)
        self.info_text.tag_configure("highlight", foreground="red", font=("Jockey One", 14, "bold"))
        self.info_text.tag_configure("normal", font=("Jockey One", 14))

        self.info_text.config(state="disabled")

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
            text="INFO",
            font=("Jockey One", 55),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=400, y=100)

    def clear_info_text(self):
        self.info_text.config(state="normal")
        self.info_text.delete("1.0", "end")
        self.info_text.config(state="disabled")

    def insert_formatted_line(self, text, tag="normal"):
        self.info_text.config(state="normal")
        self.info_text.insert("end", text + "\n", tag)
        self.info_text.config(state="disabled")

    def show_info_message(self):
        self.clear_info_text()

        self.insert_formatted_line("System Information", "title")
        self.insert_formatted_line("Ball Maze Controller v1.2", "subtitle")
        self.insert_formatted_line("Status: ", "subtitle")
        self.insert_formatted_line("• Connected to MQTT broker", "bullet")
        self.insert_formatted_line("• Arduino active", "bullet")
        self.insert_formatted_line("• Ball detected", "bullet")

        self.insert_formatted_line("\nWarnings", "subtitle")
        self.insert_formatted_line("• Position drift is high", "highlight")

    def show(self):
        pass
    
    def hide(self):
        self.pack_forget()