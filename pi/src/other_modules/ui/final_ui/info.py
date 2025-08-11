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
        self.info_frame.place(x=112, y=150, width=800, height=400)

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
        self.title.place(x=420, y=20)

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

        self.insert_formatted_line("Project information", "title")
        self.insert_formatted_line("This project aims to create a maze-solving robot", "normal")
        self.insert_formatted_line("that can autonomously balance a ball through the maze.", "normal")
        self.insert_formatted_line("The project was worked on by summer interns at NOV, summer 2025", "normal")
        self.insert_formatted_line("\nProject members:", "subtitle")
        self.insert_formatted_line("• Erlend O. Berge", "bullet")
        self.insert_formatted_line("• Philip A. Haugen", "bullet")
        self.insert_formatted_line("• Lyder K. Jacobsen", "bullet")

        self.insert_formatted_line("How to use", "subtitle")
        self.insert_formatted_line("The robot has several modes of operation", "normal")
        self.insert_formatted_line("\nAutonomous solving", "subtitle")
        self.insert_formatted_line("This is the main mode, allowing for autonomous solving of the maze.", "normal")
        self.insert_formatted_line("Touch anywhere in the image of the maze, and press 'calculate path'", "normal")
        self.insert_formatted_line("The robot reads the maze, computes a path through it, then controls the ball", "normal")
        self.insert_formatted_line("Then chose one of the two types of control: 'fast' or 'safe'", "normal")
        self.insert_formatted_line("Autonomous balancing of the ball through the maze will then start shortly.", "normal")

    def show(self):
        pass
    
    def hide(self):
        self.pack_forget()