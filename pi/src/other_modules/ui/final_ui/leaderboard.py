import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import csv
import os
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainApp


class LeaderboardScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.leaderboard_dir = os.path.join(os.getcwd(), "data")  # Adjust if different

        self.create_widgets()

    def create_widgets(self):
        self.update()

        # Background
        bg_label = tk.Label(self, image=self.background_image)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        # Title
        tk.Label(
            self,
            text="LEADERBOARD",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=330, y=30)

        # Treeview table for leaderboard
        columns = ("Name", "Time (s)", "Date", "Maze")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=15)

        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=200, anchor="center")

        self.tree.place(x=100, y=100)

        # Scrollbar
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.place(x=900, y=100, height=325)

        # Back button
        back_btn = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 20),
            fg="white",
            bg="#EE3229",
            activebackground="#B82F27",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_back
        )
        back_btn.place(x=844, y=10, width=150, height=50)

    def on_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")

    def load_leaderboard(self, maze_id: int):
        self.tree.delete(*self.tree.get_children())  # Clear old data

        file_path = os.path.join(self.leaderboard_dir, f"leaderboard_maze{maze_id}.csv")
        if not os.path.exists(file_path):
            print(f"[LeaderboardScreen] File not found: {file_path}")
            return

        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) == 4:
                    name, time_str, date_str, maze_str = row
                    self.tree.insert("", "end", values=(name, time_str, date_str, maze_str))

    def show(self):
        maze_id = self.controller.config.get("maze_id", 1)
        print(f"[LeaderboardScreen] Showing leaderboard for maze {maze_id}")
        self.load_leaderboard(maze_id)