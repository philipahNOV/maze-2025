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
        self.leaderboard_dir = os.path.abspath(
            os.path.join(os.getcwd(), "..", "..", "..", "jetson", "src", "leaderboard_data")
        )

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

        # Treeview table
        columns = ("Name", "Time", "Date", "Maze")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=15)

        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=200, anchor="center")

        self.tree.place(x=100, y=100)

        # Scrollbar
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.place(x=900, y=100, height=325)

        # Toggle Maze Button
        self.maze_toggle_button = tk.Button(
            self,
            text="Viewing Maze 1",
            font=("Jockey One", 20),
            fg="white",
            bg="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.toggle_maze
        )
        self.maze_toggle_button.place(x=380, y=460, width=260, height=50)

        # Back Button
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
            command=self.on_back
        )
        self.back_button.place(x=844, y=10, width=150, height=50)

    def toggle_maze(self):
        current_id = self.controller.config.get("maze_id", 1)
        new_id = 2 if current_id == 1 else 1
        self.controller.config["maze_id"] = new_id
        self.maze_toggle_button.config(text=f"Viewing Maze {new_id}")
        print(f"[LeaderboardScreen] Switched to maze {new_id}")
        self.load_leaderboard(new_id)

    def load_leaderboard(self, maze_id: int):
        self.tree.delete(*self.tree.get_children())  # Clear existing rows

        file_path = os.path.join(self.leaderboard_dir, f"leaderboard_maze{maze_id}.csv")
        if not os.path.exists(file_path):
            print(f"[LeaderboardScreen] File not found: {file_path}")
            return
        
        print(f"[LeaderboardScreen] Reading from: {file_path}")

        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                if len(row) == 4:
                    name, time_str, date_str, maze_str = row
                    self.tree.insert("", "end", values=(name, time_str, date_str, maze_str))

    def show(self):
        maze_id = self.controller.config.get("maze_id", 1)
        self.maze_toggle_button.config(text=f"Viewing Maze {maze_id}")
        self.load_leaderboard(maze_id)

    def on_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")
