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
            os.path.join(os.getcwd(), "..", "..", "jetson", "src", "data", "leaderboard_data")
        )

        self.create_widgets()

    def create_widgets(self):
        self.update()

        bg_label = tk.Label(self, image=self.background_image)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        tk.Label(
            self,
            text="LEADERBOARD",
            font=("Jockey One", 40),
            bg="#D9D9D9",
            fg="#1A1A1A"
        ).place(x=330, y=10)

        columns = ("Ranks","Name", "Time", "Date", "Maze")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=15)

        for col in columns:
            if col == "Rank":
                width = 70
            elif col == "Name":
                width = 180
            elif col == "Time":
                width = 100
            elif col == "Date":
                width = 120
            else:  # Maze
                width = 80
            self.tree.heading(col, text=col)
            self.tree.column(col, width=width, anchor="center")

        tree_x = 100
        tree_y = 100
        tree_width = sum([70, 180, 100, 120, 80])  # Sum of all column widths = 550

        self.tree.place(x=tree_x, y=tree_y)

        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.place(x=tree_x + tree_width, y=tree_y, height=325)

        self.maze_toggle_button = tk.Button(
            self,
            text="SWITCH MAZE",
            font=("Jockey One", 20),
            fg="white",
            bg="#60666C",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.toggle_maze
        )
        self.maze_toggle_button.place(x=380, y=460, width=260, height=50)

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
        self.maze_toggle_button.config(text=f"SWITCH MAZE")
        self.mqtt_client.client.publish("jetson/command", f"SendLeaderboard:{new_id}")
        self.load_leaderboard(new_id)

    def load_leaderboard(self, maze_id: int):
        self.tree.delete(*self.tree.get_children())

        file_path = os.path.join(self.leaderboard_dir, f"leaderboard_maze{maze_id}.csv")
        if not os.path.exists(file_path):
            return

        entries = []
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                if len(row) == 4:
                    name, time_str, date_str, maze_str = row
                    try:
                        entries.append((name, float(time_str), date_str, maze_str))
                    except ValueError:
                        continue

        entries.sort(key=lambda x: x[1])

        for idx, row in enumerate(entries, start=1):
            self.tree.insert("", "end", values=(idx, *row))

    def update_leaderboard_data(self, maze_id: int, csv_data: str):
        try:
            current_maze_id = self.controller.config.get("maze_id", 1)
            if maze_id != current_maze_id:
                return
            
            self.tree.delete(*self.tree.get_children())
            entries = []
            if csv_data.strip():
                lines = csv_data.strip().split('\n')
                for line in lines:
                    if line.strip():
                        parts = line.split(',')
                        if len(parts) == 4:
                            name, time_str, date_str, maze_str = parts
                            if maze_str == "1":
                                maze_str = "Hard"
                            elif maze_str == "2":
                                maze_str = "Easy"
                                
                            try:
                                entries.append((name, float(time_str), date_str, maze_str))
                            except ValueError:
                                continue
            
            entries.sort(key=lambda x: x[1])
            for idx, row in enumerate(entries, start=1):
                self.tree.insert("", "end", values=(idx, *row))
                
            print(f"[LEADERBOARD UI] Updated display with {len(entries)} entries for maze {maze_id}")
        except Exception as e:
            print(f"[LEADERBOARD UI] Failed to update leaderboard data: {e}")

    def show(self):
        self.focus_set()
        maze_id = self.controller.config.get("maze_id", 1)
        self.maze_toggle_button.config(text=f"SWITCH MAZE")
        self.load_leaderboard(maze_id)

    def on_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")