import csv
import os
from datetime import datetime

LEADERBOARD_FOLDER = "./leaderboard_data"

def get_leaderboard_file(maze_id: int):
    return os.path.join(LEADERBOARD_FOLDER, f"leaderboard_maze{maze_id}.csv")

def add_score(name: str, time: float, maze_id: int):
    filepath = get_leaderboard_file(maze_id)
    date_str = datetime.now().strftime("%Y-%m-%d")
    os.makedirs(LEADERBOARD_FOLDER, exist_ok=True)
    
    with open(filepath, mode="a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([name, f"{time:.2f}", date_str, maze_id])

def read_leaderboard(maze_id: int):
    filepath = get_leaderboard_file(maze_id)
    if not os.path.exists(filepath):
        return []

    with open(filepath, mode="r", newline="") as f:
        reader = csv.reader(f)
        entries = []
        for row in reader:
            if len(row) == 4:
                name, time_str, date_str, maze_str = row
                try:
                    entries.append({
                        "name": name,
                        "time": float(time_str),
                        "date": date_str,
                        "maze_id": int(maze_str)
                    })
                except ValueError:
                    continue
        return sorted(entries, key=lambda x: x["time"])  # fastest first
