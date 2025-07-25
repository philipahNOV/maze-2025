import csv
import os
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEADERBOARD_FOLDER = os.path.join(SCRIPT_DIR, "..", "data", "leaderboard_data")

def get_leaderboard_file(maze_id: int):
    return os.path.join(LEADERBOARD_FOLDER, f"leaderboard_maze{maze_id}.csv")

def add_score(name: str, time: float, maze_id: int):
    filepath = get_leaderboard_file(maze_id)
    date_str = datetime.now().strftime("%d-%m-%Y")
    os.makedirs(LEADERBOARD_FOLDER, exist_ok=True)
    
    if os.path.exists(filepath):
        with open(filepath, "rb") as f:
            f.seek(-1, 2)  # Go to last byte
            last_char = f.read(1)
            needs_newline = last_char != b'\n'
    else:
        needs_newline = False
    
    with open(filepath, mode="a", newline="") as f:
        if needs_newline:
            f.write('\n')  # Add newline if file doesn't end with one
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
        return sorted(entries, key=lambda x: x["time"])