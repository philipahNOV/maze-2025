import csv
import os
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEADERBOARD_FOLDER = os.path.join(SCRIPT_DIR, "..", "data", "leaderboard_data")

def get_leaderboard_file(maze_id: int):
    return os.path.join(LEADERBOARD_FOLDER, f"leaderboard_maze{maze_id}.csv")

def add_score(name: str, time: float, maze_id: int, mqtt_client=None):
    filepath = get_leaderboard_file(maze_id)
    date_str = datetime.now().strftime("%d-%m-%Y")
    os.makedirs(LEADERBOARD_FOLDER, exist_ok=True)
    
    if os.path.exists(filepath):
        with open(filepath, "rb") as f:
            f.seek(-1, 2)
            last_char = f.read(1)
            needs_newline = last_char != b'\n'
    else:
        needs_newline = False
    
    with open(filepath, mode="a", newline="") as f:
        if needs_newline:
            f.write('\n')
        writer = csv.writer(f)
        writer.writerow([name, f"{time:.2f}", date_str, maze_id])
    
    if mqtt_client is not None:
        send_leaderboard_data(mqtt_client, maze_id)

def send_leaderboard_data(mqtt_client, maze_id: int):
    try:
        leaderboard_data = read_leaderboard(maze_id)
        csv_data = []
        for entry in leaderboard_data:
            csv_data.append(f"{entry['name']},{entry['time']:.2f},{entry['date']},{entry['maze_id']}")
        
        csv_string = '\n'.join(csv_data)
        
        # send via MQTT
        mqtt_client.client.publish(f"pi/leaderboard_data/{maze_id}", csv_string)
    except Exception as e:
        pass

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