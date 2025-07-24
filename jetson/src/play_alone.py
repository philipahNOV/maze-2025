import time
from utils.leaderboard_utils import add_score
from camera.tracker_service import TrackerService
from datetime import datetime

def run_playalone_game(self):
    print("[PLAYALONE] Starting tracking thread...")
    maze_id = self.config.get("maze_id", 1)
    player_name = self.config.get("player_name", "Unknown")

    self.TrackerService.start_tracker()

    game_running = True
    start_time = None
    last_valid_pos_time = time.time()

    while game_running:
        ball_pos = self.tracking_service.get_ball_position()

        # Ball not detected for >3s = fail
        if ball_pos is None:
            if start_time and (time.time() - last_valid_pos_time > 3):
                print("[PLAYALONE] Game failed: ball lost > 3 seconds.")
                self.mqtt_client.client.publish("pi/command", "playalone_fail")
                break
        else:
            last_valid_pos_time = time.time()

            if start_time is None:
                print("[PLAYALONE] Ball seen — starting timer.")
                start_time = time.time()

            if self._ball_crossed_goal(ball_pos):
                duration = time.time() - start_time
                print(f"[PLAYALONE] Goal reached in {duration:.2f} sec")
                add_score(player_name, duration, maze_id)
                self.mqtt_client.client.publish("pi/command", f"playalone_success:{duration:.2f}")
                break

        time.sleep(0.1)

    self.tracking_service.stop_tracker()