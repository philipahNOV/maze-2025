import tkinter as tk
from PIL import Image, ImageTk
import os
import cv2
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainApp

class PlayVsAIScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        self.scale_ratio = controller.config['camera'].get('maze_image_scale', 0.65)
        self.true_width = controller.config['camera'].get('maze_width', 730)
        self.true_height = controller.config['camera'].get('maze_height', 640)
        self.current_turn = "waiting"
        self.pid_result = None
        self.human_result = None
        self.tracking_ready = False
        self.ball_detected = False
        self.game_started = False
        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.background_label = tk.Label(self, image=self.background_image)
        self.background_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.create_widgets()
        self.update_image()

    def create_widgets(self):
        self.title_label = tk.Label(
            self,
            text="Player vs Robot Challenge",
            font=("Jockey One", 28, "bold"),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.title_label.place(x=400, y=20)

        self.status_label = tk.Label(
            self,
            text="Click on the maze to set a goal, then click 'Start battle'!",
            font=("Jockey One", 18),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.status_label.place(x=200, y=80)

        self.results_frame = tk.Frame(self, bg="#D9D9D9")
        self.results_frame.place(x=50, y=120, width=300, height=200)

        self.pid_result_label = tk.Label(
            self.results_frame,
            text="Robot: Waiting...",
            font=("Jockey One", 16),
            bg="#D9D9D9",
            fg="#1A1A1A",
            anchor="w"
        )
        self.pid_result_label.pack(pady=10, fill="x")

        self.human_result_label = tk.Label(
            self.results_frame,
            text="Player: Waiting...",
            font=("Jockey One", 16),
            bg="#D9D9D9",
            fg="#1A1A1A",
            anchor="w"
        )
        self.human_result_label.pack(pady=10, fill="x")

        self.winner_label = tk.Label(
            self.results_frame,
            text="",
            font=("Jockey One", 20, "bold"),
            bg="#D9D9D9",
            fg="#2E8B57",
            anchor="center"
        )
        self.winner_label.pack(pady=20, fill="x")

        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        available_width = screen_width - 400
        available_height = screen_height - 160
        
        width_scale = available_width / self.true_width
        height_scale = available_height / self.true_height
        max_scale = min(width_scale, height_scale, self.scale_ratio)
        
        self.canvas_width = int(self.true_width * max_scale)
        self.canvas_height = int(self.true_height * max_scale)
        
        self.canvas = tk.Canvas(
            self,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="black",
            highlightthickness=2,
            highlightbackground="white"
        )
        self.canvas.place(x=380, y=140)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        self.start_battle_button = tk.Button(
            self,
            text="Start battle",
            font=("Jockey One", 20, "bold"),
            bg="#60666C",
            fg="white",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            width=12,
            height=2,
            command=self.start_battle,
            borderwidth=0,
            highlightthickness=0,
            relief="flat"
        )
        self.start_battle_button.place(x=50, y=350)

        self.start_human_button = tk.Button(
            self,
            text="Start turn",
            font=("Jockey One", 18, "bold"),
            bg="#60666C",
            fg="white",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            width=10,
            height=2,
            command=self.start_human_turn,
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            state=tk.DISABLED
        )
        self.start_human_button.place(x=220, y=350)

        self.back_button = tk.Button(
            self,
            text="BACK",
            font=("Jockey One", 20),
            bg="#EE3229",
            fg="white",
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            width=8,
            height=2,
            command=self.go_back,
            borderwidth=0,
            highlightthickness=0,
            relief="flat"
        )
        self.back_button.place(x=857, y=10, width=150, height=50)

    def start_battle(self):
        self.current_turn = "pid"
        self.start_battle_button.config(state=tk.DISABLED)
        self.status_label.config(text="Robot is attempting the maze...")
        self.pid_result_label.config(text="Robot: Running...", fg="#D2691E")
        self.mqtt_client.client.publish("jetson/command", "StartBattle")

    def start_human_turn(self):
        if self.current_turn == "human":
            self.start_human_button.config(state=tk.DISABLED)
            self.status_label.config(text="Player turn started! Control the ball to reach the goal.")
            self.mqtt_client.client.publish("jetson/command", "StartHumanTurn")

    def handle_pid_started(self):
        self.current_turn = "pid"
        self.status_label.config(text="Robot is attempting the maze...")
        self.pid_result_label.config(text="Robot: Running...", fg="#D2691E")

    def handle_pid_result(self, success, duration=None):
        if success:
            self.pid_result_label.config(
                text=f"Robot: SUCCESS ({duration:.2f}s)", 
                fg="#2E8B57"
            )
            self.pid_result = duration
        else:
            self.pid_result_label.config(
                text="Robot: FAILED", 
                fg="#EE3229"
            )
            self.pid_result = None
        
        self.current_turn = "human"
        self.status_label.config(text="Robot finished. Your turn! Click 'Start Turn' when ready.")
        self.start_human_button.config(state=tk.NORMAL)
        self.human_result_label.config(text="Player: Click 'Start Turn' to begin", fg="#60666C")

    def handle_human_started(self):
        self.current_turn = "human"
        self.status_label.config(text="Player turn in progress! Control the ball to reach the goal.")
        self.human_result_label.config(text="Player: Playing...", fg="#D2691E")

    def handle_human_result(self, success, duration=None):
        if success:
            self.human_result_label.config(
                text=f"Human Player: SUCCESS ({duration:.2f}s)", 
                fg="#2E8B57"
            )
            self.human_result = duration
        else:
            self.human_result_label.config(
                text="Human Player: FAILED", 
                fg="#EE3229"
            )
            self.human_result = None
        
        self.current_turn = "completed"
        self.determine_winner()

    def determine_winner(self):
        self.canvas.delete("all")
        self.canvas.create_text(
            self.canvas_width // 2, 
            self.canvas_height // 2, 
            text="Battle Complete", 
            fill="white", 
            font=("Arial", 24)
        )
        
        if self.pid_result is not None and self.human_result is not None:
            if self.pid_result < self.human_result:
                self.winner_label.config(text="Robot wins!", fg="#2E8B57")
                self.status_label.config(text="Robot was faster!")
            else:
                self.winner_label.config(text="Player wins!", fg="#2E8B57")
                self.status_label.config(text="Player was faster!")
        elif self.pid_result is not None:
            self.winner_label.config(text="Robot wins!", fg="#2E8B57")
            self.status_label.config(text="Only robot completed the maze!")
        elif self.human_result is not None:
            self.winner_label.config(text="Player wins!", fg="#2E8B57")
            self.status_label.config(text="Only the player completed the maze!")
        else:
            self.winner_label.config(text="No winner - both failed", fg="#EE3229")
            self.status_label.config(text="Neither player completed the maze!")
        
        self.start_battle_button.config(state=tk.NORMAL)
        self.start_battle_button.config(text="Start New Battle")

    def on_canvas_click(self, event):
        if self.current_turn == "waiting":
            x_ratio = self.true_width / self.canvas_width
            y_ratio = self.true_height / self.canvas_height
            
            maze_x = int(event.x * x_ratio)
            maze_y = int(event.y * y_ratio)
            
            self.mqtt_client.client.publish("jetson/command", f"Goal_set:{maze_x},{maze_y}")
            
            self.canvas.delete("goal_marker")
            self.canvas.create_oval(
                event.x - 5, event.y - 5, event.x + 5, event.y + 5,
                fill="red", outline="white", width=2, tags="goal_marker"
            )
            
            self.status_label.config(text=f"Goal set at ({maze_x}, {maze_y}). Click 'Start battle' to begin!")

    def reset_game_state(self):
        self.current_turn = "waiting"
        self.pid_result = None
        self.human_result = None
        self.tracking_ready = False
        self.ball_detected = False
        self.game_started = False
        
        self.status_label.config(text="Click on the maze to set a goal, then click 'Start battle'!")
        self.pid_result_label.config(text="Robot: Waiting...", fg="#1A1A1A")
        self.human_result_label.config(text="Player: Waiting...", fg="#1A1A1A")
        self.winner_label.config(text="")
        
        self.start_battle_button.config(state=tk.NORMAL, text="Start battle")
        self.start_human_button.config(state=tk.DISABLED)
        
        self.canvas.delete("goal_marker")

    def go_back(self):
        self.reset_game_state()
        self.controller.show_frame("HumanScreen")
        self.mqtt_client.client.publish("jetson/command", "Back")

    def update_image(self):
        if hasattr(self.mqtt_client, 'img') and self.mqtt_client.img is not None:
            try:
                img = self.mqtt_client.img.copy()
                img_resized = cv2.resize(img, (self.canvas_width, self.canvas_height))
                img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
                
                img_pil = Image.fromarray(img_rgb)
                img_tk = ImageTk.PhotoImage(img_pil)
                
                self.canvas.delete("all")
                self.canvas.create_image(
                    self.canvas_width // 2, 
                    self.canvas_height // 2, 
                    image=img_tk
                )
                self.canvas.image = img_tk
                
            except Exception as e:
                print(f"Error updating image: {e}")
        
        self.after(100, self.update_image)

    def show(self):
        self.reset_game_state()
        self.tkraise()