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
        self.offset_x = controller.config['camera'].get('maze_offset_x', 390)
        self.offset_y = controller.config['camera'].get('maze_offset_y', 10)
        self.current_turn = "waiting"
        self.pid_result = None
        self.human_result = None
        self.tracking_ready = False
        self.ball_detected = False
        self.game_started = False
        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.background_label = tk.Label(self, image=self.background_image)
        self.background_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.goal = None

        self.create_widgets()
        self.canvas_image_id = self.canvas.create_image(
            self.canvas_width // 2,
            self.canvas_height // 2,
            anchor="center",
            image=None
        )
        self.update_image()

    def create_widgets(self):
        self.title_label = tk.Label(
            self,
            text="PLAYER VS ROBOT",
            font=("Jockey One", 28),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.title_label.place(x=400, y=20)

        self.status_label = tk.Label(
            self,
            text="CLICK TO SET GOAL",
            font=("Jockey One", 18),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.status_label.place(x=400, y=80)

        self.results_frame = tk.Frame(self, bg="#D9D9D9")
        self.results_frame.place(x=50, y=120, width=300, height=200)

        self.pid_result_label = tk.Label(
            self.results_frame,
            text="ROBOT - WAITING...",
            font=("Jockey One", 16),
            bg="#D9D9D9",
            fg="#1A1A1A",
            anchor="w"
        )
        self.pid_result_label.pack(pady=10, fill="x")

        self.human_result_label = tk.Label(
            self.results_frame,
            text="PLAYER - WAITING...",
            font=("Jockey One", 16),
            bg="#D9D9D9",
            fg="#1A1A1A",
            anchor="w"
        )
        self.human_result_label.pack(pady=10, fill="x")

        self.winner_label = tk.Label(
            self.results_frame,
            text="",
            font=("Jockey One", 20),
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
            text="START ROBOT",
            font=("Jockey One", 20),
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
            text="START TURN",
            font=("Jockey One", 20),
            bg="#60666C",
            fg="white",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            width=12,
            height=2,
            command=self.start_human_turn,
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            state=tk.DISABLED
        )
        self.start_human_button.place(x=50, y=450)

        self.elevator_button = tk.Button(
            self,
            text="ELEVATOR",
            font=("Jockey One", 20),
            bg="#60666C",
            fg="white",
            activebackground="#4B4C4C",
            activeforeground="#DFDFDF",
            command=self.elevator_pressed,
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
        )
        self.elevator_button.place(x=814, y=450, width=200, height=75)

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
        self.back_button.place(x=864, y=10, width=150, height=50)

    def elevator_pressed(self):
        self.mqtt_client.client.publish("jetson/command", "Elevator")

    def start_battle(self):
        self.current_turn = "pid"
        self.start_battle_button.config(state=tk.DISABLED)
        self.status_label.config(text="ROBOT IS ATTEMPTING THE MAZE...")
        self.pid_result_label.config(text="ROBOT - RUNNING...", fg="#D2691E")
        self.mqtt_client.client.publish("jetson/command", "StartBattle")

        if hasattr(self, 'click_marker') and self.click_marker is not None:
            self.canvas.delete(self.click_marker)
            r = 8
            self.click_marker = self.canvas.create_oval(
                self.goal[0] - r, self.goal[1] - r, self.goal[0] + r, self.goal[1] + r,
                fill="green", outline="white", width=4, tags="goal_marker"
            )

    def start_human_turn(self):
        if self.current_turn == "human":
            self.start_human_button.config(state=tk.DISABLED)
            self.status_label.config(text="PLAYER TURN STARTED - CONTROL THE BALL")
            self.mqtt_client.client.publish("jetson/command", "StartHumanTurn")

    def handle_pid_started(self):
        self.current_turn = "pid"
        self.status_label.config(text="ROBOT IS ATTEMPTING THE MAZE...")
        self.pid_result_label.config(text="ROBOT - RUNNING...", fg="#D2691E")

    def handle_pid_result(self, success, duration=None, failure_reason=None):
        if success:
            self.pid_result_label.config(
                text=f"ROBOT - SUCCESS ({duration:.2f}s)", 
                fg="#2E8B57"
            )
            self.pid_result = duration
            self.current_turn = "human"
            self.status_label.config(text="ROBOT FINISHED - CLICK START TURN")
            self.start_human_button.config(state=tk.NORMAL)
            self.human_result_label.config(text="PLAYER - CLICK START TURN", fg="#60666C")
        else:
            if failure_reason == "no_path":
                self.pid_result_label.config(
                    text="ROBOT - NO PATH FOUND", 
                    fg="#EE3229"
                )
                self.status_label.config(text="PATHFINDING FAILED - SET NEW GOAL")
                self.current_turn = "waiting"
                self.start_battle_button.config(state=tk.NORMAL, text="RETRY MAZE")
            else:
                self.pid_result_label.config(
                    text="ROBOT - FAILED", 
                    fg="#EE3229"
                )
                self.pid_result = None
                self.current_turn = "human"
                self.status_label.config(text="ROBOT FAILED - CLICK START TURN")
                self.start_human_button.config(state=tk.NORMAL)
                self.human_result_label.config(text="PLAYER TURN - CLICK START TURN", fg="#60666C")

    def handle_human_started(self):
        self.current_turn = "human"
        self.status_label.config(text="PLAYER TURN - CONTROL USING JOYSTICK")
        self.human_result_label.config(text="PLAYER - PLAYING...", fg="#D2691E")

    def handle_human_result(self, success, duration=None):
        if success:
            self.human_result_label.config(
                text=f"PLAYER: SUCCESS ({duration:.2f}s)", 
                fg="#2E8B57"
            )
            self.human_result = duration
        else:
            self.human_result_label.config(
                text="PLAYER: FAILED", 
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
            text="MAZE COMPLETE", 
            fill="white", 
            font=("Jockey One", 24)
        )
        
        if self.pid_result is not None and self.human_result is not None:
            if self.pid_result < self.human_result:
                self.winner_label.config(text="ROBOT WINS", fg="#2E8B57")
                self.status_label.config(text="ROBOT WAS FASTER")
            else:
                self.winner_label.config(text="PLAYER WINS", fg="#2E8B57")
                self.status_label.config(text="PLAYER WAS FASTER")
        elif self.pid_result is not None:
            self.winner_label.config(text="ROBOT WINS", fg="#2E8B57")
            self.status_label.config(text="ROBOT COMPLETED MAZE")
        elif self.human_result is not None:
            self.winner_label.config(text="PLAYER WINS", fg="#2E8B57")
            self.status_label.config(text="PLAYER COMPLETED MAZE")
        else:
            self.winner_label.config(text="NO WINNER", fg="#EE3229")
            self.status_label.config(text="MAZE NOT COMPLETED")
        
        self.start_battle_button.config(state=tk.NORMAL)
        self.start_battle_button.config(text="TRY AGAIN")

    def on_canvas_click(self, event):
        if self.current_turn == "waiting":
            x, y = event.x, event.y
            canvas_width = int(self.true_width * self.scale_ratio)
            canvas_height = int(self.true_height * self.scale_ratio)

            if 0 <= x <= canvas_width and 0 <= y <= canvas_height:
                if hasattr(self, 'click_marker') and self.click_marker is not None:
                    self.canvas.delete(self.click_marker)

                r = 5
                self.click_marker = self.canvas.create_oval(
                    event.x - r, event.y - r, event.x + r, event.y + r,
                    fill="red", outline="white", width=2, tags="goal_marker"
                )

                x_ratio = self.true_width / self.canvas_width
                y_ratio = self.true_height / self.canvas_height

                maze_x = self.true_width - int(x * x_ratio) + self.offset_x
                maze_y = self.true_height - int(y * y_ratio) + self.offset_y - 20

                self.goal = (x, y)

                self.mqtt_client.client.publish("jetson/command", f"Goal_set:{maze_x},{maze_y}")
                self.status_label.config(text=f"GOAL SET AT ({maze_x}, {maze_y}) - CLICK START ROBOT")

    def on_canvas_click_old(self, event):
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
            
            self.status_label.config(text=f"GOAL SET AT ({maze_x}, {maze_y}) - CLICK START BATTLE")

    def reset_game_state(self):
        self.current_turn = "waiting"
        self.pid_result = None
        self.human_result = None
        self.tracking_ready = False
        self.ball_detected = False
        self.game_started = False
        
        self.status_label.config(text="CLICK TO SET GOAL - THEN START ROBOT")
        self.pid_result_label.config(text="ROBOT: WAITING...", fg="#1A1A1A")
        self.human_result_label.config(text="PLAYER: WAITING...", fg="#1A1A1A")
        self.winner_label.config(text="")

        self.start_battle_button.config(state=tk.NORMAL, text="START ROBOT")
        self.start_human_button.config(state=tk.DISABLED)

        self.canvas.delete("all")
        self.canvas_image_id = self.canvas.create_image(
            self.canvas_width // 2,
            self.canvas_height // 2,
            anchor="center",
            image=None
        )
        
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
                
                #self.canvas.delete("all")
                self.canvas.itemconfig(self.canvas_image_id, image=img_tk)
                self.canvas.image = img_tk
                
            except Exception as e:
                print(f"Error updating image: {e}")
        
        self.after(100, self.update_image)

    def show(self):
        self.reset_game_state()
        self.tkraise()