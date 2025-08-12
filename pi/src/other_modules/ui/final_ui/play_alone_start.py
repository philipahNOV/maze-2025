import tkinter as tk
from PIL import Image, ImageTk
import os
import cv2
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainApp

class PlayAloneStartScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.scale_ratio = controller.config['camera'].get('maze_image_scale', 0.65)
        self.true_width = controller.config['camera'].get('maze_width', 730)
        self.true_height = controller.config['camera'].get('maze_height', 640)
        
        self.tracking_ready = False
        self.ball_detected = False
        self.game_started = False

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.background_label = tk.Label(self, image=self.background_image)
        self.background_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.create_widgets()
        self.update_image()

    def create_widgets(self):
        self.status_label = tk.Label(
            self,
            text="WAITING FOR TRACKING...",
            font=("Jockey One", 18),
            bg="#D9D9D9",
            fg="#1A1A1A"
        )
        self.status_label.place(x=200, y=20)

        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        available_width = screen_width - 300
        available_height = screen_height - 160
        
        width_scale = available_width / self.true_width
        height_scale = available_height / self.true_height
        max_scale = min(width_scale, height_scale, self.scale_ratio)
        
        self.canvas_width = int(self.true_width * max_scale)
        self.canvas_height = int(self.true_height * max_scale)
        
        self.canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height)
        self.canvas.place(x=150, y=80)

        placeholder = ImageTk.PhotoImage(Image.new("RGB", (1, 1), (0, 0, 0)))
        self.image = placeholder
        self.image_id = self.canvas.create_image(0, 0, anchor="nw", image=self.image)

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
        self.back_button.place(x=864, y=10, width=150, height=50)

        self.start_button = tk.Button(
            self,
            text="START GAME",
            font=("Jockey One", 20),
            fg="white",
            bg="#808080",
            activebackground="#4CAF50",
            activeforeground="white",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.on_start_game_click,
            state="disabled"
        )
        self.start_button.place(x=770, y=385, width=200, height=75)

        #self.add_essential_buttons()

    def update_image(self):
        if self.mqtt_client.img is not None:
            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
        else:
            img = Image.open(self.controller.blank_image_path).convert("RGB")

        img_scaled = img.resize(
            (self.canvas_width, self.canvas_height),
            Image.Resampling.LANCZOS
        )
        self.image = ImageTk.PhotoImage(img_scaled)
        self.canvas.itemconfig(self.image_id, image=self.image)

        self.after(200, self.update_image)


    def on_button_click_back(self):
        self.reset_game_state()
        self.clear_all_memory()
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("HumanScreen")

    def add_essential_buttons(self):
        exit_btn = tk.Button(
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
        exit_btn.place(x=964, y=10, width=50, height=50)

        restart_btn = tk.Button(
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
        restart_btn.place(x=904, y=10, width=50, height=50)

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def on_button_click_restart(self):
        self.reset_game_state()
        self.clear_all_memory()
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def on_start_game_click(self):
        button_text = self.start_button.cget("text")

        if button_text in ["GOAL REACHED!", "TRY AGAIN"]:
            self.reset_game_state()
            self.clear_all_memory()
            self.status_label.config(text="WAITING FOR TRACKING...", fg="#1A1A1A")
            self.mqtt_client.client.publish("jetson/command", "RestartPlayAlone")
            return
            
        if self.tracking_ready and self.ball_detected:
            self.game_started = True
            self.start_button.config(state="disabled", bg="#808080", text="GAME STARTED")
            self.status_label.config(text="IN PROGRESS - USE JOYSTICK TO CONTROL THE BALL")
            self.mqtt_client.client.publish("jetson/command", "StartPlayAloneGame")

    def update_tracking_status(self, tracking_ready=False, ball_detected=False):
        self.tracking_ready = tracking_ready
        self.ball_detected = ball_detected
        
        if not self.game_started:
            if tracking_ready and ball_detected:
                self.start_button.config(state="normal", bg="#4CAF50", text="START GAME")
                self.status_label.config(text="BALL DETECTED - PRESS START GAME")
            elif tracking_ready:
                self.start_button.config(state="disabled", bg="#FFA500", text="WAITING FOR BALL")
                self.status_label.config(text="TRACKING READY, WAITING FOR BALL...")
            else:
                self.start_button.config(state="disabled", bg="#808080", text="STARTING TRACKER...")
                self.status_label.config(text="STARTING TRACKING...")

    def reset_game_state(self):
        self.tracking_ready = False
        self.ball_detected = False
        self.game_started = False
        
        self.start_button.config(
            state="disabled", 
            bg="#808080", 
            text="START GAME"
        )
        self.status_label.config(text="WAITING FOR TRACKING...")

    def clear_completion_screen(self):
        self.canvas.delete("all")
        self.image_id = self.canvas.create_image(0, 0, anchor="nw", image=self.image)

    def clear_all_memory(self):
        try:
            if hasattr(self.mqtt_client, 'img'):
                self.mqtt_client.img = None
            
            self.canvas.delete("all")
            
            blank_img = Image.open(self.controller.blank_image_path).convert("RGB")
            blank_scaled = blank_img.resize(
                (self.canvas_width, self.canvas_height),
                Image.Resampling.LANCZOS
            )
            self.image = ImageTk.PhotoImage(blank_scaled)
            self.image_id = self.canvas.create_image(0, 0, anchor="nw", image=self.image)
            
        except Exception as e:
            self.canvas.delete("all")
            self.image_id = self.canvas.create_image(0, 0, anchor="nw")

    def show_game_result(self, result_type, duration=None):
        self.canvas.delete("all")
        self.canvas.create_text(
            self.canvas_width // 2, 
            self.canvas_height // 2, 
            text="MAZE COMPLETE", 
            fill="white", 
            font=("Jockey One", 24)
        )
        
        if result_type == "success" and duration:
            self.start_button.config(
                state="normal",
                bg="#4CAF50",
                text="GOAL REACHED"
            )
            self.status_label.config(
                text=f"YOU WIN - TIME: {duration} SECONDS",
                fg="#2E7D32"
            )
            
        elif result_type == "failure":
            self.controller.show_frame("PlayAloneFailedScreen")
            self.mqtt_client.client.publish("jetson/command", "PlayAloneFailed")
            self.start_button.config(
                state="normal",
                bg="#F44336",
                text="TRY AGAIN"
            )
            self.status_label.config(
                text="FAILED - TRY AGAIN",
                fg="#C62828"
            )

    def _auto_reset_after_result(self):
        self.reset_game_state()
        self.status_label.config(fg="#1A1A1A")

    def show(self):
        self.reset_game_state()
        self.clear_all_memory()
        self.focus_set()
        self.update_idletasks()

    def hide(self):
        self.pack_forget()