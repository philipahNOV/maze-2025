import tkinter as tk
from PIL import Image, ImageTk
import os
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp
import cv2


class CustomPathScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.scale_ratio = controller.config['camera'].get('maze_image_scale', 0.8)
        self.true_width = controller.config['camera'].get('maze_width', 730)
        self.true_height = controller.config['camera'].get('maze_height', 710)
        self.offset_x = controller.config['camera'].get('maze_offset_x', 390)
        self.offset_y = controller.config['camera'].get('maze_offset_y', 10)
        self.animation_fps = controller.config["animations"]["path_finding"].get("fps", 8)

        self.has_pathfinded = False
        self.awaiting_path = False

        self.pathfinding_images = [
            ImageTk.PhotoImage(
                Image.open(os.path.join(self.controller.pathfinding_animation_path, f"{i}.png"))
                .convert("RGBA")
                .resize((100, 100), Image.Resampling.LANCZOS)
            )
            for i in range(1, 9)
        ]
        self.animation_index = 0
        self._animating = False

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.touch_image = ImageTk.PhotoImage(
            Image.open(controller.touch_path).resize((30, 30), Image.Resampling.LANCZOS)
        )

        self.create_widgets()
        self.update_image()

    def update_image(self):
        if self.mqtt_client.img is not None and not self.mqtt_client.finding_path:
            if self.awaiting_path:
                if self.mqtt_client.path_failed:
                    print("[UI] Pathfinding failed. Showing image but disabling START buttons.")
                    self.awaiting_path = False
                    self.has_pathfinded = False
                    self.disable_button_start()
                    self.path_failed_label.place(x=730, y=135)
                else:
                    print("[UI] Path found. Enabling START buttons.")
                    self.awaiting_path = False
                    self.has_pathfinded = True
                    self.enable_button_start()
                    self.path_failed_label.place_forget()

            self._animating = False
            self.animation_label.place_forget()

            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            img_scaled = img.resize(
                (int(self.true_width * self.scale_ratio), int(self.true_height * self.scale_ratio)),
                Image.Resampling.LANCZOS
            )

            self.image = ImageTk.PhotoImage(img_scaled)
            self.canvas.itemconfig(self.image_id, image=self.image)
            self.status_label.place_forget()

        else:
            self.has_pathfinded = False
            self.disable_button_start()
            self.path_failed_label.place_forget()

            if not self._animating:
                self._animating = True
                self.animation_label.place(x=442, y=300, width=100, height=100, anchor="center")
                self.animate_pathfinding()

            blank_img = Image.open(self.controller.blank_image_path).convert("RGB")
            img_scaled = blank_img.resize(
                (int(self.true_width * self.scale_ratio), int(self.true_height * self.scale_ratio)),
                Image.Resampling.LANCZOS
            )
            self.image = ImageTk.PhotoImage(img_scaled)
            self.canvas.itemconfig(self.image_id, image=self.image)
            self.status_label.place(x=442, y=370, anchor="n")

        self.after(200, self.update_image)


    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("MainScreen")

    def on_button_click_calculate(self):
        self.mqtt_client.client.publish("jetson/command", "CalculatePath")
        self.awaiting_path = True
        self.has_pathfinded = False
        self.mqtt_client.finding_path = True
        self.mqtt_client.path_failed = False

        self.disable_button_calculate()
        if hasattr(self, 'click_marker') and self.click_marker is not None:
            self.canvas.delete(self.click_marker)

    def on_button_click_start(self):
        self.mqtt_client.client.publish("jetson/command", "StartSafe")
        self.controller.show_frame("ControllingScreen")

    def on_button_click_start_speed(self):
        self.mqtt_client.client.publish("jetson/command", "StartSpeed")
        self.controller.show_frame("ControllingScreen")

    def on_button_click_restart(self):
        self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.restart_program()

    def on_button_click_exit(self):
        if self.controller.reset_jetson_on_exit:
            self.mqtt_client.client.publish("jetson/command", "Restart")
        self.controller.on_close()

    def on_canvas_click(self, event):
        x, y = event.x, event.y
        canvas_width = int(self.true_width * self.scale_ratio)
        canvas_height = int(self.true_height * self.scale_ratio)

        if 0 <= x <= canvas_width and 0 <= y <= canvas_height:
            if hasattr(self, 'click_marker') and self.click_marker is not None:
                self.canvas.delete(self.click_marker)

            r = 5
            self.click_marker = self.canvas.create_oval(x - r, y - r, x + r, y + r, fill="green", outline="")

            transformed_x = self.true_width - int(x / self.scale_ratio) + self.offset_x
            transformed_y = self.true_height - int(y / self.scale_ratio) + self.offset_y

            self.enable_button_calculate()
            print(f"[CustomPathScreen] Clicked at pixel: ({transformed_x}, {transformed_y})")
            self.mqtt_client.client.publish("jetson/command", f"Goal_set:{transformed_x},{transformed_y}")
        else:
            print("[CustomPathScreen] Click ignored: outside of image bounds.")

    def enable_button_calculate(self):
        self.calculate_button.config(state="normal", bg="#EE3229", activebackground="#B82F27", fg="white", activeforeground="#DFDFDF")

    def disable_button_calculate(self):
        self.calculate_button.config(state="disabled", bg="#723D3A", activebackground="#331E1D", fg="#9E9E9E", activeforeground="#7A7A7A")

    def enable_button_start(self):
        self.start_button.config(state="normal", bg="#EE3229", activebackground="#B82F27", fg="white", activeforeground="#DFDFDF")
        self.start_speed_button.config(state="normal", bg="#EE3229", activebackground="#B82F27", fg="white", activeforeground="#DFDFDF")

    def disable_button_start(self):
        self.start_button.config(state="disabled", bg="#723D3A", activebackground="#331E1D", fg="#9E9E9E", activeforeground="#7A7A7A")
        self.start_speed_button.config(state="disabled", bg="#723D3A", activebackground="#331E1D", fg="#9E9E9E", activeforeground="#7A7A7A")

    def animate_pathfinding(self):
        if not self._animating or not self.mqtt_client.finding_path:
            return
        self.animation_label.configure(image=self.pathfinding_images[self.animation_index])
        self.animation_index = (self.animation_index + 1) % len(self.pathfinding_images)
        self.after(int(1000 / self.animation_fps), self.animate_pathfinding)

    def add_essential_buttons(self):
        self.exit_button = tk.Button(
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
        self.exit_button.place(x=964, y=10, width=50, height=50) 

        self.exit_button = tk.Button(
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
        self.exit_button.place(x=904, y=10, width=50, height=50)

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.canvas = tk.Canvas(self, width=int(self.true_width * self.scale_ratio), height=int(self.true_height * self.scale_ratio))
        self.canvas.place(x=150, y=16, relwidth=1, relheight=1)
        self.animation_label = tk.Label(self, bg="#D9D9D9")
        self.animation_label.place(x=442, y=300, width=100, height=100, anchor="center")
        self.touch_label = tk.Label(self, image=self.touch_image, bg="#D9D9D9")
        self.touch_label.place(x=742, y=105, width=30, height=30)
                
        placeholder = ImageTk.PhotoImage(Image.new("RGB", (1, 1), (0, 0, 0)))
        self.image = placeholder
        self.image_id = self.canvas.create_image(0, 0, anchor="nw", image=self.image)

        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.add_essential_buttons()

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
            command=self.on_button_click_back,
        )
        self.back_button.place(x=744, y=10, width=150, height=50)

        self.status_label = tk.Label(
            self,
            text="FINDING PATH",
            font=("Jockey One", 35),
            fg="white",
            bg="#4D4D4D",
            anchor="w",
            justify="left"
        )

        self.calculate_button = tk.Button(
            self,
            text="CALCULATE PATH",
            font=("Jockey One", 20),
            fg="#9E9E9E",
            bg="#723D3A",           
            activebackground="#331E1D",
            activeforeground="#7A7A7A",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            state="disabled",
            command=self.on_button_click_calculate,
        )
        self.calculate_button.place(x=770, y=300, width=200, height=75)

        self.start_button = tk.Button(
            self,
            text="START ROBOT\nSAFE CONTROLLER",
            font=("Jockey One", 14),
            fg="#9E9E9E",
            bg="#723D3A",           
            activebackground="#331E1D",
            activeforeground="#7A7A7A",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            justify="center",
            state="disabled",
            command=self.on_button_click_start,
        )
        self.start_button.place(x=770, y=385, width=200, height=75)

        self.start_speed_button = tk.Button(
            self,
            text="START ROBOT\nFAST CONTROLLER",
            font=("Jockey One", 14),
            fg="#9E9E9E",
            bg="#723D3A",           
            activebackground="#331E1D",
            activeforeground="#7A7A7A",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            justify="center",
            state="disabled",
            command=self.on_button_click_start_speed,
        )
        self.start_speed_button.place(x=770, y=470, width=200, height=75)

        self.goal_label = tk.Label(
            self,
            text="CLICK IMAGE TO SET GOAL",
            font=("Jockey One", 16),
            fg="#000000",
            bg="#D9D9D9"
        )
        self.goal_label.place(x=780, y=105)

        self.path_failed_label = tk.Label(
            self,
            text="⚠ Pathfinding Failed. Try Again.",
            font=("Jockey One", 16),
            fg=self.controller.nov_red,
            bg="#D9D9D9",
        )

    def show(self):
        self.awaiting_path = False
        self.has_pathfinded = False
        self.mqtt_client.finding_path = False
        if hasattr(self, 'click_marker') and self.click_marker is not None:
            self.canvas.delete(self.click_marker)
        self.disable_button_calculate()
        self.disable_button_start()


    def hide(self):
        self.pack_forget()