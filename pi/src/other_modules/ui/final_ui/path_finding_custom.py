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

        self.scale_ratio = 0.8
        self.true_width = 730
        self.true_height = 710
        self.offset_x = 390
        self.offset_y = 10

        self.waiting_phase = 0

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))

        # Layout the widgets including the logo
        self.create_widgets()
        #self.update_image()  # Start updating the image

    def update_image(self):
        if self.mqtt_client.img is not None and self.mqtt_client.path_found is not None:
            # Convert OpenCV BGR to RGB
            frame = cv2.cvtColor(self.mqtt_client.img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            img_scaled = img.resize(
                (int(self.true_width * self.scale_ratio), int(self.true_height * self.scale_ratio)),
                Image.Resampling.LANCZOS
            )

            self.image = ImageTk.PhotoImage(img_scaled)
            self.canvas.itemconfig(self.image_id, image=self.image)

            # Hide status text
            self.status_label.place_forget()
        else:
            # Show blank image and animated text
            blank_img = Image.open(self.controller.blank_image_path).convert("RGB")
            img_scaled = blank_img.resize(
                (int(self.true_width * self.scale_ratio), int(self.true_height * self.scale_ratio)),
                Image.Resampling.LANCZOS
            )
            self.image = ImageTk.PhotoImage(img_scaled)
            self.canvas.itemconfig(self.image_id, image=self.image)

            dots = "." * (self.waiting_phase // 2 + 1)
            self.waiting_phase = (self.waiting_phase + 1) % 6
            self.status_label.config(text=f"FINDING PATH{dots}")
            self.status_label.place(x=220, y=250, width=400, height=50)

        self.after(200, self.update_image)  # update every 200 ms 

    def on_button_click_back(self):
        self.mqtt_client.client.publish("jetson/command", "Back")
        self.controller.show_frame("NavigationScreen")

    def on_button_click_calculate(self):
        self.mqtt_client.client.publish("jetson/command", "CalculatePath")
        self.mqtt_client.path_found = False
        if hasattr(self, 'click_marker') and self.click_marker is not None:
            self.canvas.delete(self.click_marker)

    def on_canvas_click(self, event):
        x, y = event.x, event.y

        # Remove previous dot if it exists
        if hasattr(self, 'click_marker') and self.click_marker is not None:
            self.canvas.delete(self.click_marker)

        # Draw new green dot
        r = 5
        self.click_marker = self.canvas.create_oval(x-r, y-r, x+r, y+r, fill="green", outline="")

        x = self.true_width - int(x / self.scale_ratio) + self.offset_x
        y = self.true_height - int(y / self.scale_ratio) + self.offset_y

        self.enable_buttons()  # Enable buttons when goal is set
        print(f"[CustomPathScreen] Clicked at pixel: ({x}, {y})")
        self.mqtt_client.client.publish("jetson/command", f"Goal_set:{x},{y}")

    def enable_buttons(self):
        self.calculate_button.config(state="normal", bg="#EE3229", activebackground="#B82F27", fg="white", activeforeground="#DFDFDF")

    def disable_buttons(self):
        self.calculate_button.config(state="disabled", bg="#723D3A", activebackground="#331E1D", fg="#9E9E9E", activeforeground="#7A7A7A")


    def add_essential_buttons(self):
        self.exit_button = tk.Button(
            self,
            text="✖",
            font=("Jockey One", 30),
            fg="white",
            bg="#EE3229",              # Red exit button
            activebackground="#B82F27",
            activeforeground="#DFDFDF",
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            command=self.controller.on_close  # or self.controller.destroy
        )
        self.exit_button.place(x=964, y=10, width=50, height=50) 

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.canvas = tk.Canvas(self, width=int(self.true_width * self.scale_ratio), height=int(self.true_height * self.scale_ratio))
        self.canvas.place(x=150, y=16, relwidth=1, relheight=1)
                
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
        self.back_button.place(x=804, y=10, width=150, height=50)

        self.status_label = tk.Label(
            self,
            text="",
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
            state="disabled",  # Initially disabled
            command=self.on_button_click_calculate,
        )
        self.calculate_button.place(x=770, y=300, width=200, height=75)

    def show(self):
        """Make this frame visible"""
        self.update_image()  # Start updating the image
        self.mqtt_client.path_found = False

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
