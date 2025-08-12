import tkinter as tk
from PIL import Image, ImageTk
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from main import MainApp


class CalibratingScreen(tk.Frame):
    def __init__(self, parent, controller: 'MainApp', mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.background_image = ImageTk.PhotoImage(Image.open(controller.background_path))
        self.create_widgets()

    def on_button_click_back(self):
        self.controller.show_frame("AdminToolsScreen")

    def on_button_click_motor(self, dir):
        self.dir = dir
        self.mqtt_client.client.publish("jetson/command", "Motor:" + dir)

    def on_release(self, event):
        self.dir = None
        self.mqtt_client.client.publish("jetson/command", "Motor:stop")

    def on_button_click_confirm(self):
        self.mqtt_client.client.publish("jetson/command", "Confirm")
        self.controller.show_frame("AdminToolsScreen")

    def create_widgets(self):
        self.update()
        self.bg_label = tk.Label(self, image=self.background_image)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)

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

        self.title = tk.Label(
            self,
            text="CALIBRATE OFFSETS",
            font=("Jockey One", 50),
            fg="#1A1A1A",
            bg="#D9D9D9"
        )
        self.title.place(x=380, y=70)

        self.up_button = tk.Button(
            self,
            text="⇧",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.up_button.place(x=452, y=200, width=120, height=120)
        self.up_button.bind("<ButtonRelease-1>", self.on_release)
        self.up_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("up"))

        self.down_button = tk.Button(
            self,
            text="⇩",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.down_button.place(x=452, y=440, width=120, height=120)
        self.down_button.bind("<ButtonRelease-1>", self.on_release)
        self.down_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("down")) 

        self.left_button = tk.Button(
            self,
            text="⇦",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.left_button.place(x=332, y=320, width=120, height=120)   
        self.left_button.bind("<ButtonRelease-1>", self.on_release)    
        self.left_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("left")) 

        self.right_button = tk.Button(
            self,
            text="⇨",
            font=("Jockey One", 30),
            fg="white",                    # Text color
            borderwidth=0,            # No border
            highlightthickness=0,     # No highlight border
            background="#EE3229",     # Match image color or use transparent if supported
            activebackground="#B82F27",  # Match on press
            activeforeground="#DFDFDF",
        )
        self.right_button.place(x=692, y=320, width=120, height=120)   
        self.right_button.bind("<ButtonRelease-1>", self.on_release)     
        self.right_button.bind("<ButtonPress-1>", lambda event: self.on_button_click_motor("right"))  

        self.info_button = tk.Button(
        self,
        text="CONFIRM",
        font=("Jockey One", 26),
        fg="white",
        bg="#EE3229",
        activebackground="#B82F27",
        activeforeground="#DFDFDF",
        borderwidth=0,
        highlightthickness=0,
        relief="flat",
        command=self.on_button_click_confirm
        )
        self.info_button.place(x=391, y=450, width=243, height=74)

    def show(self):
        pass
    
    def hide(self):
        self.pack_forget()