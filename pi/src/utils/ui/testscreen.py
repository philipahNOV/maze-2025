import tkinter as tk

class ScreenTemplate(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client

        self.create_widgets()

    def create_widgets(self):
        # Back button
        back_button = tk.Button(self, text="← Back", command=self.go_back)
        back_button.grid(row=0, column=0, sticky="nw", padx=10, pady=10)

        # Title
        title_label = tk.Label(self, text="Screen Title", font=("Helvetica", 24))
        title_label.grid(row=1, column=0, columnspan=2, pady=(20, 10))

        # Info Label
        info_label = tk.Label(self, text="This is a simple screen with a layout template.", font=("Helvetica", 14))
        info_label.grid(row=2, column=0, columnspan=2, pady=(10, 20))

        # Example Button
        action_button = tk.Button(self, text="Do Action", command=self.do_action)
        action_button.grid(row=3, column=0, padx=20, pady=10)

        # Exit Button
        exit_button = tk.Button(self, text="Exit App", command=self.exit_app)
        exit_button.grid(row=3, column=1, padx=20, pady=10)

        # Grid configuration
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

    def go_back(self):
        self.mqtt_client.client.publish("jetson/command", "0.0")
        self.controller.show_frame("Screen1")

    def do_action(self):
        print("Action button pressed")
        self.mqtt_client.client.publish("jetson/command", "9.9")

    def exit_app(self):
        self.controller.show_frame("Screen1")

    def show(self):
        self.pack(expand=True, fill=tk.BOTH)

    def hide(self):
        self.pack_forget()