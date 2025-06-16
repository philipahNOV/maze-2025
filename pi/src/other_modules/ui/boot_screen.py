import tkinter as tk

class BootScreen(tk.Frame):
    def __init__(self, parent, controller, mqtt_client):
        super().__init__(parent)
        self.controller = controller
        self.mqtt_client = mqtt_client
        # Layout the widgets
        self.create_widgets()
        # Start the periodic check
        self.check_pi_state()
        self.mqtt_client.initiate_handshake()

    def create_widgets(self):
        # Main instruction label
        main_label = tk.Label(self, text="Booting: Turn on Jetson:", font=('Helvetica', 25))
        main_label.grid(row=1, column=0, sticky="w", padx=(175, 20), pady=(150, 5))  # Adjusted vertical padding

        # Configure column widths to allocate extra space to the right side
        self.grid_columnconfigure(0, weight=3)  # Give more weight to the text column
        self.grid_columnconfigure(1, weight=1)  # Smaller weight to the button column

        # Configure row heights to allow for compact vertical layout and central alignment
        self.grid_rowconfigure(0, weight=0)  # Adjust weight for more compact layout
        self.grid_rowconfigure(1, weight=0)  # Adjust weight for more compact layout
        

    def check_pi_state(self):
        if self.mqtt_client.pi_state == "0.0":
            self.controller.show_frame("Screen1")
        else:
            self.after(1000, self.check_pi_state)  # Check again after 1000 ms (1 second)

    def show(self):
        """Make this frame visible"""
        self.pack(expand=True, fill=tk.BOTH)
        # Ensure the periodic check starts when the frame is shown
        self.check_pi_state()

    def hide(self):
        """Hide this frame"""
        self.pack_forget()
        self.grid_remove()
