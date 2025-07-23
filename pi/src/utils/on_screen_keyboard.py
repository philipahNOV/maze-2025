import tkinter as tk

class OnScreenKeyboard(tk.Toplevel):
    def __init__(self, master, target_entry):
        super().__init__(master)
        self.title("Keyboard")
        self.configure(bg="#222")
        self.resizable(False, False)
        self.target_entry = target_entry
        self.build_keyboard()

    def build_keyboard(self):
        keys = [
            ['1','2','3','4','5','6','7','8','9','0','⌫'],
            ['Q','W','E','R','T','Y','U','I','O','P'],
            ['A','S','D','F','G','H','J','K','L','Æ','Ø','Å'],
            ['Z','X','C','V','B','N','M'],
            ['SPACE', 'CLEAR', 'OK']
        ]

        for row_index, row in enumerate(keys):
            row_frame = tk.Frame(self, bg="#222")
            row_frame.pack(pady=2)

            for key in row:
                btn = tk.Button(
                    row_frame, text=key, width=4, height=2,
                    bg="#444", fg="white", font=("Arial", 12, "bold"),
                    activebackground="#666", activeforeground="white",
                    command=lambda k=key: self.handle_key(k)
                )
                btn.pack(side="left", padx=2)

    def handle_key(self, key):
        if not self.target_entry:
            return

        if key == "SPACE":
            self.target_entry.insert(tk.END, " ")
        elif key == "CLEAR":
            self.target_entry.delete(0, tk.END)
        elif key == "⌫":
            current = self.target_entry.get()
            self.target_entry.delete(0, tk.END)
            self.target_entry.insert(0, current[:-1])
        elif key == "OK":
            self.destroy()
        else:
            self.target_entry.insert(tk.END, key)