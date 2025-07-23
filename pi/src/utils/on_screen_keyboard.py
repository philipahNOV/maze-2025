# custom_keyboard.py

import tkinter as tk

class OnScreenKeyboard(tk.Toplevel):
    def __init__(self, master, target_entry, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.target_entry = target_entry
        self.configure(bg="#222222")
        self.title("Keyboard")
        self.geometry("+200+600")  # Adjust position if needed
        self.create_keyboard()
        self.lift()

    def create_keyboard(self):
        keys = [
            ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', 'å'],
            ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', 'ø'],
            ['z', 'x', 'c', 'v', 'b', 'n', 'm', 'æ'],
            ['SPACE', 'BACK', 'CLEAR']
        ]

        for row_idx, row in enumerate(keys):
            for col_idx, key in enumerate(row):
                action = lambda k=key: self.key_press(k)
                btn = tk.Button(self, text=key.upper(), width=5, height=2, command=action,
                                font=("Jockey One", 14), bg="#444", fg="white",
                                activebackground="#666")
                btn.grid(row=row_idx, column=col_idx, padx=2, pady=2)

    def key_press(self, key):
        if key == "BACK":
            current = self.target_entry.get()
            self.target_entry.delete(0, tk.END)
            self.target_entry.insert(0, current[:-1])
        elif key == "CLEAR":
            self.target_entry.delete(0, tk.END)
        elif key == "SPACE":
            self.target_entry.insert(tk.END, " ")
        else:
            self.target_entry.insert(tk.END, key)

        parent = self.master
        if hasattr(parent, 'check_start_ready'):
            parent.check_start_ready()