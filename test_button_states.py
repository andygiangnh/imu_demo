"""
Test script to verify the new communication protocol and button state management
"""

import tkinter as tk
from tkinter import ttk
import time

def test_button_states():
    """Test the button enable/disable functionality"""
    
    root = tk.Tk()
    root.title("Button State Test")
    root.geometry("400x300")
    
    # Create test buttons
    buttons = {}
    
    commands = [
        ("Get Angles", "."),
        ("Reset Yaw", "z"),
        ("Raw Data", "r"),
        ("Heading", "h"),
        ("🧭 Cal Mag", "m"),
        ("🔄 Cal Gyro", "c")
    ]
    
    frame = ttk.Frame(root, padding=10)
    frame.pack(fill="both", expand=True)
    
    for i, (text, cmd) in enumerate(commands):
        row = i // 2
        col = i % 2
        btn = ttk.Button(frame, text=text, 
                        command=lambda c=cmd: print(f"Command: {c}"))
        btn.grid(row=row, column=col, padx=5, pady=5, sticky="ew")
        buttons[cmd] = btn
    
    # Configure grid weights
    for col in range(2):
        frame.columnconfigure(col, weight=1)
    
    # Test functions
    def disable_buttons():
        for btn in buttons.values():
            btn.configure(state='disabled')
        print("Buttons disabled")
    
    def enable_buttons():
        for btn in buttons.values():
            btn.configure(state='normal')
        print("Buttons enabled")
    
    # Control buttons
    control_frame = ttk.Frame(root, padding=10)
    control_frame.pack()
    
    ttk.Button(control_frame, text="Disable All", command=disable_buttons).pack(side="left", padx=5)
    ttk.Button(control_frame, text="Enable All", command=enable_buttons).pack(side="left", padx=5)
    
    # Status display
    status_text = tk.Text(root, height=5, width=50)
    status_text.pack(padx=10, pady=10)
    
    def log_message(message):
        status_text.insert(tk.END, f"{message}\n")
        status_text.see(tk.END)
    
    log_message("Button state test ready")
    log_message("New protocol: c1=gyro progress, c2=gyro done, c3=gyro failed")
    log_message("              m1=mag progress, m2=mag done")
    
    root.mainloop()

if __name__ == "__main__":
    test_button_states()
