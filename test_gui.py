import tkinter as tk
from tkinter import font as tkfont

class TestApp:
    def __init__(self, window):
        self.window = window
        window.geometry("900x520")
        
        print("Testing Unicode labels:")
        
        print("Testing: ⚙")
        tk.Label(self.window, text="⚙  KRAI 2026  —  MASTER CONTROL DASHBOARD").pack()
        
        print("Testing: ▶")
        tk.Button(self.window, text="▶  START AUTO").pack()
        
        print("Testing: ↺")
        tk.Button(self.window, text="↺  RETRY  AREA 1").pack()
        
        print("Testing: ⛔")
        tk.Button(self.window, text="⛔  EMERGENCY STOP").pack()
        
        print("Testing: ⟳")
        tk.Button(self.window, text="⟳  RESET SYSTEM").pack()
        
        print("Testing: 💾")
        tk.Button(self.window, text="💾  APPLY & SAVE COORDINATES").pack()
        
        print("All unicode tests passed!")

def main():
    root = tk.Tk()
    app = TestApp(root)
    root.after(500, root.quit)
    root.mainloop()

if __name__ == '__main__':
    main()
