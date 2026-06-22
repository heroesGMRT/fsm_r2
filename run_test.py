import sys
import tkinter
from mission_fsm.ui.dashboard import RobotDashboard

print("TEST START")
sys.stdout.flush()

root = tkinter.Tk()
print("TK INIT")
sys.stdout.flush()

dummy_fsm = type('Dummy', (), {'task': type('Task', (), {'current_state': 'IDLE'})()})()
app = RobotDashboard(root, dummy_fsm)
print("DASHBOARD INIT DONE")
sys.stdout.flush()

root.after(500, root.quit)
root.mainloop()
