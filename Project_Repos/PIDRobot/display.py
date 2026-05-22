import serial
import matplotlib.pyplot as plt
from collections import deque
import re
from matplotlib.animation import FuncAnimation
import csv
from datetime import datetime
import tkinter as tk
from tkinter import ttk

# === Config USB COM port ===
port = 'COM5'
baud_rate = 9600
max_points = 200

# Alarms
ANGLE_HIGH = 30
ANGLE_LOW = -30

# === Initialize serial ===
ser = serial.Serial(port, baud_rate)

# === Data storage ===
comp_angle_data = deque([0]*max_points, maxlen=max_points)
forward_percent_data = deque([0]*max_points, maxlen=max_points)

# === Regex pattern ===
pattern = r"Comp Angle:\s*(-?\d+\.\d+).*?Forwards Percentage:\s*(\d+\.\d+)"

# === Logging setup ===
log_file = "scada_log.csv"
with open(log_file, "a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "comp_angle", "forward_percent"])

def log_data(angle, forward):
    with open(log_file, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([datetime.now(), angle, forward])

# === Alarm system ===
def check_alarms(angle):
    if angle > ANGLE_HIGH:
        alarm_label.config(text="ALARM: Angle too HIGH", foreground="red")
    elif angle < ANGLE_LOW:
        alarm_label.config(text="ALARM: Angle too LOW", foreground="red")
    else:
        alarm_label.config(text="Normal", foreground="green")

# === Control commands ===
def send_command(cmd):
    ser.write((cmd + "\n").encode())

# === GUI (HMI) ===
root = tk.Tk()
root.title("SCADA Dashboard")

frame = ttk.Frame(root)
frame.pack()

alarm_label = ttk.Label(frame, text="Normal", font=("Arial", 16))
alarm_label.pack()

btn_reset = ttk.Button(frame, text="RESET", command=lambda: send_command("RESET"))
btn_reset.pack()

btn_kp = ttk.Button(frame, text="KP=1.2", command=lambda: send_command("KP=1.2"))
btn_kp.pack()

btn_forward = ttk.Button(frame, text="Forward 20%", command=lambda: send_command("FWD=20"))
btn_forward.pack()

# === Plot ===
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='Comp Angle')
line2, = ax.plot([], [], label='Forwards %')
ax.set_ylim(-180, 100)
ax.set_xlim(0, max_points)
ax.legend()
plt.tight_layout()

def update(frame):
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            match = re.search(pattern, line)
            if match:
                comp_angle = float(match.group(1))
                forward_percent = float(match.group(2))

                comp_angle_data.append(comp_angle)
                forward_percent_data.append(forward_percent)

                log_data(comp_angle, forward_percent)
                check_alarms(comp_angle)

        except:
            continue

    line1.set_data(range(len(comp_angle_data)), comp_angle_data)
    line2.set_data(range(len(forward_percent_data)), forward_percent_data)
    return line1, line2

ani = FuncAnimation(fig, update, interval=10)

# Run GUI + Plot
plt.show(block=False)
root.mainloop()
