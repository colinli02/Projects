import serial
import matplotlib.pyplot as plt
from collections import deque
import re
from matplotlib.animation import FuncAnimation

# === Configuration ===
port = 'COM5'         # Change this to your serial port
baud_rate = 9600
max_points = 200      # Number of points shown in the plot

# === Initialize serial connection ===
ser = serial.Serial(port, baud_rate)

# === Data storage ===
comp_angle_data = deque([0]*max_points, maxlen=max_points)
forward_percent_data = deque([0]*max_points, maxlen=max_points)

# === Regex pattern to extract values ===
pattern = r"Comp Angle:\s*(-?\d+\.\d+).*?Forwards Percentage:\s*(\d+\.\d+)"

# === Set up plot ===
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='Comp Angle')
line2, = ax.plot([], [], label='Forwards %')
ax.set_ylim(-180, 100)
ax.set_xlim(0, max_points)
ax.set_title("Real-Time Sensor Data")
ax.set_xlabel("Samples")
ax.set_ylabel("Values")
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
        except:
            continue  # Ignore bad lines

    line1.set_data(range(len(comp_angle_data)), comp_angle_data)
    line2.set_data(range(len(forward_percent_data)), forward_percent_data)
    return line1, line2

ani = FuncAnimation(fig, update, interval=10)
plt.show()