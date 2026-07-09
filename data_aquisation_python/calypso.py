import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

PORT     = "COM10"
BAUDRATE = 9600
MAX_PTS  = 50

speeds     = []
directions = []

fig = plt.figure(figsize=(10, 5))

ax1 = fig.add_subplot(121, polar=True)
ax1.set_title("Wind Direction", pad=20)
ax1.set_theta_zero_location('N')
ax1.set_theta_direction(-1)

ax2 = fig.add_subplot(122)
ax2.set_title("Wind Speed")
ax2.set_ylabel("m/s")
ax2.set_xlabel("samples")

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

def update(frame):
    global speeds, directions

    raw = ser.readline().decode('ascii', errors='ignore').strip()
    if ',' not in raw:
        return

    try:
        parts = raw.split(',')
        speed = float(parts[0])
        direc = float(parts[1])
    except:
        return

    speeds.append(speed)
    directions.append(direc)

    if len(speeds) > MAX_PTS:
        speeds     = speeds[-MAX_PTS:]
        directions = directions[-MAX_PTS:]

    curr_spd = speeds[-1]
    
    # ---- File IPC: Write to Plotter ----
    try:
        with open("calypso.txt", "a") as f:
            f.write(f"{speed:.2f},{direc:.1f}\n")
    except (PermissionError, IOError):
        # Ignore temporary Windows file locks
        pass

    # polar
    ax1.clear()
    ax1.set_theta_zero_location('N')
    ax1.set_theta_direction(-1)
    ax1.set_title("Wind Direction", pad=20)
    theta = np.radians(directions[-1])
    ax1.annotate("", xy=(theta, curr_spd),
                 xytext=(0, 0),
                 arrowprops=dict(arrowstyle="->", color="blue", lw=2))
    ax1.set_ylim(0, max(speeds + [1]))

    # speed history
    ax2.clear()
    ax2.set_title("Wind Speed")
    ax2.set_ylabel("m/s")
    ax2.set_xlabel("samples")
    ax2.plot(speeds, color='blue')
    ax2.fill_between(range(len(speeds)), speeds, alpha=0.2)
    ax2.set_ylim(0, max(speeds + [1]) * 1.2)
    ax2.text(0.02, 0.95, f"current: {curr_spd:.2f} m/s",
             transform=ax2.transAxes,
             fontsize=9, color='blue', va='top')

ani = animation.FuncAnimation(fig, update, interval=600, cache_frame_data=False)
plt.tight_layout()
plt.show()