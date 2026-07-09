#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ File-Based Wind Comparison Dashboard                                       │
#   │ Safely tails two independent text files without locking or crashing        │
#   └────────────────────────────────────────────────────────────────────────────┘
import os
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- CONFIGURATION ---
DSP_FILE    = "mine.txt"
SIMPLE_FILE = "calypso.txt"
MAX_PTS     = 100

# --- BUFFERS ---
hist_time       = deque(maxlen=MAX_PTS)
hist_spd_dsp    = deque(maxlen=MAX_PTS)
hist_spd_simple = deque(maxlen=MAX_PTS)

start_time = time.time()
last_dsp_spd, last_dsp_dir = 0.0, 0.0
last_smp_spd, last_smp_dir = 0.0, 0.0

# File pointers to remember where we last read
dsp_ptr = 0
smp_ptr = 0

def create_file_if_missing(filepath):
    if not os.path.exists(filepath):
        with open(filepath, 'w') as f:
            f.write("")

create_file_if_missing(DSP_FILE)
create_file_if_missing(SIMPLE_FILE)

#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ SAFE FILE READER                                                           │
#   └────────────────────────────────────────────────────────────────────────────┘

def read_new_lines(filepath, last_position):
    """
    Safely reads only newly appended lines.
    Returns the (latest_speed, latest_direction, new_file_position).
    """
    latest_spd, latest_dir = None, None
    try:
        with open(filepath, 'r') as f:
            f.seek(last_position)
            lines = f.readlines()
            new_position = f.tell()
            
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) == 2:
                    try:
                        latest_spd = float(parts[0])
                        latest_dir = float(parts[1])
                    except ValueError:
                        pass
                        
            return latest_spd, latest_dir, new_position
            
    except (PermissionError, IOError):
        # Windows locked the file for a split second because the sensor script 
        # was writing to it. Ignore it and try again next frame.
        return None, None, last_position

#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ UI SETUP & ANIMATION                                                       │
#   └────────────────────────────────────────────────────────────────────────────┘

fig = plt.figure(figsize=(12, 6), facecolor="#ffffff")
fig.canvas.manager.set_window_title('File-Tailing Sensor Comparison')

# Polar Setup
ax_polar = fig.add_subplot(121, polar=True)
ax_polar.set_theta_zero_location('N')
ax_polar.set_theta_direction(-1)
ax_polar.set_title("Wind Direction Comparison", pad=20)

arrow_dsp, = ax_polar.plot([], [], color="#e24b4a", lw=2.5, marker='^', ms=8, label="DSP")
arrow_smp, = ax_polar.plot([], [], color="#4a90e2", lw=2.5, marker='^', ms=8, label="Simple")
ax_polar.legend(loc="lower left", bbox_to_anchor=(0.9, 0.1))

# Cartesian Setup
ax_speed = fig.add_subplot(122)
ax_speed.set_title("Wind Speed Over Time")
ax_speed.set_ylabel("Speed (m/s)")
ax_speed.set_xlabel("Time (s)")
ax_speed.grid(True, alpha=0.3)

line_dsp, = ax_speed.plot([], [], color='#e24b4a', label='DSP', lw=2)
line_smp, = ax_speed.plot([], [], color='#4a90e2', label='Simple', lw=2)
legend = ax_speed.legend(loc="upper left")

def update(frame):
    global last_dsp_spd, last_dsp_dir, dsp_ptr
    global last_smp_spd, last_smp_dir, smp_ptr

    # Safely read newest lines
    new_d_spd, new_d_dir, dsp_ptr = read_new_lines(DSP_FILE, dsp_ptr)
    if new_d_spd is not None:
        last_dsp_spd, last_dsp_dir = new_d_spd, new_d_dir

    new_s_spd, new_s_dir, smp_ptr = read_new_lines(SIMPLE_FILE, smp_ptr)
    if new_s_spd is not None:
        last_smp_spd, last_smp_dir = new_s_spd, new_s_dir

    # Update History
    current_time = time.time() - start_time
    hist_time.append(current_time)
    hist_spd_dsp.append(last_dsp_spd)
    hist_spd_simple.append(last_smp_spd)

    # Dynamic limits
    max_spd = max(max(hist_spd_dsp + hist_spd_simple + deque([1])), 5)
    ax_speed.set_xlim(max(0, current_time - 10), current_time + 1)
    ax_speed.set_ylim(0, max_spd * 1.2)
    ax_polar.set_ylim(0, max_spd * 1.2)

    # Fast Screen Updates
    line_dsp.set_data(hist_time, hist_spd_dsp)
    line_smp.set_data(hist_time, hist_spd_simple)
    
    arrow_dsp.set_data([0, np.radians(last_dsp_dir)], [0, last_dsp_spd])
    arrow_smp.set_data([0, np.radians(last_smp_dir)], [0, last_smp_spd])
    
    legend.texts[0].set_text(f'DSP: {last_dsp_spd:.2f} m/s')
    legend.texts[1].set_text(f'Simple: {last_smp_spd:.2f} m/s')

    return [line_dsp, line_smp, arrow_dsp, arrow_smp]

if __name__ == '__main__':
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()