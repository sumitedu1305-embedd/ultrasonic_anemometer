import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

print("done second time")

# -------- CONFIG --------
PORT = "COM6"
BAUD = 921600

HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55

PAYLOAD_SAMPLES = 500
# ------------------------

ser = serial.Serial(PORT, BAUD, timeout=1)

# -------- Frame Reader --------
def read_frame():
    # --- SOUTHOUT ---
    while True:
        data = ser.read(2)
        if len(data) < 2:
            continue

        value = data[0] | (data[1] << 8)
        if value == HEADER_SOUTHOUT:
            payload = ser.read(PAYLOAD_SAMPLES * 2)
            if len(payload) != PAYLOAD_SAMPLES * 2:
                continue
            samples1 = struct.unpack('<' + 'H'*PAYLOAD_SAMPLES, payload)
            break

    # --- NORTHOUT ---
    while True:
        data = ser.read(2)
        if len(data) < 2:
            continue

        value = data[0] | (data[1] << 8)
        if value == HEADER_NORTHOUT:
            payload = ser.read(PAYLOAD_SAMPLES * 2)
            if len(payload) != PAYLOAD_SAMPLES * 2:
                continue
            samples2 = struct.unpack('<' + 'H'*PAYLOAD_SAMPLES, payload)
            break

    return np.array(samples1), np.array(samples2)

# -------- Plot Setup --------
fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

x = np.arange(PAYLOAD_SAMPLES)

line1, = axs[0].plot(x, np.zeros(PAYLOAD_SAMPLES))
line2, = axs[1].plot(x, np.zeros(PAYLOAD_SAMPLES))

axs[0].set_title("Southout Raw Signal")
axs[1].set_title("Northout Raw Signal")

axs[0].set_ylim(1950, 2250)
axs[1].set_ylim(1950, 2250)

# -------- Update Loop --------
def update(frame):
    samples1, samples2 = read_frame()

    samples1 = samples1 - np.mean(samples1)
    samples2 = samples2 - np.mean(samples1)

    line1.set_ydata(samples1)
    line2.set_ydata(samples2)

    return line1, line2

# -------- Run --------
ani = FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()