import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt


print("done second time")

# -------- CONFIG --------
PORT = "COM6"
BAUD = 921600

HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55

PAYLOAD_SAMPLES = 500

FS = 1e6

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
fig, axs = plt.subplots(4, 1, figsize=(8, 6), sharex=True)

x = np.arange(PAYLOAD_SAMPLES)

line1, = axs[0].plot(x, np.zeros(PAYLOAD_SAMPLES))
line2, = axs[1].plot(x, np.zeros(PAYLOAD_SAMPLES))
line3, = axs[2].plot(x, np.zeros(PAYLOAD_SAMPLES))
line4, = axs[3].plot(x, np.zeros(PAYLOAD_SAMPLES))

axs[0].set_ylim(-150, 150)
axs[1].set_ylim(-150, 150)
axs[2].set_ylim(-150, 150)
axs[3].set_ylim(-150, 150)

def bandpass_filter(data, lowcut, highcut, fs, order=4):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    
    b, a = butter(order, [low, high], btype='band')
    filtered = filtfilt(b, a, data)  # zero-phase filtering
    
    return filtered

# -------- Update Loop --------
def update(frame):
    samples1, samples2 = read_frame()

    samples1 = samples1 - np.mean(samples1)
    samples2 = samples2 - np.mean(samples2)

    window1 = np.hanning(len(samples1))
    window2 = np.hanning(len(samples2))
    samples1_hanning = samples1 * window1
    samples2_hanning = samples2 * window2

    samples1_filtered = bandpass_filter(samples1_hanning,30e3,50e3,FS)
    samples2_filtered = bandpass_filter(samples2_hanning,30e3,50e3,FS)

    line1.set_ydata(samples1)
    line2.set_ydata(samples2)
    line3.set_ydata(samples1_filtered)
    line4.set_ydata(samples2_filtered)

    return line1, line2


try:
    # -------- Run --------
    ani = FuncAnimation(fig, update, interval=50)
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    plt.close('all')