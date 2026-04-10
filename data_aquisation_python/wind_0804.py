import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt, correlate
from collections import deque

print("done second time")

# -------- CONFIG --------
PORT = "COM6"
BAUD = 921600

HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55

PAYLOAD_SAMPLES = 500

FS = 1e6

SENSOR_DISTANCE = 0.210   # meters
SOUND_SPEED = 343 

lag_buffer = deque(maxlen=30)
wind_buffer = deque(maxlen=30)

# ------------------------

ser = serial.Serial(PORT, BAUD, timeout=1)

# -------- Frame Reader --------
def read_frame():
    while True:
        # ---- FIND SOUTH ----
        data = ser.read(2)
        if len(data) < 2:
            continue

        value = data[0] | (data[1] << 8)

        if value != HEADER_SOUTHOUT:
            continue

        payload1 = ser.read(PAYLOAD_SAMPLES * 2)
        if len(payload1) != PAYLOAD_SAMPLES * 2:
            continue

        samples1 = struct.unpack('<' + 'H'*PAYLOAD_SAMPLES, payload1)

        # ---- IMMEDIATELY EXPECT NORTH ----
        data = ser.read(2)
        if len(data) < 2:
            continue

        value = data[0] | (data[1] << 8)

        if value != HEADER_NORTHOUT:
            continue  # discard pair, resync

        payload2 = ser.read(PAYLOAD_SAMPLES * 2)
        if len(payload2) != PAYLOAD_SAMPLES * 2:
            continue

        samples2 = struct.unpack('<' + 'H'*PAYLOAD_SAMPLES, payload2)

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
axs[2].set_ylim(-100, 100)
axs[3].set_ylim(-100, 100)


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

    s1 = samples1_filtered / (np.std(samples1_filtered) + 1e-8)
    s2 = samples2_filtered / (np.std(samples2_filtered) + 1e-8)
    corr = correlate(s1, s2, mode='full')
    lags = np.arange(-len(s1)+1, len(s1))
    i = np.argmax(corr)
    lag = lags[i]
    if 0 < i < len(corr)-1:
        y0, y1, y2 = corr[i-1], corr[i], corr[i+1]
        frac = (y0 - y2) / (2*(y0 - 2*y1 + y2) + 1e-8)
        lag = lag + frac
    dt = lag / FS
    wind_speed = (lag / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    # ---- STORE ----
    lag_buffer.append(lag)
    wind_buffer.append(wind_speed)
    lag_med = np.median(lag_buffer)
    wind_med = np.median(wind_buffer)
    lag_clean = [x for x in lag_buffer if abs(x - lag_med) < 1]
    wind_clean = [x for x in wind_buffer if abs(x - wind_med) < 1]
    if len(lag_clean) == 0:
        lag_clean = list(lag_buffer)
    if len(wind_clean) == 0:
        wind_clean = list(wind_buffer)
    lag_smooth = np.median(lag_clean)
    wind_smooth = np.median(wind_clean)

    print(f"raw: {wind_speed:.2f} | smooth: {wind_smooth:.2f} m/s | raw: {lag:.2f} | smooth: {lag_smooth:.2f}",end = '          \r')

    line1.set_ydata(samples1)
    line2.set_ydata(samples2)
    line3.set_ydata(samples1_filtered)
    line4.set_ydata(samples2_filtered)

    return line1, line2, line3, line4

try:
    # -------- Run --------
    ani = FuncAnimation(fig, update, interval=50)
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    plt.close('all')