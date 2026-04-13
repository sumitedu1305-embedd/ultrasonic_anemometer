import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt, correlate
from collections import deque
import os

# -------- CONFIG --------
PORT = "COM6"
BAUD = 921600

HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55
HEADER_WESTOUT = 0xCC55
HEADER_EASTOUT = 0xDD55

PAYLOAD_SAMPLES = 500

FS = 1e6

SENSOR_DISTANCE = 0.210   # meters
SOUND_SPEED = 343 

lag_buffer_ns = deque(maxlen=30)
wind_buffer_ns = deque(maxlen=30)
lag_buffer_ew = deque(maxlen=30)
wind_buffer_ew = deque(maxlen=30)

# ------------------------

ser = serial.Serial(PORT, BAUD, timeout=1)

# -------- Frame Reader --------
def read_frame():
    while True:
        # ---- SOUTH OUT ----
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

        # ---- NORTH OUT ----
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

        # ---- WEST OUT ----
        data = ser.read(2)
        if len(data) < 2:
            continue

        value = data[0] | (data[1] << 8)

        if value != HEADER_WESTOUT:
            continue  # discard pair, resync

        payload3 = ser.read(PAYLOAD_SAMPLES * 2)
        if len(payload3) != PAYLOAD_SAMPLES * 2:
            continue

        samples3 = struct.unpack('<' + 'H'*PAYLOAD_SAMPLES, payload3)

        # ---- EAST OUT ----
        data = ser.read(2)
        if len(data) < 2:
            continue

        value = data[0] | (data[1] << 8)

        if value != HEADER_EASTOUT:
            continue  # discard pair, resync

        payload4 = ser.read(PAYLOAD_SAMPLES * 2)
        if len(payload4) != PAYLOAD_SAMPLES * 2:
            continue

        samples4 = struct.unpack('<' + 'H'*PAYLOAD_SAMPLES, payload4)

        return np.array(samples1), np.array(samples2), np.array(samples3), np.array(samples4)

# -------- Plot Setup --------
fig, axs = plt.subplots(4, 2, figsize=(8, 6), sharex=True)
axs_flat = axs.flatten() 

x = np.arange(PAYLOAD_SAMPLES)

line1, = axs_flat[0].plot(x, np.zeros(PAYLOAD_SAMPLES))
line2, = axs_flat[1].plot(x, np.zeros(PAYLOAD_SAMPLES))
line3, = axs_flat[2].plot(x, np.zeros(PAYLOAD_SAMPLES))
line4, = axs_flat[3].plot(x, np.zeros(PAYLOAD_SAMPLES))
line5, = axs_flat[4].plot(x, np.zeros(PAYLOAD_SAMPLES))
line6, = axs_flat[5].plot(x, np.zeros(PAYLOAD_SAMPLES))
line7, = axs_flat[6].plot(x, np.zeros(PAYLOAD_SAMPLES))
line8, = axs_flat[7].plot(x, np.zeros(PAYLOAD_SAMPLES))

axs_flat[0].set_ylim(-100, 100)
axs_flat[1].set_ylim(-50, 50)
axs_flat[2].set_ylim(-100, 100)
axs_flat[3].set_ylim(-50, 50)
axs_flat[4].set_ylim(-100, 100)
axs_flat[5].set_ylim(-50, 50)
axs_flat[6].set_ylim(-100, 100)
axs_flat[7].set_ylim(-50, 50)

os.system('cls' if os.name == 'nt' else 'clear')


def bandpass_filter(data, lowcut, highcut, fs, order=4):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    
    b, a = butter(order, [low, high], btype='band')
    filtered = filtfilt(b, a, data)  # zero-phase filtering
    
    return filtered

# -------- Update Loop --------
def update(frame):

    samples1, samples2, samples3, samples4 = read_frame()

    samples1 = samples1 - np.mean(samples1)
    samples2 = samples2 - np.mean(samples2)
    samples3 = samples3 - np.mean(samples3)
    samples4 = samples4 - np.mean(samples4)

    window1 = np.hanning(len(samples1))
    window2 = np.hanning(len(samples2))
    window3 = np.hanning(len(samples3))
    window4 = np.hanning(len(samples4))
    samples1_hanning = samples1 * window1
    samples2_hanning = samples2 * window2
    samples3_hanning = samples3 * window3
    samples4_hanning = samples4 * window4

    samples1_filtered = bandpass_filter(samples1_hanning,30e3,50e3,FS)
    samples2_filtered = bandpass_filter(samples2_hanning,30e3,50e3,FS)
    samples3_filtered = bandpass_filter(samples3_hanning,30e3,50e3,FS)
    samples4_filtered = bandpass_filter(samples4_hanning,30e3,50e3,FS)

    s1 = samples1_filtered / (np.std(samples1_filtered) + 1e-8)
    s2 = samples2_filtered / (np.std(samples2_filtered) + 1e-8)
    corr_ns = correlate(s1, s2, mode='full')
    lags_ns = np.arange(-len(s1)+1, len(s1))
    i_ns = np.argmax(corr_ns)
    lag_ns = lags_ns[i_ns]
    if 0 < i_ns < len(corr_ns)-1:
        y0, y1, y2 = corr_ns[i_ns-1], corr_ns[i_ns], corr_ns[i_ns+1]
        frac = (y0 - y2) / (2*(y0 - 2*y1 + y2) + 1e-8)
        lag_ns = lag_ns + frac
    wind_speed_ns = (lag_ns / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    s3 = samples3_filtered / (np.std(samples3_filtered) + 1e-8)
    s4 = samples4_filtered / (np.std(samples4_filtered) + 1e-8)
    corr_ew = correlate(s3, s4, mode='full')
    lags_ew = np.arange(-len(s3)+1, len(s3))
    i_ew = np.argmax(corr_ew)
    lag_ew = lags_ew[i_ew]
    if 0 < i_ew < len(corr_ew)-1:
        y0, y1, y2 = corr_ew[i_ew-1], corr_ew[i_ew], corr_ew[i_ew+1]
        frac = (y0 - y2) / (2*(y0 - 2*y1 + y2) + 1e-8)
        lag_ew = lag_ew + frac
    wind_speed_ew = (lag_ew / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    # ---- STORE ----
    lag_buffer_ns.append(lag_ns)
    wind_buffer_ns.append(wind_speed_ns)
    lag_med_ns = np.median(lag_buffer_ns)
    wind_med_ns = np.median(wind_buffer_ns)
    lag_clean_ns = [x for x in lag_buffer_ns if abs(x - lag_med_ns) < 2]
    wind_clean_ns = [x for x in wind_buffer_ns if abs(x - wind_med_ns) < 2]
    if len(lag_clean_ns) == 0:
        lag_clean_ns = list(lag_buffer_ns)
    if len(wind_clean_ns) == 0:
        wind_clean_ns = list(wind_buffer_ns)
    lag_smooth_ns = np.median(lag_clean_ns)
    wind_smooth_ns = np.median(wind_clean_ns)

    lag_buffer_ew.append(lag_ew)
    wind_buffer_ew.append(wind_speed_ew)
    lag_med_ew = np.median(lag_buffer_ew)
    wind_med_ew = np.median(wind_buffer_ew)
    lag_clean_ew = [x for x in lag_buffer_ew if abs(x - lag_med_ew) < 2]
    wind_clean_ew = [x for x in wind_buffer_ew if abs(x - wind_med_ew) < 2]
    if len(lag_clean_ew) == 0:
        lag_clean_ew = list(lag_buffer_ew)
    if len(wind_clean_ew) == 0:
        wind_clean_ew = list(wind_buffer_ew)
    lag_smooth_ew = np.median(lag_clean_ew)
    wind_smooth_ew = np.median(wind_clean_ew)

    wind_speed = (np.sqrt(wind_smooth_ns**2 + wind_smooth_ew**2))
    direction_rad = np.arctan2(wind_speed_ew,wind_speed_ns)    
    direction_deg = (np.degrees(direction_rad) + 360) % 360

    print(f"NS -> {lag_smooth_ns:.3f} : {wind_speed_ns:.3f}")
    print(f"EW -> {lag_smooth_ew:.3f} : {wind_speed_ew:.3f}")
    print(f"WS -> {wind_speed:.3f} | WD -> {direction_deg:.3f}")
    print("\n\n")

    line1.set_ydata(samples1)
    line3.set_ydata(samples2)
    line5.set_ydata(samples3)
    line7.set_ydata(samples4)

    line2.set_ydata(samples1_filtered)
    line4.set_ydata(samples2_filtered)
    line6.set_ydata(samples3_filtered)
    line8.set_ydata(samples4_filtered)

    return line1, line2, line3, line4, line5, line6, line7, line8

try:
    # -------- Run --------
    ani = FuncAnimation(fig, update, interval=50)
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    plt.close('all')