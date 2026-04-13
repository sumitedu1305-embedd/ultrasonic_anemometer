import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt, correlate
from collections import deque
import threading
import queue
import os
import sys

matplotlib.use('TkAgg')  # Fast backend; change to 'Qt5Agg' if you prefer

# -------- CONFIG --------
PORT = "COM6"
BAUD = 921600

HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55
HEADER_WESTOUT  = 0xCC55
HEADER_EASTOUT  = 0xDD55

PAYLOAD_SAMPLES = 350
FS              = 1e6
SENSOR_DISTANCE = 0.210   # meters
SOUND_SPEED     = 343.0

# Threading
frame_queue   = queue.Queue(maxsize=4)   # small cap: drop stale frames, stay realtime
stop_event    = threading.Event()

# Smoothing buffers
lag_buffer_ns  = deque(maxlen=30)
wind_buffer_ns = deque(maxlen=30)
lag_buffer_ew  = deque(maxlen=30)
wind_buffer_ew = deque(maxlen=30)
# ------------------------


# -------- Serial Reader Thread --------
def serial_reader(port, baud):
    """Runs in background: reads frames, pushes into frame_queue. Never blocks the plot."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[serial_reader] ERROR opening port: {e}")
        stop_event.set()
        return

    def read_exact(n):
        buf = b''
        while len(buf) < n and not stop_event.is_set():
            chunk = ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    def expect_header(expected):
        """Drain until we find the expected 2-byte header."""
        while not stop_event.is_set():
            b = ser.read(1)
            if not b:
                continue
            low = b[0]
            b2 = ser.read(1)
            if not b2:
                continue
            val = low | (b2[0] << 8)
            if val == expected:
                return True
        return False

    def read_payload():
        data = read_exact(PAYLOAD_SAMPLES * 2)
        if len(data) != PAYLOAD_SAMPLES * 2:
            return None
        return np.array(struct.unpack('<' + 'H' * PAYLOAD_SAMPLES, data), dtype=np.float32)

    while not stop_event.is_set():
        try:
            if not expect_header(HEADER_SOUTHOUT): break
            s1 = read_payload()
            if s1 is None: continue

            if not expect_header(HEADER_NORTHOUT): break
            s2 = read_payload()
            if s2 is None: continue

            if not expect_header(HEADER_WESTOUT): break
            s3 = read_payload()
            if s3 is None: continue

            if not expect_header(HEADER_EASTOUT): break
            s4 = read_payload()
            if s4 is None: continue

            # Non-blocking put: if queue is full, drop oldest frame to stay realtime
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put_nowait((s1, s2, s3, s4))

        except Exception as e:
            if not stop_event.is_set():
                print(f"[serial_reader] Exception: {e}")

    ser.close()
    print("[serial_reader] Stopped.")


# -------- DSP --------
_butter_cache = {}

def get_butter_coeffs(lowcut, highcut, fs, order=4):
    key = (lowcut, highcut, fs, order)
    if key not in _butter_cache:
        nyquist = 0.5 * fs
        b, a = butter(order, [lowcut / nyquist, highcut / nyquist], btype='band')
        _butter_cache[key] = (b, a)
    return _butter_cache[key]

def bandpass_filter(data, lowcut, highcut, fs, order=4):
    b, a = get_butter_coeffs(lowcut, highcut, fs, order)
    return filtfilt(b, a, data)

def smooth_median(buf, new_val, threshold=2):
    buf.append(new_val)
    med = np.median(buf)
    clean = [x for x in buf if abs(x - med) < threshold] or list(buf)
    return float(np.median(clean))


# -------- Plot Setup --------
fig, axs = plt.subplots(4, 2, figsize=(10, 7), sharex=True)
fig.patch.set_facecolor('#0d0d0d')
axs_flat = axs.flatten()

x = np.arange(PAYLOAD_SAMPLES)
LABELS = [
    "S raw", "S filtered",
    "N raw", "N filtered",
    "W raw", "W filtered",
    "E raw", "E filtered",
]
COLORS_RAW  = '#3a7bd5'
COLORS_FILT = '#00d2ff'

lines = []
for i, ax in enumerate(axs_flat):
    ax.set_facecolor('#111111')
    ax.tick_params(colors='#888888', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#333333')
    color = COLORS_FILT if i % 2 else COLORS_RAW
    ln, = ax.plot(x, np.zeros(PAYLOAD_SAMPLES), lw=0.8, color=color)
    ax.set_ylim(-100, 100)
    ax.set_title(LABELS[i], color='#aaaaaa', fontsize=7, pad=2)
    lines.append(ln)

# Wind info text on the figure
info_text = fig.text(
    0.5, 0.01,
    "Wind: -- m/s | Dir: --° | Lag N/S: -- | Lag E/W: --",
    ha='center', va='bottom', fontsize=9,
    color='#00ffcc', fontfamily='monospace',
    bbox=dict(boxstyle='round,pad=0.3', facecolor='#1a1a1a', edgecolor='#333333')
)

plt.tight_layout(rect=[0, 0.04, 1, 1])
os.system('cls' if os.name == 'nt' else 'clear')

# Pre-build hanning windows (same length every frame)
_hanning = np.hanning(PAYLOAD_SAMPLES)


# -------- Update Loop (main thread) --------
def update(frame_num):
    try:
        s1, s2, s3, s4 = frame_queue.get_nowait()
    except queue.Empty:
        return lines  # nothing new, keep old display

    # DC removal
    s1 = s1 - s1.mean()
    s2 = s2 - s2.mean()
    s3 = s3 - s3.mean()
    s4 = s4 - s4.mean()

    # Windowing
    s1h = s1 * _hanning
    s2h = s2 * _hanning
    s3h = s3 * _hanning
    s4h = s4 * _hanning

    # Bandpass
    s1f = bandpass_filter(s1h, 30e3, 50e3, FS)
    s2f = bandpass_filter(s2h, 30e3, 50e3, FS)
    s3f = bandpass_filter(s3h, 30e3, 50e3, FS)
    s4f = bandpass_filter(s4h, 30e3, 50e3, FS)

    # Cross-correlation N/S
    n1 = s1f / (s1f.std() + 1e-8)
    n2 = s2f / (s2f.std() + 1e-8)
    corr_ns = correlate(n1, n2, mode='full')
    lags_ns  = np.arange(-len(n1)+1, len(n1))
    i_ns     = int(np.argmax(corr_ns))
    lag_ns   = float(lags_ns[i_ns])
    if 0 < i_ns < len(corr_ns)-1:
        y0, y1, y2 = corr_ns[i_ns-1], corr_ns[i_ns], corr_ns[i_ns+1]
        frac   = (y0 - y2) / (2*(y0 - 2*y1 + y2) + 1e-8)
        lag_ns = lag_ns + frac
    wind_speed_ns = (lag_ns / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    # Cross-correlation E/W
    n3 = s3f / (s3f.std() + 1e-8)
    n4 = s4f / (s4f.std() + 1e-8)
    corr_ew = correlate(n3, n4, mode='full')
    lags_ew  = np.arange(-len(n3)+1, len(n3))
    i_ew     = int(np.argmax(corr_ew))
    lag_ew   = float(lags_ew[i_ew])
    if 0 < i_ew < len(corr_ew)-1:
        y0, y1, y2 = corr_ew[i_ew-1], corr_ew[i_ew], corr_ew[i_ew+1]
        frac   = (y0 - y2) / (2*(y0 - 2*y1 + y2) + 1e-8)
        lag_ew = lag_ew + frac
    wind_speed_ew = (lag_ew / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    # Smoothing (your original median-based logic, unchanged)
    lag_smooth_ns  = smooth_median(lag_buffer_ns,  lag_ns)
    wind_smooth_ns = smooth_median(wind_buffer_ns,  wind_speed_ns)
    lag_smooth_ew  = smooth_median(lag_buffer_ew,  lag_ew)
    wind_smooth_ew = smooth_median(wind_buffer_ew,  wind_speed_ew)

    wind_speed    = np.sqrt(wind_smooth_ns**2 + wind_smooth_ew**2)
    direction_rad = np.arctan2(wind_smooth_ew, wind_smooth_ns)
    direction_deg = (np.degrees(direction_rad) + 360) % 360

    # ---- Terminal print (single overwriting line) ----
    sys.stdout.write(
        f"\r Wind: {wind_speed:6.2f} m/s | Dir: {direction_deg:6.1f}° "
        f"| Lag N/S: {lag_smooth_ns:+8.2f} samples "
        f"| Lag E/W: {lag_smooth_ew:+8.2f} samples   "
    )
    sys.stdout.flush()

    # ---- Update figure overlay text ----
    info_text.set_text(
        f"Wind: {wind_speed:.2f} m/s  |  Dir: {direction_deg:.1f}°  "
        f"|  Lag N/S: {lag_smooth_ns:+.2f}  |  Lag E/W: {lag_smooth_ew:+.2f}"
    )

    # ---- Plot data ----
    raw_ylim  = max(np.max(np.abs(s1)), np.max(np.abs(s2)),
                    np.max(np.abs(s3)), np.max(np.abs(s4))) * 1.2 + 1
    filt_ylim = max(np.max(np.abs(s1f)), np.max(np.abs(s2f)),
                    np.max(np.abs(s3f)), np.max(np.abs(s4f))) * 1.2 + 1

    raw_pairs  = [(0, s1),  (2, s2),  (4, s3),  (6, s4)]
    filt_pairs = [(1, s1f), (3, s2f), (5, s3f), (7, s4f)]

    for idx, data in raw_pairs:
        lines[idx].set_ydata(data)
        axs_flat[idx].set_ylim(-raw_ylim, raw_ylim)

    for idx, data in filt_pairs:
        lines[idx].set_ydata(data)
        axs_flat[idx].set_ylim(-filt_ylim, filt_ylim)

    return lines


# -------- Main --------
if __name__ == '__main__':
    # Start serial reader in daemon thread
    reader_thread = threading.Thread(target=serial_reader, args=(PORT, BAUD), daemon=True)
    reader_thread.start()
    print("Serial reader started. Close the plot window or Ctrl+C to stop.\n")

    try:
        ani = FuncAnimation(
            fig, update,
            interval=50,      # ms between frames; lower = faster but more CPU
            blit=True,        # only redraw changed artists = much faster
            cache_frame_data=False
        )
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.close('all')
        print("\nStopped.")