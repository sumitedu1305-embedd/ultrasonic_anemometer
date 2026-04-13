"""
COMPASS PATCH for wind_anemometer.py
─────────────────────────────────────
Replace your plot-setup block and update() function with the versions below.
Everything else (serial reader thread, DSP, smoothing) stays identical.
"""

import serial, struct, numpy as np
import matplotlib.pyplot as plt, matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow
from scipy.signal import butter, filtfilt, correlate
from collections import deque
import threading, queue, os, sys

# -------- CONFIG (unchanged) --------
PORT = "COM6"
BAUD = 921600

HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55
HEADER_WESTOUT  = 0xCC55
HEADER_EASTOUT  = 0xDD55

PAYLOAD_SAMPLES = 350
FS              = 1e6
SENSOR_DISTANCE = 0.210
SOUND_SPEED     = 343.0

frame_queue = queue.Queue(maxsize=4)
stop_event  = threading.Event()

lag_buffer_ns  = deque(maxlen=30)
wind_buffer_ns = deque(maxlen=30)
lag_buffer_ew  = deque(maxlen=30)
wind_buffer_ew = deque(maxlen=30)

# -------- Serial Reader (unchanged from threaded version) --------
def serial_reader(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[serial_reader] ERROR: {e}")
        stop_event.set(); return

    def read_exact(n):
        buf = b''
        while len(buf) < n and not stop_event.is_set():
            chunk = ser.read(n - len(buf))
            if chunk: buf += chunk
        return buf

    def expect_header(expected):
        while not stop_event.is_set():
            b = ser.read(1)
            if not b: continue
            b2 = ser.read(1)
            if not b2: continue
            if (b[0] | (b2[0] << 8)) == expected: return True
        return False

    def read_payload():
        data = read_exact(PAYLOAD_SAMPLES * 2)
        if len(data) != PAYLOAD_SAMPLES * 2: return None
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

            if frame_queue.full():
                try: frame_queue.get_nowait()
                except queue.Empty: pass
            frame_queue.put_nowait((s1, s2, s3, s4))
        except Exception as e:
            if not stop_event.is_set(): print(f"[serial_reader] {e}")
    ser.close()

# -------- DSP (unchanged) --------
_butter_cache = {}

def get_butter_coeffs(lc, hc, fs, order=4):
    key = (lc, hc, fs, order)
    if key not in _butter_cache:
        nyq = 0.5 * fs
        b, a = butter(order, [lc/nyq, hc/nyq], btype='band')
        _butter_cache[key] = (b, a)
    return _butter_cache[key]

def bandpass_filter(data, lc, hc, fs, order=4):
    b, a = get_butter_coeffs(lc, hc, fs, order)
    return filtfilt(b, a, data)

def smooth_median(buf, val, threshold=2):
    buf.append(val)
    med = np.median(buf)
    clean = [x for x in buf if abs(x - med) < threshold] or list(buf)
    return float(np.median(clean))

_hanning = np.hanning(PAYLOAD_SAMPLES)

# -------- Plot & Compass Setup --------
BG  = '#0d0d0d'
FG  = '#cccccc'
RED = '#e24b4a'
DIM = '#444444'

fig = plt.figure(figsize=(12, 7), facecolor=BG)

# Left side: 4×2 signal grid (takes 60% of width)
gs = fig.add_gridspec(4, 3, left=0.04, right=0.96, top=0.95, bottom=0.04,
                       hspace=0.35, wspace=0.3)

sig_axes = [fig.add_subplot(gs[r, c]) for r in range(4) for c in range(2)]
ax_compass = fig.add_subplot(gs[:, 2])   # right column: compass

LABELS = ['N raw','N filt','S raw','S filt','E raw','E filt','W raw','W filt']
sig_lines = []
x = np.arange(PAYLOAD_SAMPLES)
for i, ax in enumerate(sig_axes):
    ax.set_facecolor('#111111')
    ax.tick_params(colors='#555555', labelsize=6)
    for sp in ax.spines.values(): sp.set_edgecolor('#2a2a2a')
    color = '#00d2ff' if i % 2 else '#3a7bd5'
    ln, = ax.plot(x, np.zeros(PAYLOAD_SAMPLES), lw=0.7, color=color)
    ax.set_ylim(-100, 100)
    ax.set_title(LABELS[i], color='#666666', fontsize=7, pad=1)
    sig_lines.append(ln)

# -------- Compass Axes --------
ax_compass.set_facecolor(BG)
ax_compass.set_aspect('equal')
ax_compass.set_xlim(-1.3, 1.3)
ax_compass.set_ylim(-1.3, 1.3)
ax_compass.axis('off')

# Rings
for r, alpha in [(1.0, 0.25), (0.7, 0.15), (0.4, 0.10)]:
    ring = plt.Circle((0, 0), r, color=FG, fill=False, lw=0.5, alpha=alpha)
    ax_compass.add_patch(ring)

# Cardinal ticks & labels
CARDINALS = [('N', 0), ('NE', 45), ('E', 90), ('SE', 135),
             ('S', 180), ('SW', 225), ('W', 270), ('NW', 315)]
for label, deg in CARDINALS:
    rad = np.radians(deg)
    x0, y0 = np.sin(rad)*0.90, np.cos(rad)*0.90
    x1, y1 = np.sin(rad)*1.00, np.cos(rad)*1.00
    ax_compass.plot([x0, x1], [y0, y1], color=DIM, lw=0.8)
    xl, yl = np.sin(rad)*1.15, np.cos(rad)*1.15
    is_card = len(label) == 1
    ax_compass.text(xl, yl, label,
                    ha='center', va='center',
                    fontsize=9 if is_card else 6,
                    color=FG if is_card else DIM,
                    fontweight='bold' if is_card else 'normal')

# Minor ticks every 10°
for deg in range(0, 360, 10):
    if deg % 45 == 0: continue
    rad = np.radians(deg)
    x0, y0 = np.sin(rad)*0.95, np.cos(rad)*0.95
    x1, y1 = np.sin(rad)*1.00, np.cos(rad)*1.00
    ax_compass.plot([x0, x1], [y0, y1], color=DIM, lw=0.4)

# Speed circle (filled, scales with wind speed)
speed_circle = plt.Circle((0, 0), 0.01, color=RED, alpha=0.15, fill=True)
ax_compass.add_patch(speed_circle)

# Arrow: starts as a vertical line, rotated each frame
arrow_line,  = ax_compass.plot([0, 0], [0,  0.65], color=RED, lw=2.5, solid_capstyle='round')
arrow_head,  = ax_compass.plot([0], [0.65], marker='^', ms=8, color=RED, markeredgewidth=0)
arrow_tail,  = ax_compass.plot([0, 0], [0, -0.30], color=DIM, lw=1.2,
                                linestyle='--', dash_capstyle='round')

center_dot = ax_compass.plot(0, 0, 'o', color=FG, ms=4, zorder=5)[0]

# Text readouts inside compass
spd_txt = ax_compass.text(0, -1.1,  '-- m/s', ha='center', va='center',
                           fontsize=10, color=RED, fontfamily='monospace')
dir_txt = ax_compass.text(0,  1.25, '--°',    ha='center', va='center',
                           fontsize=8,  color=FG,  fontfamily='monospace')
lag_txt = ax_compass.text(0, -1.22, 'N/S: --  E/W: --', ha='center', va='center',
                           fontsize=6.5, color=DIM, fontfamily='monospace')

compass_title = ax_compass.text(0, 1.38, 'WIND', ha='center', va='center',
                                 fontsize=9, color=DIM, fontfamily='monospace',
                                 fontweight='bold')

# -------- Helpers --------
def dir_label(deg):
    dirs = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
            'S','SSW','SW','WSW','W','WNW','NW','NNW']
    return dirs[int((deg + 11.25) / 22.5) % 16]

def rotate_point(x, y, deg):
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return c*x - s*y, s*x + c*y

MAX_SPEED = 25.0   # m/s — max for speed ring scaling

# -------- Update --------
def update(frame_num):
    try:
        s1, s2, s3, s4 = frame_queue.get_nowait()
    except queue.Empty:
        return sig_lines + [arrow_line, arrow_head, arrow_tail, speed_circle,
                             spd_txt, dir_txt, lag_txt]

    s1 = s1 - s1.mean(); s2 = s2 - s2.mean()
    s3 = s3 - s3.mean(); s4 = s4 - s4.mean()

    s1h = s1 * _hanning; s2h = s2 * _hanning
    s3h = s3 * _hanning; s4h = s4 * _hanning

    s1f = bandpass_filter(s1h, 30e3, 50e3, FS)
    s2f = bandpass_filter(s2h, 30e3, 50e3, FS)
    s3f = bandpass_filter(s3h, 30e3, 50e3, FS)
    s4f = bandpass_filter(s4h, 30e3, 50e3, FS)

    n1 = s1f / (s1f.std() + 1e-8); n2 = s2f / (s2f.std() + 1e-8)
    corr_ns = correlate(n1, n2, mode='full')
    lags_ns = np.arange(-len(n1)+1, len(n1))
    i_ns = int(np.argmax(corr_ns)); lag_ns = float(lags_ns[i_ns])
    if 0 < i_ns < len(corr_ns)-1:
        y0,y1,y2 = corr_ns[i_ns-1], corr_ns[i_ns], corr_ns[i_ns+1]
        lag_ns += (y0-y2)/(2*(y0-2*y1+y2)+1e-8)
    wind_ns = -(lag_ns / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    n3 = s3f / (s3f.std() + 1e-8); n4 = s4f / (s4f.std() + 1e-8)
    corr_ew = correlate(n3, n4, mode='full')
    lags_ew = np.arange(-len(n3)+1, len(n3))
    i_ew = int(np.argmax(corr_ew)); lag_ew = float(lags_ew[i_ew])
    if 0 < i_ew < len(corr_ew)-1:
        y0,y1,y2 = corr_ew[i_ew-1], corr_ew[i_ew], corr_ew[i_ew+1]
        lag_ew += (y0-y2)/(2*(y0-2*y1+y2)+1e-8)
    wind_ew = -(lag_ew / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    lag_sn  = smooth_median(lag_buffer_ns,  lag_ns)
    wind_sn = smooth_median(wind_buffer_ns, wind_ns)
    lag_se  = smooth_median(lag_buffer_ew,  lag_ew)
    wind_se = smooth_median(wind_buffer_ew, wind_ew)

    wind_speed    = np.sqrt(wind_sn**2 + wind_se**2)
    direction_rad = np.arctan2(wind_se, wind_sn)
    direction_deg = (np.degrees(direction_rad) + 360) % 360

    # ---- Terminal ----
    sys.stdout.write(
        f"\r Wind: {wind_speed:6.2f} m/s | Dir: {direction_deg:6.1f}° {dir_label(direction_deg):<3s} "
        f"| Lag N/S: {lag_sn:+8.2f} | Lag E/W: {lag_se:+8.2f}   "
    )
    sys.stdout.flush()

    # ---- Compass arrow (rotate unit vector) ----
    ax_px, ay_px = rotate_point(0,  0.65, direction_deg)
    tail_x, tail_y = rotate_point(0, -0.30, direction_deg)
    arrow_line.set_data([0, ax_px], [0, ay_px])
    arrow_head.set_data([ax_px], [ay_px])
    arrow_tail.set_data([0, tail_x], [0, tail_y])

    # Rotate head marker
    arrow_head.set_marker((3, 0, direction_deg))   # triangle marker rotated

    # Speed ring
    r = min(wind_speed / MAX_SPEED, 1.0) * 0.85
    speed_circle.set_radius(max(r, 0.01))

    # Text
    spd_txt.set_text(f'{wind_speed:.1f} m/s')
    dir_txt.set_text(f'{direction_deg:.0f}° {dir_label(direction_deg)}')
    lag_txt.set_text(f'N/S {lag_sn:+.1f}  E/W {lag_se:+.1f}')

    # ---- Signal plots ----
    raw_lim  = max(np.max(np.abs(s1)), np.max(np.abs(s2)),
                   np.max(np.abs(s3)), np.max(np.abs(s4))) * 1.2 + 1
    filt_lim = max(np.max(np.abs(s1f)), np.max(np.abs(s2f)),
                   np.max(np.abs(s3f)), np.max(np.abs(s4f))) * 1.2 + 1

    for idx, data, lim in [(0,s1,raw_lim),(1,s1f,filt_lim),(2,s2,raw_lim),(3,s2f,filt_lim),
                            (4,s3,raw_lim),(5,s3f,filt_lim),(6,s4,raw_lim),(7,s4f,filt_lim)]:
        sig_lines[idx].set_ydata(data)
        sig_axes[idx].set_ylim(-lim, lim)

    return sig_lines + [arrow_line, arrow_head, arrow_tail, speed_circle,
                        spd_txt, dir_txt, lag_txt]


if __name__ == '__main__':
    reader = threading.Thread(target=serial_reader, args=(PORT, BAUD), daemon=True)
    reader.start()
    print("Running — close window or Ctrl+C to stop.\n")

    try:
        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.close('all')
        print("\nStopped.")