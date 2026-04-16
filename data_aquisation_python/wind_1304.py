import serial, struct, numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt, correlate
from collections import deque
import threading, queue, os, sys

# -------- CONFIG --------
PORT = "COM6"
BAUD = 921600

SILENT_END = 80   # samples; adjust to where echo starts on your earliest channel

HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55
HEADER_WESTOUT  = 0xCC55
HEADER_EASTOUT  = 0xDD55

PAYLOAD_SAMPLES = 350
FS              = 1e6
SENSOR_DISTANCE = 0.210
SOUND_SPEED     = 343.0

CALIB_DURATION  = 5.0   # seconds to collect lags when calibrating

frame_queue = queue.Queue(maxsize=4)
stop_event  = threading.Event()

lag_buffer_ns  = deque(maxlen=30)
wind_buffer_ns = deque(maxlen=30)
lag_buffer_ew  = deque(maxlen=30)
wind_buffer_ew = deque(maxlen=30)

# -------- Calibration state --------
calib_lock        = threading.Lock()
calib_active      = False
calib_start_time  = 0.0
calib_lags_ns     = []
calib_lags_ew     = []
offset_ns         = 0.0   # samples, subtracted before wind calc
offset_ew         = 0.0

def remove_dc(sig, silent_end=SILENT_END):
    return sig - sig[:silent_end].mean()

# -------- Serial Reader --------
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

# -------- DSP --------
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
GRN = '#00c97d'

fig = plt.figure(figsize=(12, 7), facecolor=BG)
fig.canvas.manager.set_window_title('Wind Anemometer — press C to calibrate')

gs = fig.add_gridspec(4, 3, left=0.04, right=0.96, top=0.95, bottom=0.04, hspace=0.35, wspace=0.3)

sig_axes = [fig.add_subplot(gs[r, c]) for r in range(4) for c in range(2)]
ax_compass = fig.add_subplot(gs[:, 2])

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

# -------- Compass --------
ax_compass.set_facecolor(BG)
ax_compass.set_aspect('equal')
ax_compass.set_xlim(-1.3, 1.3)
ax_compass.set_ylim(-1.45, 1.45)
ax_compass.axis('off')

for r, alpha in [(1.0, 0.25), (0.7, 0.15), (0.4, 0.10)]:
    ax_compass.add_patch(plt.Circle((0, 0), r, color=FG, fill=False, lw=0.5, alpha=alpha))

CARDINALS = [('N',0),('NE',45),('E',90),('SE',135), ('S',180),('SW',225),('W',270),('NW',315)]
for label, deg in CARDINALS:
    rad = np.radians(deg)
    ax_compass.plot([np.sin(rad)*0.90, np.sin(rad)*1.00],
                    [np.cos(rad)*0.90, np.cos(rad)*1.00], color=DIM, lw=0.8)
    is_card = len(label) == 1
    ax_compass.text(np.sin(rad)*1.15, np.cos(rad)*1.15, label,
                    ha='center', va='center',
                    fontsize=9 if is_card else 6,
                    color=FG if is_card else DIM,
                    fontweight='bold' if is_card else 'normal')

for deg in range(0, 360, 10):
    if deg % 45 == 0: continue
    rad = np.radians(deg)
    ax_compass.plot([np.sin(rad)*0.95, np.sin(rad)*1.00],
                    [np.cos(rad)*0.95, np.cos(rad)*1.00], color=DIM, lw=0.4)

speed_circle = plt.Circle((0, 0), 0.01, color=RED, alpha=0.15, fill=True)
ax_compass.add_patch(speed_circle)

arrow_line, = ax_compass.plot([0, 0], [0,  0.65], color=RED, lw=2.5, solid_capstyle='round')
arrow_head, = ax_compass.plot([0], [0.65], marker='^', ms=8, color=RED, markeredgewidth=0)
arrow_tail, = ax_compass.plot([0, 0], [0, -0.30], color=DIM, lw=1.2,
                               linestyle='--', dash_capstyle='round')
ax_compass.plot(0, 0, 'o', color=FG, ms=4, zorder=5)

spd_txt    = ax_compass.text(0, -1.08, '-- m/s',         ha='center', va='center', fontsize=10, color=RED,  fontfamily='monospace')
dir_txt    = ax_compass.text(0,  1.25, '--°',             ha='center', va='center', fontsize=8,  color=FG,   fontfamily='monospace')
lag_txt    = ax_compass.text(0, -1.20, 'N/S: --  E/W: --',ha='center', va='center', fontsize=6.5,color=DIM,  fontfamily='monospace')
offset_txt = ax_compass.text(0, -1.32, 'offset: 0 / 0',  ha='center', va='center', fontsize=6.5,color=DIM,  fontfamily='monospace')
calib_txt  = ax_compass.text(0, -1.43, '',                ha='center', va='center', fontsize=7,  color=GRN,  fontfamily='monospace')
ax_compass.text(0, 1.38, 'WIND', ha='center', va='center', fontsize=9, color=DIM,
                fontfamily='monospace', fontweight='bold')

# -------- Helpers --------
def dir_label(deg):
    dirs = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
            'S','SSW','SW','WSW','W','WNW','NW','NNW']
    return dirs[int((deg + 11.25) / 22.5) % 16]

def rotate_point(px, py, deg):
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return c*px - s*py, s*px + c*py

MAX_SPEED = 25.0

# -------- Calibration trigger (keyboard) --------
def on_key(event):
    global calib_active, calib_start_time, calib_lags_ns, calib_lags_ew
    if event.key in ('c', 'C'):
        with calib_lock:
            if calib_active:
                return   # already running, ignore
            calib_active     = True
            calib_start_time = plt.matplotlib.dates.datetime.datetime.now().timestamp()
            calib_lags_ns    = []
            calib_lags_ew    = []
        print(f"\n[CAL] Calibration started — keep still for {CALIB_DURATION:.0f}s ...")

fig.canvas.mpl_connect('key_press_event', on_key)

# -------- Update --------
def update(frame_num):
    global calib_active, offset_ns, offset_ew

    try:
        s1, s2, s3, s4 = frame_queue.get_nowait()
    except queue.Empty:
        return sig_lines + [arrow_line, arrow_head, arrow_tail, speed_circle,
                             spd_txt, dir_txt, lag_txt, offset_txt, calib_txt]

    # DC + window + filter
    s1 = remove_dc(s1); s2 = remove_dc(s2)
    s3 = remove_dc(s3); s4 = remove_dc(s4)

    s1f = bandpass_filter(s1 * _hanning, 30e3, 50e3, FS)
    s2f = bandpass_filter(s2 * _hanning, 30e3, 50e3, FS)
    s3f = bandpass_filter(s3 * _hanning, 30e3, 50e3, FS)
    s4f = bandpass_filter(s4 * _hanning, 30e3, 50e3, FS)

    # Raw lags (samples)
    def get_lag(a, b):
        na = a / (a.std() + 1e-8)
        nb = b / (b.std() + 1e-8)
        corr = correlate(na, nb, mode='full')
        lags  = np.arange(-len(na)+1, len(na))
        i     = int(np.argmax(corr))
        lag   = float(lags[i])
        if 0 < i < len(corr)-1:
            y0, y1, y2 = corr[i-1], corr[i], corr[i+1]
            lag += (y0 - y2) / (2*(y0 - 2*y1 + y2) + 1e-8)
        return lag

    raw_lag_ns = get_lag(s1f, s2f)
    raw_lag_ew = get_lag(s3f, s4f)

    # ---- Calibration collection ----
    import time
    with calib_lock:
        if calib_active:
            calib_lags_ns.append(raw_lag_ns)
            calib_lags_ew.append(raw_lag_ew)
            elapsed = time.time() - calib_start_time
            remaining = CALIB_DURATION - elapsed

            if remaining > 0:
                calib_txt.set_text(f'CALIBRATING... {remaining:.1f}s')
                calib_txt.set_color(GRN)
            else:
                # Commit offsets
                offset_ns = float(np.median(calib_lags_ns))
                offset_ew = float(np.median(calib_lags_ew))
                calib_active = False
                # Clear smoothing buffers — old data had old offsets
                lag_buffer_ns.clear(); wind_buffer_ns.clear()
                lag_buffer_ew.clear(); wind_buffer_ew.clear()
                calib_txt.set_text(f'CAL DONE  ns={offset_ns:+.2f}  ew={offset_ew:+.2f}')
                calib_txt.set_color(GRN)
                print(f"[CAL] Done — offset N/S: {offset_ns:+.3f} samples  "
                      f"E/W: {offset_ew:+.3f} samples")
        else:
            # Fade the done message after a while (just blank it; simple enough)
            pass

    # ---- Apply offset then compute wind ----
    corrected_lag_ns = raw_lag_ns - offset_ns
    corrected_lag_ew = raw_lag_ew - offset_ew

    wind_ns = -(corrected_lag_ns / FS) * (SOUND_SPEED**2 / ( SENSOR_DISTANCE))
    wind_ew = -(corrected_lag_ew / FS) * (SOUND_SPEED**2 / ( SENSOR_DISTANCE))

    lag_sn  = smooth_median(lag_buffer_ns,  corrected_lag_ns)
    wind_sn = smooth_median(wind_buffer_ns, wind_ns)
    lag_se  = smooth_median(lag_buffer_ew,  corrected_lag_ew)
    wind_se = smooth_median(wind_buffer_ew, wind_ew)

    wind_speed    = np.sqrt(wind_sn**2 + wind_se**2)
    direction_rad = np.arctan2(wind_se, wind_sn)
    direction_deg = (np.degrees(direction_rad) + 360) % 360

    # ---- Terminal ----
    cal_marker = '[CAL]' if calib_active else f'[off {offset_ns:+.1f}/{offset_ew:+.1f}]'
    sys.stdout.write(
        f"\r {cal_marker} Wind: {wind_speed:6.2f} m/s | Dir: {direction_deg:6.1f}° "
        f"{dir_label(direction_deg):<3s} | Lag N/S: {lag_sn:+7.2f} | Lag E/W: {lag_se:+7.2f}   "
    )
    sys.stdout.flush()

    # ---- Compass ----
    ax_px, ay_px   = rotate_point(0,  0.65, direction_deg)
    tail_x, tail_y = rotate_point(0, -0.30, direction_deg)
    arrow_line.set_data([0, ax_px], [0, ay_px])
    arrow_head.set_data([ax_px], [ay_px])
    arrow_head.set_marker((3, 0, direction_deg))
    arrow_tail.set_data([0, tail_x], [0, tail_y])

    r = min(wind_speed / MAX_SPEED, 1.0) * 0.85
    speed_circle.set_radius(max(r, 0.01))

    spd_txt.set_text(f'{wind_speed:.1f} m/s')
    dir_txt.set_text(f'{direction_deg:.0f}° {dir_label(direction_deg)}')
    lag_txt.set_text(f'N/S {lag_sn:+.2f}  E/W {lag_se:+.2f}')
    offset_txt.set_text(f'offset  ns:{offset_ns:+.2f}  ew:{offset_ew:+.2f}')

    # ---- Signal plots ----
    raw_lim  = max(np.max(np.abs(s1)), np.max(np.abs(s2)),
                   np.max(np.abs(s3)), np.max(np.abs(s4))) * 1.2 + 1
    filt_lim = max(np.max(np.abs(s1f)), np.max(np.abs(s2f)),
                   np.max(np.abs(s3f)), np.max(np.abs(s4f))) * 1.2 + 1

    for idx, data, lim in [(0,s1,raw_lim),(1,s1f,filt_lim),(2,s2,raw_lim),(3,s2f,filt_lim),
                            (4,s3,raw_lim),(5,s3f,filt_lim),(6,s4,raw_lim),(7,s4f,filt_lim)]:
        sig_lines[idx].set_ydata(data)
        sig_axes[idx].set_ylim(-lim, lim)

    return sig_lines + [arrow_line, arrow_head, arrow_tail, speed_circle, spd_txt, dir_txt, lag_txt, offset_txt, calib_txt]


if __name__ == '__main__':
    print("Wind Anemometer — press C in the plot window to calibrate (keep sensors still).")
    print(f"Calibration collects {CALIB_DURATION:.0f}s of lags and stores the median as offset.\n")

    reader = threading.Thread(target=serial_reader, args=(PORT, BAUD), daemon=True)
    reader.start()

    try:
        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.close('all')
        print("\nStopped.")