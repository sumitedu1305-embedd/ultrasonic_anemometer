#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ Wind Anemometer — Real-time ultrasonic wind speed & direction display      │
#   │ Serial → DSP → Cross-correlation → Compass UI                              │
#   └────────────────────────────────────────────────────────────────────────────┘
import sys
import time
import queue
import struct
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt, correlate
import serial

print("\n\n")

#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ CONFIGURATION                                                              │
#   └────────────────────────────────────────────────────────────────────────────┘

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = 921600

# Frame headers (little-endian 2-byte magic numbers per channel)
HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55
HEADER_WESTOUT  = 0xCC55
HEADER_EASTOUT  = 0xDD55
HEADER_TEMP     = 0xEE55

# Signal parameters
PAYLOAD_SAMPLES    = 275          # ADC samples per frame
FS                 = 1e6          # Sampling frequency (Hz)
SILENT_END         = 60           # Samples before echo arrives (for DC removal)

# Physical geometry
SENSOR_DISTANCE = 0.04272294
EFFECTIVE_DISTANCE = 2 * SENSOR_DISTANCE  # Full TX → reflector → RX path (metres)
SOUND_SPEED        = 343.0        # m/s at ~20°C

# Calibration
CALIB_DURATION     = 5.0          # Seconds to collect lags during calibration

MAX_DISPLAY_SPEED  = 10.0         # m/s — clamps compass speed-circle radius

#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ SHARED STATE                                                               │
#   └────────────────────────────────────────────────────────────────────────────┘

frame_queue = queue.Queue(maxsize=4)
stop_event  = threading.Event()

# Rolling buffers for median smoothing (last 30 readings)
lag_buffer_ns  = deque(maxlen=30)
lag_buffer_ew  = deque(maxlen=30)
wind_buffer_ns = deque(maxlen=30)
wind_buffer_ew = deque(maxlen=30)

# Calibration state (protected by calib_lock)
calib_lock       = threading.Lock()
calib_active     = False
calib_start_time = 0.0
calib_lags_ns    = []
calib_lags_ew    = []
offset_ns        = 0.0   # Lag offset in samples — subtracted before wind calc
offset_ew        = 0.0

# Vector smoothing & direction hold (ported from main)
smooth_ns          = 0.0
smooth_ew          = 0.0
DIR_VECTOR_ALPHA   = 0.1    # EMA factor for wind vector components
MIN_DIR_SPEED      = 0.15   # m/s — below this, freeze arrow direction
last_direction_deg = 0.0

#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ SIGNAL PREPROCESSING                                                       │
#   └────────────────────────────────────────────────────────────────────────────┘

def remove_dc(signal, silent_end=SILENT_END):
    return signal - signal[:silent_end].mean()


# Cache Butterworth coefficients so they aren't recomputed every frame
_butter_cache = {}

def get_butter_coeffs(low_cut, high_cut, fs, order=4):
    key = (low_cut, high_cut, fs, order)
    if key not in _butter_cache:
        nyq  = 0.5 * fs
        b, a = butter(order, [low_cut / nyq, high_cut / nyq], btype='band')
        _butter_cache[key] = (b, a)
    return _butter_cache[key]


def bandpass_filter(data, low_cut, high_cut, fs, order=4):
    b, a = get_butter_coeffs(low_cut, high_cut, fs, order)
    return filtfilt(b, a, data)


# Pre-computed Hanning window (applied once per frame)
_hanning_window = np.hanning(PAYLOAD_SAMPLES)



#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ CROSS-CORRELATION LAG ESTIMATION                                           │
#   └────────────────────────────────────────────────────────────────────────────┘

def  get_lag(signal_a, signal_b):
    """
    Estimate sub-sample lag between two signals via normalised cross-correlation.
    Parabolic interpolation refines the integer peak to sub-sample accuracy.
    Returns lag in samples (positive = signal_a leads signal_b).
    """
    norm_a = signal_a / (signal_a.std() + 1e-8)
    norm_b = signal_b / (signal_b.std() + 1e-8)

    corr = correlate(norm_a, norm_b, mode='full')
    lags = np.arange(-len(norm_a) + 1, len(norm_a))

    peak_idx = int(np.argmax(corr))
    lag      = float(lags[peak_idx])

    # Parabolic sub-sample refinement
    if 0 < peak_idx < len(corr) - 1:
        y0, y1, y2 = corr[peak_idx - 1], corr[peak_idx], corr[peak_idx + 1]
        lag += (y0 - y2) / (2 * (y0 - 2 * y1 + y2) + 1e-8)

    return lag


#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ OUTLIER-RESISTANT SMOOTHING                                                │
#   └────────────────────────────────────────────────────────────────────────────┘

def smooth_median(buffer, new_value, outlier_threshold=2.0):
    """
    Append new_value to buffer, then return the median of values within
    outlier_threshold of the median (resistant to spikes).
    """
    buffer.append(new_value)
    median  = np.median(buffer)
    cleaned = [v for v in buffer if abs(v - median) < outlier_threshold]
    return float(np.median(cleaned or list(buffer)))


#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ SERIAL READER  (runs in a background thread)                               │
#   └────────────────────────────────────────────────────────────────────────────┘

def serial_reader(port, baud):
    """
    Continuously reads four-channel frames from the serial port and
    pushes them onto frame_queue. One frame = S + N + W + E payloads.
    """
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        print(f"[serial_reader] ERROR opening port: {exc}")
        stop_event.set()
        return

    def read_exact(n_bytes):
        buf = b''
        while len(buf) < n_bytes and not stop_event.is_set():
            chunk = ser.read(n_bytes - len(buf))
            if chunk:
                buf += chunk
        return buf

    def expect_header(expected_header):
        while not stop_event.is_set():
            byte1 = ser.read(1)
            if not byte1:
                continue
            byte2 = ser.read(1)
            if not byte2:
                continue
            if (byte1[0] | (byte2[0] << 8)) == expected_header:
                return True
        return False

    def read_payload():
        raw = read_exact(PAYLOAD_SAMPLES * 2)
        if len(raw) != PAYLOAD_SAMPLES * 2:
            return None
        return np.array(
            struct.unpack('<' + 'H' * PAYLOAD_SAMPLES, raw),
            dtype=np.float32
        )

    while not stop_event.is_set():
        try:
            if not expect_header(HEADER_SOUTHOUT): break
            south = read_payload()
            if south is None: continue

            if not expect_header(HEADER_NORTHOUT): break
            north = read_payload()
            if north is None: continue

            if not expect_header(HEADER_WESTOUT): break
            west = read_payload()
            if west is None: continue

            if not expect_header(HEADER_EASTOUT): break
            east = read_payload()
            if east is None: continue
            
            if not expect_header(HEADER_TEMP): break
            temp_raw = read_exact(4)
            if len(temp_raw) != 4: continue
            
            sound_speed_val = struct.unpack('<f', temp_raw)[0]

            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass

            frame_queue.put_nowait((south, north, west, east, sound_speed_val))

        except Exception as exc:
            if not stop_event.is_set():
                print(f"[serial_reader] {exc}")

    ser.close()


#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ PLOT & COMPASS SETUP                                                       │
#   └────────────────────────────────────────────────────────────────────────────┘

# Colour palette (white background theme — matches main)
BG  = "#ffffff"
RED = '#e24b4a'
GRN = '#00c97d'
BLK = "#000000"
DIM = '#888888'

fig = plt.figure(figsize=(8, 6), facecolor=BG)
fig.canvas.manager.set_window_title('Wind Anemometer — press C to calibrate')

gs = fig.add_gridspec(
    4, 3,
    left=0.04, right=0.96, top=0.95, bottom=0.04,
    hspace=0.35, wspace=0.3
)

sig_axes   = [fig.add_subplot(gs[row, col]) for row in range(4) for col in range(2)]
ax_compass = fig.add_subplot(gs[:, 2])

# ---- Signal subplots -------------------------------------------------------
SIGNAL_LABELS = ['S raw', 'S filt', 'N raw', 'N filt',
                 'W raw', 'W filt', 'E raw', 'E filt']
sig_lines = []
x_axis    = np.arange(PAYLOAD_SAMPLES)

for idx, ax in enumerate(sig_axes):
    ln, = ax.plot(x_axis, np.zeros(PAYLOAD_SAMPLES), lw=0.7)
    ax.set_ylim(-100, 100)
    ax.set_title(SIGNAL_LABELS[idx], color=BLK, fontsize=7, pad=1)
    sig_lines.append(ln)

# ---- Compass face ----------------------------------------------------------
ax_compass.set_facecolor(BG)
ax_compass.set_aspect('equal')
ax_compass.set_xlim(-1.3, 1.3)
ax_compass.set_ylim(-1.45, 1.45)
ax_compass.axis('off')

for radius, alpha in [(1.0, 0.25), (0.7, 0.15), (0.4, 0.10)]:
    ax_compass.add_patch(
        plt.Circle((0, 0), radius, color=BLK, fill=False, lw=0.5, alpha=alpha)
    )

CARDINALS = [
    ('N', 0), ('NE', 45), ('E', 90), ('SE', 135),
    ('S', 180), ('SW', 225), ('W', 270), ('NW', 315)
]
for label, deg in CARDINALS:
    rad = np.radians(deg)
    sx, sy = np.sin(rad), np.cos(rad)
    ax_compass.plot([sx * 0.90, sx * 1.00], [sy * 0.90, sy * 1.00], color=BLK, lw=0.8)
    is_cardinal = len(label) == 1
    ax_compass.text(
        sx * 1.15, sy * 1.15, label,
        ha='center', va='center',
        fontsize=9 if is_cardinal else 6,
        color=BLK if is_cardinal else DIM,
        fontweight='bold' if is_cardinal else 'normal'
    )

for deg in range(0, 360, 10):
    if deg % 45 == 0:
        continue
    rad = np.radians(deg)
    sx, sy = np.sin(rad), np.cos(rad)
    ax_compass.plot([sx * 0.95, sx * 1.00], [sy * 0.95, sy * 1.00], color=BLK, lw=0.4)

# ---- Compass dynamic elements ----------------------------------------------
speed_circle = plt.Circle((0, 0), 0.01, color=RED, alpha=0.15, fill=True)
ax_compass.add_patch(speed_circle)

arrow_line, = ax_compass.plot([0, 0], [0,  0.65], color=RED, lw=2.5, solid_capstyle='round')
arrow_head, = ax_compass.plot([0],    [0.65],      marker='^', ms=8, color=RED, markeredgewidth=0)
arrow_tail, = ax_compass.plot([0, 0], [0, -0.30],  color=BLK, lw=1.2, linestyle='--', dash_capstyle='round')
ax_compass.plot(0, 0, 'o', color=BLK, ms=4, zorder=5)

# ---- Compass text labels (offset_txt added from main) ----------------------
spd_txt    = ax_compass.text(0, -1.08, '-- m/s',            ha='center', va='center', fontsize=10,  color=RED, fontfamily='monospace')
dir_txt    = ax_compass.text(0,  1.25, '--°',               ha='center', va='center', fontsize=8,   color=BLK, fontfamily='monospace')
lag_txt    = ax_compass.text(0, -1.20, 'N/S: --  E/W: --',  ha='center', va='center', fontsize=6.5, color=BLK, fontfamily='monospace')
offset_txt = ax_compass.text(0, -1.32, 'offset: 0 / 0',     ha='center', va='center', fontsize=6.5, color=BLK, fontfamily='monospace')
calib_txt  = ax_compass.text(0, -1.43, '',                   ha='center', va='center', fontsize=7,   color=GRN, fontfamily='monospace')
ax_compass.text(0,  1.38, 'WIND', ha='center', va='center', fontsize=9, color=BLK, fontfamily='monospace', fontweight='bold')


#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ HELPERS                                                                    │
#   └────────────────────────────────────────────────────────────────────────────┘

_COMPASS_ROSE = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
                 'S','SSW','SW','WSW','W','WNW','NW','NNW']

def direction_label(degrees):
    return _COMPASS_ROSE[int((degrees + 11.25) / 22.5) % 16]


def rotate_point(px, py, degrees):
    rad = np.radians(degrees)
    c, s = np.cos(rad), np.sin(rad)
    return c * px - s * py, s * px + c * py


#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ CALIBRATION — keyboard trigger                                             │
#   └────────────────────────────────────────────────────────────────────────────┘

def on_key_press(event):
    global calib_active, calib_start_time, calib_lags_ns, calib_lags_ew

    if event.key not in ('c', 'C'):
        return

    with calib_lock:
        if calib_active:
            return

        calib_active     = True
        calib_start_time = time.time()
        calib_lags_ns    = []
        calib_lags_ew    = []

    print(f"\n\n ----> CALIBRATING\n"
          f" [CAL] Calibration started — keep sensors still for {CALIB_DURATION:.0f}s ...")

fig.canvas.mpl_connect('key_press_event', on_key_press)


#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ ANIMATION UPDATE  (called by FuncAnimation every ~50 ms)                  │
#   └────────────────────────────────────────────────────────────────────────────┘

_ALL_ARTISTS = lambda: (
    sig_lines + [arrow_line, arrow_head, arrow_tail,
                 speed_circle, spd_txt, dir_txt, lag_txt, offset_txt, calib_txt]
)

def update(_frame_num):
    global calib_active, offset_ns, offset_ew
    global smooth_ns, smooth_ew, last_direction_deg

    try:
        # --- MODIFIED: Added sound_speed_val to unpacking ---
        south, north, west, east, sound_speed_val = frame_queue.get_nowait()
    except queue.Empty:
        return _ALL_ARTISTS()

    # ---- Preprocessing: DC removal → Hanning window → bandpass filter ------
    south = remove_dc(south)
    north = remove_dc(north)
    west  = remove_dc(west)
    east  = remove_dc(east)

    south_f = bandpass_filter(south * _hanning_window, 30e3, 50e3, FS)
    north_f = bandpass_filter(north * _hanning_window, 30e3, 50e3, FS)
    west_f  = bandpass_filter(west  * _hanning_window, 30e3, 50e3, FS)
    east_f  = bandpass_filter(east  * _hanning_window, 30e3, 50e3, FS)

    # ---- Cross-correlation lag estimation ----------------------------------
    raw_lag_ns = get_lag(south_f, north_f)
    raw_lag_ew = get_lag(west_f,  east_f) ;

    # ---- Calibration: collect lags or finalise offset ----------------------
    with calib_lock:
        if calib_active:
            calib_lags_ns.append(raw_lag_ns)
            calib_lags_ew.append(raw_lag_ew)

            remaining = CALIB_DURATION - (time.time() - calib_start_time)

            if remaining > 0:
                calib_txt.set_text(f'CALIBRATING... {remaining:.1f}s')
                calib_txt.set_color(GRN)
            else:
                offset_ns    = float(np.median(calib_lags_ns))
                offset_ew    = float(np.median(calib_lags_ew))
                calib_active = False

                lag_buffer_ns.clear();  wind_buffer_ns.clear()
                lag_buffer_ew.clear();  wind_buffer_ew.clear()

                # Reset EMA state so stale pre-calibration vector is gone
                smooth_ns = 0.0
                smooth_ew = 0.0

                calib_txt.set_text(f'CAL DONE  ns={offset_ns:+.2f}  ew={offset_ew:+.2f}')
                calib_txt.set_color(GRN)
                print(f"\n [CAL] Done — offset N/S: {offset_ns:+.3f} samples  "
                      f"E/W: {offset_ew:+.3f} samples\n")

    # ---- Apply calibration offset ------------------------------------------
    lag_ns = raw_lag_ns - offset_ns
    lag_ew = raw_lag_ew - offset_ew

    # ---- Wind speed calculation  v = Δt · c² / (2d) -----------------------
    wind_ns = -(lag_ns / FS) * (SOUND_SPEED ** 2 / EFFECTIVE_DISTANCE)
    wind_ew =  (lag_ew / FS) * (SOUND_SPEED ** 2 / EFFECTIVE_DISTANCE)

    # ---- Outlier-resistant median smoothing --------------------------------
    smooth_lag_ns  = smooth_median(lag_buffer_ns,  lag_ns)
    smooth_wind_ns = smooth_median(wind_buffer_ns, wind_ns)
    smooth_lag_ew  = smooth_median(lag_buffer_ew,  lag_ew)
    smooth_wind_ew = smooth_median(wind_buffer_ew, wind_ew)

    # ---- EMA on wind vector components (from main) -------------------------
    smooth_ns  = (1 - DIR_VECTOR_ALPHA) * smooth_ns + DIR_VECTOR_ALPHA * smooth_wind_ns
    smooth_ew  = (1 - DIR_VECTOR_ALPHA) * smooth_ew + DIR_VECTOR_ALPHA * smooth_wind_ew
    wind_speed = np.sqrt(smooth_ns ** 2 + smooth_ew ** 2)

    # ---- Direction hold at low speed (from main) ---------------------------
    if wind_speed > MIN_DIR_SPEED:
        direction_rad      = np.arctan2(smooth_ew, smooth_ns)
        direction_deg      = (np.degrees(direction_rad) + 360) % 360
        last_direction_deg = direction_deg
    else:
        direction_deg = last_direction_deg
        
    # ---- File IPC: Write to Plotter ----
    # Append the newest cross-correlated wind vector to the text file
    try:
        with open("mine.txt", "a") as f:
            f.write(f"{wind_speed:.3f},{direction_deg:.1f}\n")
    except (PermissionError, IOError):
        # If the Plotter is currently reading the file, ignore and skip this frame
        pass

    # ---- Terminal readout --------------------------------------------------
    cal_tag = '[CAL]' if calib_active else f'[off {offset_ns:+.1f}/{offset_ew:+.1f}]'
    
    # --- MODIFIED: Added sound_speed_val to terminal print ---
    sys.stdout.write(
        f"\r {cal_tag}  Wind: {wind_speed:6.3f} m/s | "
        f"Dir: {direction_deg:6.1f}° {direction_label(direction_deg):<3s} | "
        f"Lag N/S: {smooth_lag_ns:+7.2f} | Lag E/W: {smooth_lag_ew:+7.2f} | "
        f"SndSpd: {sound_speed_val:6.2f} m/s | "
        f"Temp: {((sound_speed_val-331.3)/0.606):3.2f}"
    )
    sys.stdout.flush()

    # ---- Update compass arrow ----------------------------------------------
    ax_px,  ay_px  = rotate_point(0,  0.65, direction_deg)
    tail_x, tail_y = rotate_point(0, -0.30, direction_deg)

    arrow_line.set_data([0, ax_px],  [0, ay_px])
    arrow_head.set_data([ax_px],     [ay_px])
    arrow_head.set_marker((3, 0, direction_deg))
    arrow_tail.set_data([0, tail_x], [0, tail_y])

    radius = min(wind_speed / MAX_DISPLAY_SPEED, 1.0) * 0.85
    speed_circle.set_radius(max(radius, 0.01))

    # ---- Update compass text (offset_txt from main) ------------------------
    spd_txt.set_text(f'{wind_speed:.1f} m/s')
    dir_txt.set_text(f'{direction_deg:.0f}° {direction_label(direction_deg)}')
    lag_txt.set_text(f'N/S {smooth_lag_ns:+.2f}  E/W {smooth_lag_ew:+.2f}')
    offset_txt.set_text(f'off ns:{offset_ns:+.2f}  ew:{offset_ew:+.2f}')

    # ---- Update signal plots -----------------------------------------------
    raw_lim  = max(np.max(np.abs(s)) for s in [south, north, west, east])         * 1.2 + 1
    filt_lim = max(np.max(np.abs(s)) for s in [south_f, north_f, west_f, east_f]) * 1.2 + 1

    plot_data = [
        (south,   raw_lim),  (south_f, filt_lim),
        (north,   raw_lim),  (north_f, filt_lim),
        (west,    raw_lim),  (west_f,  filt_lim),
        (east,    raw_lim),  (east_f,  filt_lim),
    ]
    for idx, (data, ylim) in enumerate(plot_data):
        sig_lines[idx].set_ydata(data)
        sig_axes[idx].set_ylim(-ylim, ylim)

    return _ALL_ARTISTS()

#   ┌────────────────────────────────────────────────────────────────────────────┐
#   │ ENTRY POINT                                                                │
#   └────────────────────────────────────────────────────────────────────────────┘

if __name__ == '__main__':
    print("=" * 87)
    print(" Wind Anemometer — press C in the plot window to calibrate (keep sensors still).")
    print(f" Calibration collects {CALIB_DURATION:.0f}s of lags and stores the median as offset.")
    print(f" Reflector path = 2 × SENSOR_DISTANCE = {EFFECTIVE_DISTANCE} m")
    print("=" * 87)
    print()

    reader_thread = threading.Thread(
        target=serial_reader,
        args=(PORT, BAUD),
        daemon=True
    )
    reader_thread.start()

    try:
        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\n")
        stop_event.set()
        plt.close('all')
        print("Stopped.")