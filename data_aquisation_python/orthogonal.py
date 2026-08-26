import os
import queue
import serial
import struct
import sys
import threading
import time
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, correlate, filtfilt
from scipy.signal import hilbert

print("\n\n")

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 1. SYSTEM CONFIGURATION & HARDWARE SETTINGS                                │
#   └────────────────────────────────────────────────────────────────────────────┘

# Serial Interface Settings
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = 921600

# Packet Hardware Headers (North -> South -> East -> West)
HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55
HEADER_WESTOUT  = 0xCC55
HEADER_EASTOUT  = 0xDD55
HEADER_TEMP     = 0xEE55

# Physical & DSP Parameters
PAYLOAD_SAMPLES = 350
FS              = 1e6       # Sampling Frequency (1 MHz)
SENSOR_DISTANCE = 0.210     # Distance between transducers (meters)
SOUND_SPEED     = 343.0     # Speed of sound at ~20°C (m/s)
CALIB_DURATION  = 5.0       # Zero-wind calibration duration (seconds)

# Data Buffers & Queues
frame_queue    = queue.Queue(maxsize=4)
stop_event     = threading.Event()
lag_buffer_ns  = deque(maxlen=30)
wind_buffer_ns = deque(maxlen=30)
lag_buffer_ew  = deque(maxlen=30)
wind_buffer_ew = deque(maxlen=30)

# Temperature state (updated by serial_reader, read by update())
latest_sound_speed = SOUND_SPEED
temp_lock          = threading.Lock()

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 2. CALIBRATION & FILTERING STATE                                           │
#   └────────────────────────────────────────────────────────────────────────────┘

# Zero-Wind Static Calibration Variables
calib_lock_zero       = threading.Lock()
calib_active_zero     = False
calib_start_time_zero = 0.0
calib_lags_ns_zero    = []
calib_lags_ew_zero    = []
offset_ns_zero        = 0.0
offset_ew_zero        = 0.0

# Dynamic Drift Adaption Variables
OFFSET_ALPHA  = 0.002      # Slow baseline tracking factor
ZERO_WIND_TH  = 0.35       # Wind speed floor for drift tracking (m/s)
LAG_STABLE_TH = 0.8        # Maximum sample variance allowed for tuning stability

# Vector Smoothing & Thresholding Variables
smooth_ns          = 0.0
smooth_ew          = 0.0
DIR_VECTOR_ALPHA   = 0.1
MIN_DIR_SPEED      = 0.15
last_direction_deg = 0.0

# Cached Signal Objects (Speeds up processing)
_butter_cache = {}
_hanning      = np.hanning(PAYLOAD_SAMPLES)

# Axis-limit smoothing state. Recomputing set_ylim() every frame from the
# instantaneous noisy max (a) invalidates blit's cached background so it
# forces a full redraw every tick, defeating blit and dropping FPS, and
# (b) makes the visible range flicker/oscillate with sample noise. We track
# a slow EMA of the limit and only call set_ylim when it's actually moved
# enough to matter.
_raw_lim_smooth   = 100.0
_filt_lim_smooth  = 40.0
LIM_EMA_ALPHA     = 0.08   # how fast the tracked limit follows the signal
LIM_UPDATE_THRESH = 0.12   # only call set_ylim if it moved >12% from current

ratio_pos_ns = 1.00 # considered this to be correct  (this gives symmetery but a little shift but can be removed during zero calibration)
ratio_neg_ns = 0.9024
ratio_pos_ew = 0.9487
ratio_neg_ew = 0.8810

PRE_LOWPASS_CUTOFF = 60e3   # just above the 30-50k band; trims HF noise/aliasing

# --- Crosstalk gating (fixed-delay direct coupling removal) ---
# Crosstalk couples into the receiver almost immediately after the transmit
# pulse fires and decays quickly, at a FIXED sample offset every frame.
# The real echo arrives later and its delay is what we're measuring, so we
# blank the early fixed-delay region before any filtering/correlation.
# TUNE THIS: zoom into a raw buffer right after the transmit pulse and find
# the sample index where the crosstalk burst has decayed into noise floor.
GATE_SAMPLES = 150
GATE_RAMP    = 20   # short cosine ramp so we don't introduce a step edge

def get_lowpass_coeffs(cutoff, fs, order=4):
    key = ('lp', cutoff, fs, order)
    if key not in _butter_cache:
        nyq = 0.5 * fs
        b, a = butter(order, cutoff / nyq, btype='low')
        _butter_cache[key] = (b, a)
    return _butter_cache[key]

def lowpass_filter(data, cutoff, fs, order=4):
    b, a = get_lowpass_coeffs(cutoff, fs, order)
    return filtfilt(b, a, data)

def gate_signal(data, gate_samples=GATE_SAMPLES, ramp=GATE_RAMP):
    """Zero out the fixed-delay crosstalk region before filtering/correlation.
    Uses a short raised-cosine ramp instead of a hard cutoff to avoid
    introducing spectral leakage from a step discontinuity."""
    gated = data.copy()
    if gate_samples <= 0:
        return gated
    gated[:gate_samples] = 0.0
    if ramp > 0 and gate_samples + ramp <= len(gated):
        ramp_win = np.hanning(ramp * 2)[:ramp]
        gated[gate_samples:gate_samples + ramp] *= ramp_win
    return gated

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 3. DIGITAL SIGNAL PROCESSING (DSP) HELPER FUNCTIONS                        │
#   └────────────────────────────────────────────────────────────────────────────┘

def get_butter_coeffs(lc, hc, fs, order=4):
    """Calculates and caches bandpass filter coefficients to save CPU time."""
    key = (lc, hc, fs, order)
    if key not in _butter_cache:
        nyq = 0.5 * fs
        b, a = butter(order, [lc / nyq, hc / nyq], btype='band')
        _butter_cache[key] = (b, a)
    return _butter_cache[key]

def bandpass_filter(data, lc, hc, fs, order=4):
    """Applies a zero-phase forward-backward Butterworth filter."""
    b, a = get_butter_coeffs(lc, hc, fs, order)
    return filtfilt(b, a, data)

def smooth_median(buf, val, threshold=0.7):
    """Applies a localized variance filter to smooth out sudden noise spikes."""
    buf.append(val)
    med = np.median(buf)
    clean = [x for x in buf if abs(x - med) < threshold] or list(buf)
    return float(np.median(clean))

def get_lag(a, b):
    """Calculates fine-grained phase delta via cross-correlation and parabolic fit."""
    na = a / (a.std() + 1e-8)
    nb = b / (b.std() + 1e-8)
    corr = correlate(na, nb, mode='full')
    lags = np.arange(-len(na) + 1, len(na))
    i = int(np.argmax(corr))
    lag = float(lags[i])
    if 0 < i < len(corr) - 1:
        y0, y1, y2 = corr[i - 1], corr[i], corr[i + 1]
        lag += (y0 - y2) / (2 * (y0 - 2 * y1 + y2) + 1e-8)
    return lag

def get_echo_window(sig, thresh_frac=0.5, min_len=100):
    """Find where the signal envelope reaches steady-state amplitude and
    return a boolean mask covering only that region. The transducer's
    ring-up transient (low, uneven amplitude at the start of the burst)
    is excluded so cross-correlation only sees the clean, high-SNR
    steady-state portion of the echo — keeps lag estimates from being
    dragged off by phase inconsistency in the ramp-up region."""
    env = np.abs(hilbert(sig))
    env_smooth = np.convolve(env, np.ones(15) / 15, mode='same')
    peak = env_smooth.max()
    if peak <= 0:
        return np.ones(len(sig), dtype=bool)
    onset_idx = int(np.argmax(env_smooth > thresh_frac * peak))
    end_idx   = min(onset_idx + max(min_len, len(sig) - onset_idx), len(sig))
    mask = np.zeros(len(sig), dtype=bool)
    mask[onset_idx:end_idx] = True
    return mask

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 4. SERIAL DATA INGESTION PIPELINE                                          │
#   └────────────────────────────────────────────────────────────────────────────┘

def serial_reader(port, baud):
    """Background worker thread responsible for reading frame packets safely."""
    global latest_sound_speed

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[serial_reader] ERROR: {e}")
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
        while not stop_event.is_set():
            b = ser.read(1)
            if not b: continue
            b2 = ser.read(1)
            if not b2: continue
            if (b[0] | (b2[0] << 8)) == expected:
                return True
        return False

    def read_payload():
        data = read_exact(PAYLOAD_SAMPLES * 2)
        if len(data) != PAYLOAD_SAMPLES * 2:
            return None
        return np.array(struct.unpack('<' + 'H' * PAYLOAD_SAMPLES, data), dtype=np.float32)

    def read_temperature():
        data = read_exact(4)
        if len(data) != 4:
            return None
        return struct.unpack('<f', data)[0]

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

            if expect_header(HEADER_TEMP):
                spd = read_temperature()
                if spd is not None and 300.0 < spd < 380.0:
                    with temp_lock:
                        latest_sound_speed = spd

            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put_nowait((s1, s2, s3, s4))

        except Exception as e:
            if not stop_event.is_set():
                print(f"[serial_reader] Exception encountered: {e}")

    ser.close()

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 5. MATPLOTLIB VISUALIZATION SETUP                                          │
#   └────────────────────────────────────────────────────────────────────────────┘

BG  = "#ffffff"
RED = '#e24b4a'
GRN = '#00c97d'
BLK = "#000000"

fig = plt.figure(figsize=(8,6), facecolor=BG)
gs  = fig.add_gridspec(4, 3, left=0.04, right=0.96, top=0.95, bottom=0.04, hspace=0.35, wspace=0.3)

sig_axes   = [fig.add_subplot(gs[r, c]) for r in range(4) for c in range(2)]
ax_compass = fig.add_subplot(gs[:, 2])

LABELS = ['N RAW', 'N FILT', 'S RAW', 'S FILT', 'E RAW', 'E FILT', 'W RAW', 'W FILT']
sig_lines = []
x = np.arange(PAYLOAD_SAMPLES)

for i, ax in enumerate(sig_axes):
    ln, = ax.plot(x, np.zeros(PAYLOAD_SAMPLES), lw=0.7)
    ax.set_ylim(-100, 100)
    ax.set_title(LABELS[i], color=BLK, fontsize=7, pad=1)
    sig_lines.append(ln)

ax_compass.set_facecolor(BG)
ax_compass.set_aspect('equal')
ax_compass.set_xlim(-1.3, 1.3)
ax_compass.set_ylim(-1.45, 1.45)
ax_compass.axis('off')

for r, alpha in [(1.0, 0.25), (0.7, 0.15), (0.4, 0.10)]:
    ax_compass.add_patch(plt.Circle((0, 0), r, color=BLK, fill=False, lw=0.5, alpha=alpha))

CARDINALS = [('N', 0), ('NE', 45), ('E', 90), ('SE', 135), ('S', 180), ('SW', 225), ('W', 270), ('NW', 315)]
for label, deg in CARDINALS:
    rad = np.radians(deg)
    ax_compass.plot([np.sin(rad) * 0.90, np.sin(rad) * 1.00],
                    [np.cos(rad) * 0.90, np.cos(rad) * 1.00], color=BLK, lw=0.8)
    is_card = len(label) == 1
    ax_compass.text(np.sin(rad) * 1.15, np.cos(rad) * 1.15, label,
                    ha='center', va='center', fontsize=9 if is_card else 6,
                    color=BLK, fontweight='bold' if is_card else 'normal')

for deg in range(0, 360, 10):
    if deg % 45 == 0: continue
    rad = np.radians(deg)
    ax_compass.plot([np.sin(rad) * 0.95, np.sin(rad) * 1.00],
                    [np.cos(rad) * 0.95, np.cos(rad) * 1.00], color=BLK, lw=0.4)

speed_circle = plt.Circle((0, 0), 0.01, color=RED, alpha=0.15, fill=True)
ax_compass.add_patch(speed_circle)

arrow_line, = ax_compass.plot([0, 0], [0,  0.65], color=RED, lw=2.5, solid_capstyle='round')
arrow_head, = ax_compass.plot([0], [0.65], marker='^', ms=8, color=RED, markeredgewidth=0)
arrow_tail, = ax_compass.plot([0, 0], [0, -0.30], color=BLK, lw=1.2, linestyle='--', dash_capstyle='round')
ax_compass.plot(0, 0, 'o', color=BLK, ms=4, zorder=5)

spd_txt    = ax_compass.text(0, -1.08, '-- m/s',           ha='center', va='center', fontsize=10,  color=RED, fontfamily='monospace')
dir_txt    = ax_compass.text(0,  1.25, '--°',              ha='center', va='center', fontsize=8,   color=BLK, fontfamily='monospace')
lag_txt    = ax_compass.text(0, -1.20, 'N/S: --  E/W: --', ha='center', va='center', fontsize=6.5, color=BLK, fontfamily='monospace')
offset_txt = ax_compass.text(0, -1.32, 'offset: 0 / 0',    ha='center', va='center', fontsize=6.5, color=BLK, fontfamily='monospace')
calib_txt  = ax_compass.text(0, -1.43, '',                  ha='center', va='center', fontsize=7,   color=GRN, fontfamily='monospace')

ax_compass.text(0, 1.38, 'WIND', ha='center', va='center', fontsize=9, color=BLK, fontfamily='monospace', fontweight='bold')

def dir_label(deg):
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    return dirs[int((deg + 11.25) / 22.5) % 16]

def rotate_point(px, py, deg):
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return c * px - s * py, s * px + c * py

MAX_SPEED = 7.0

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 6. INTERACTIVE CALIBRATION HANDLERS (KEYBOARD EVENTS)                      │
#   └────────────────────────────────────────────────────────────────────────────┘

def on_key(event):
    """Handles keyboard presses for calibration."""
    global calib_active_zero, calib_start_time_zero, calib_lags_ns_zero, calib_lags_ew_zero

    # 'C' Key: Execute Local Zero-Wind Offset Auto-Calibration
    if event.key in ('c', 'C'):
        with calib_lock_zero:
            if calib_active_zero:
                return
            calib_active_zero     = True
            calib_start_time_zero = time.time()
            calib_lags_ns_zero    = []
            calib_lags_ew_zero    = []
        print(f"\n [CAL          ] Calibration started — keep still for {CALIB_DURATION:.0f}s ...")

fig.canvas.mpl_connect('key_press_event', on_key)

# --- Window-close handling ---
# Root cause of the AttributeError traceback: closing the Tk window doesn't
# instantly stop FuncAnimation's timer. One more _on_timer tick can fire
# after Tk has already started tearing down the canvas, so blit's
# restore_region() call hits a canvas that no longer has that attribute.
# We stop the timer and set stop_event as soon as the close event fires,
# before Tk finishes destroying anything.
def on_close(event):
    stop_event.set()
    try:
        if ani.event_source is not None:
            ani.event_source.stop()
    except Exception:
        pass

fig.canvas.mpl_connect('close_event', on_close)

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 7. MAIN RUNTIME ITERATION (ANIMATION UPDATE LOOP)                          │
#   └────────────────────────────────────────────────────────────────────────────┘

def update(frame_num):
    """This function is called continuously by Matplotlib to update the graphs."""
    global calib_active_zero, offset_ns_zero, offset_ew_zero
    global smooth_ns, smooth_ew, last_direction_deg
    global _raw_lim_smooth, _filt_lim_smooth

    artists = [arrow_line, arrow_head, arrow_tail, speed_circle, spd_txt, dir_txt, lag_txt, offset_txt, calib_txt]

    # Bail out immediately if we're shutting down (window closed) so we
    # never touch a canvas that Tk is in the middle of destroying.
    if stop_event.is_set():
        return sig_lines + artists

    try:
        s1, s2, s3, s4 = frame_queue.get_nowait()
    except queue.Empty:
        return sig_lines + artists

    # Snapshot latest temperature-derived sound speed (thread-safe)
    with temp_lock:
        sound_speed = latest_sound_speed

    # --- SIGNAL PRE-PROCESSING ---
    s1 = s1 - s1.mean()
    s2 = s2 - s2.mean()
    s3 = s3 - s3.mean()
    s4 = s4 - s4.mean()

    # Gate out the fixed-delay crosstalk burst before any filtering, so the
    # bandpass/correlation stage only ever sees the echo, not the direct
    # electrical/acoustic coupling that shares the same frequency band.
    #s1 = gate_signal(s1)
    #s2 = gate_signal(s2)
    #s3 = gate_signal(s3)
    #s4 = gate_signal(s4)

    s1 = lowpass_filter(s1, PRE_LOWPASS_CUTOFF, FS)
    s2 = lowpass_filter(s2, PRE_LOWPASS_CUTOFF, FS)
    s3 = lowpass_filter(s3, PRE_LOWPASS_CUTOFF, FS)
    s4 = lowpass_filter(s4, PRE_LOWPASS_CUTOFF, FS)

    s1f = bandpass_filter(s1 * _hanning, 30e3, 50e3, FS)
    s2f = bandpass_filter(s2 * _hanning, 30e3, 50e3, FS)
    s3f = bandpass_filter(s3 * _hanning, 30e3, 50e3, FS)
    s4f = bandpass_filter(s4 * _hanning, 30e3, 50e3, FS)

    # Restrict correlation to the steady-state burst region so the
    # transducer's low-amplitude ring-up transient doesn't drag the lag
    # estimate off — see get_echo_window().
    mask_ns = get_echo_window(s1f) & get_echo_window(s2f)
    mask_ew = get_echo_window(s3f) & get_echo_window(s4f)

    raw_lag_ns = get_lag(s1f[mask_ns], s2f[mask_ns])
    raw_lag_ew = get_lag(s3f[mask_ew], s4f[mask_ew])

    # --- CALIBRATION LOGIC ---
    with calib_lock_zero:
        if calib_active_zero:
            calib_lags_ns_zero.append(raw_lag_ns)
            calib_lags_ew_zero.append(raw_lag_ew)
            elapsed   = time.time() - calib_start_time_zero
            remaining = CALIB_DURATION - elapsed

            if remaining > 0:
                calib_txt.set_text(f'CALIBRATING... {remaining:.1f}s')
                calib_txt.set_color(GRN)
            else:
                offset_ns_zero    = float(np.median(calib_lags_ns_zero))
                offset_ew_zero    = float(np.median(calib_lags_ew_zero))
                calib_active_zero = False
                lag_buffer_ns.clear(); wind_buffer_ns.clear()
                lag_buffer_ew.clear(); wind_buffer_ew.clear()
                calib_txt.set_text(f'CAL DONE  ns={offset_ns_zero:+.2f}  ew={offset_ew_zero:+.2f}')
                calib_txt.set_color(GRN)
                print(f"\n [CAL          ] Done — offset N/S: {offset_ns_zero:+.5f} samples | E/W: {offset_ew_zero:+.5f} samples\n\n ------>\n")

    # --- WIND VECTOR CALCULATIONS ---
    corrected_lag_ns = raw_lag_ns - offset_ns_zero
    corrected_lag_ew = raw_lag_ew - offset_ew_zero

    tmp_wind_ns = -(corrected_lag_ns / FS) * (sound_speed**2 / SENSOR_DISTANCE)
    tmp_wind_ew = -(corrected_lag_ew / FS) * (sound_speed**2 / SENSOR_DISTANCE)
    tmp_speed   = np.sqrt(tmp_wind_ns**2 + tmp_wind_ew**2)

    stable_ns = len(lag_buffer_ns) > 5 and np.std(lag_buffer_ns) < LAG_STABLE_TH
    stable_ew = len(lag_buffer_ew) > 5 and np.std(lag_buffer_ew) < LAG_STABLE_TH

    if tmp_speed < ZERO_WIND_TH and stable_ns and stable_ew and not calib_active_zero:
        offset_ns_zero = (1 - OFFSET_ALPHA) * offset_ns_zero + OFFSET_ALPHA * raw_lag_ns
        offset_ew_zero = (1 - OFFSET_ALPHA) * offset_ew_zero + OFFSET_ALPHA * raw_lag_ew

    corrected_lag_ns = raw_lag_ns - offset_ns_zero
    corrected_lag_ew = raw_lag_ew - offset_ew_zero
    
    corrected_lag_ns *= 1.0          if corrected_lag_ns >= 0 else ratio_neg_ns
    corrected_lag_ew *= ratio_pos_ew if corrected_lag_ew >= 0 else ratio_neg_ew

    wind_ns =  -(corrected_lag_ns / FS) * (sound_speed**2 / SENSOR_DISTANCE)
    wind_ew =  (corrected_lag_ew / FS) * (sound_speed**2 / SENSOR_DISTANCE)

    lag_sn  = smooth_median(lag_buffer_ns,  corrected_lag_ns)
    wind_sn = smooth_median(wind_buffer_ns, wind_ns)
    lag_se  = smooth_median(lag_buffer_ew,  corrected_lag_ew)
    wind_se = smooth_median(wind_buffer_ew, wind_ew)

    smooth_ns  = (1 - DIR_VECTOR_ALPHA) * smooth_ns + DIR_VECTOR_ALPHA * wind_sn
    smooth_ew  = (1 - DIR_VECTOR_ALPHA) * smooth_ew + DIR_VECTOR_ALPHA * wind_se
    wind_speed = np.sqrt(smooth_ns**2 + smooth_ew**2)

    if wind_speed > MIN_DIR_SPEED:
        direction_rad      = np.arctan2(smooth_ew, smooth_ns)
        direction_deg      = (np.degrees(direction_rad) + 360) % 360
        last_direction_deg = direction_deg
    else:
        direction_deg = last_direction_deg

    display_temp_c = (sound_speed - 331.3) / 0.606

    # --- UPDATE USER INTERFACE & TERMINAL ---
    cal_marker = '[CAL          ]' if calib_active_zero else f'[off {offset_ns_zero:+.1f}/{offset_ew_zero:+.1f}]'
    sys.stdout.write(
        f"\r {cal_marker} Wind: {wind_speed:3.5f} m/s | Dir: {direction_deg:6.2f}° "
        f"{dir_label(direction_deg):<3s} | Lag N/S: {lag_sn:+3.3f} | Lag E/W: {lag_se:+3.3f} | "
        f"Temp: {display_temp_c:+.1f}°C  Vs: {sound_speed:.1f}m/s   "
    )
    sys.stdout.flush()

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
    offset_txt.set_text(f'off ns:{offset_ns_zero:+.2f} ew:{offset_ew_zero:+.2f}  Vs:{sound_speed:.1f}m/s  T:{display_temp_c:+.1f}°C')

    raw_lim_now  = max(np.max(np.abs(s1)), np.max(np.abs(s2)), np.max(np.abs(s3)), np.max(np.abs(s4))) * 1.2 + 1
    filt_lim_now = max(np.max(np.abs(s1f)), np.max(np.abs(s2f)), np.max(np.abs(s3f)), np.max(np.abs(s4f))) * 1.2 + 1

    _raw_lim_smooth  = (1 - LIM_EMA_ALPHA) * _raw_lim_smooth  + LIM_EMA_ALPHA * raw_lim_now
    _filt_lim_smooth = (1 - LIM_EMA_ALPHA) * _filt_lim_smooth + LIM_EMA_ALPHA * filt_lim_now

    plot_configs = [
        (0, s1, _raw_lim_smooth),  (1, s1f, _filt_lim_smooth), (2, s2, _raw_lim_smooth),  (3, s2f, _filt_lim_smooth),
        (4, s3, _raw_lim_smooth),  (5, s3f, _filt_lim_smooth), (6, s4, _raw_lim_smooth),  (7, s4f, _filt_lim_smooth)
    ]
    for idx, data, lim in plot_configs:
        sig_lines[idx].set_ydata(data)
        cur_lo, cur_hi = sig_axes[idx].get_ylim()
        # Only touch the axis (and invalidate blit's cached background)
        # when the limit has actually drifted enough to matter.
        if cur_hi <= 0 or abs(lim - cur_hi) > LIM_UPDATE_THRESH * cur_hi:
            sig_axes[idx].set_ylim(-lim, lim)

    return sig_lines + artists

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 8. EXECUTION CONTEXT ENTRYPOINT                                            │
#   └────────────────────────────────────────────────────────────────────────────┘

if __name__ == '__main__':
    print("Ultrasonic Wind Anemometer Telemetry Processor")
    print("  C  →  Zero-wind offset calibration  (Keep system stable)\n")
    print(f"C-calibration samples {CALIB_DURATION:.0f}s of raw packet phase lags to establish an offset.\n")

    reader = threading.Thread(target=serial_reader, args=(PORT, BAUD), daemon=True)
    reader.start()

    try:
        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)

        # blit=True is smooth but can race on window close: Tk may already
        # have a timer tick queued when the window is destroyed, so it
        # fires restore_region() on a canvas that no longer has it
        # (AttributeError: FigureCanvasBase has no attribute
        # 'restore_region'). This happens AFTER the window is already gone,
        # so it's cosmetic, not a real fault. Rather than give up blit
        # performance to dodge it, silence only this exact error and let
        # every other exception print normally.
        def _suppress_post_close_blit_error(exc_type, exc_value, exc_tb):
            if isinstance(exc_value, AttributeError) and 'restore_region' in str(exc_value):
                return
            import traceback as _tb
            _tb.print_exception(exc_type, exc_value, exc_tb)

        try:
            fig.canvas.manager.window.report_callback_exception = _suppress_post_close_blit_error
        except Exception:
            pass
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        # ani.event_source can already be None here — on_close() or Tk's own
        # teardown may have cleared it before we get to this line. Guard it.
        if 'ani' in dir() and getattr(ani, 'event_source', None) is not None:
            ani.event_source.stop()
        plt.close('all')

        print("\n\nExecution terminated successfully.\n\n")