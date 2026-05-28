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
PORT = "COM8"
BAUD = 921600

# Packet Hardware Headers (North -> South -> East -> West)
HEADER_SOUTHOUT = 0xAA55
HEADER_NORTHOUT = 0xBB55
HEADER_WESTOUT  = 0xCC55
HEADER_EASTOUT  = 0xDD55

# Physical & DSP Parameters
PAYLOAD_SAMPLES = 220
FS              = 1e6       # Sampling Frequency (1 MHz)
SENSOR_DISTANCE = 0.210     # Distance between transducers (meters)
SOUND_SPEED     = 343.0     # Speed of sound at ~20°C (m/s)
CALIB_DURATION  = 3.0       # Zero-wind calibration duration (seconds)

# Data Buffers & Queues
frame_queue    = queue.Queue(maxsize=4)
stop_event     = threading.Event()
lag_buffer_ns  = deque(maxlen=30)
wind_buffer_ns = deque(maxlen=30)
lag_buffer_ew  = deque(maxlen=30)
wind_buffer_ew = deque(maxlen=30)

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

# Transducer Geometric Asymmetry Scales (Reference = 1.0)
POS_SCALE_NS = 1.0 
NEG_SCALE_NS = 0.94117647
POS_SCALE_EW = 1.09090909
NEG_SCALE_EW = 0.84210526

# manually done by me and thus errornous
# to s +4.8 pos_ns
# to n -5.1 neg_ns
# to w +4.4 pos_ew 
# to e -5.7 neg_ew
# considering +ns as correct the others are
# any other * x = pos_ns = 4.8
# tested and assymetery works fine but auto would be much better (maybe on kismat)

# Asymmetry Multistage Guide Mapping
ASYM_STAGES = [
    '+NS  (wind N→S)', 
    '-NS  (wind S→N)', 
    '+EW  (wind E→W)', 
    '-EW  (wind W→E)'
]
ASYM_INSTRUCT = [
    'blow wind North → South',
    'blow wind South → North',
    'blow wind East  → West ',
    'blow wind West  → East ',
]

asym_lock   = threading.Lock()
asym_stage  = -1            # -1 means Idle state
asym_buffer = deque(maxlen=30)
asym_means  = [None, None, None, None]

# Cached Signal Objects (Speeds up processing)
_butter_cache = {}
_hanning      = np.hanning(PAYLOAD_SAMPLES)

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
    # Only keep values that are close to the median (removes extreme outliers)
    clean = [x for x in buf if abs(x - med) < threshold] or list(buf)
    return float(np.median(clean))

def get_lag(a, b):
    
    """Calculates fine-grained phase delta via cross-correlation and parabolic fit."""
    # Standardize the signals to prevent amplitude differences from messing up timing
    na = a / (a.std() + 1e-8)
    nb = b / (b.std() + 1e-8)
    
    # Cross-correlate to find how much signal 'b' is delayed compared to 'a'
    corr = correlate(na, nb, mode='full')
    lags = np.arange(-len(na) + 1, len(na))
    
    # Find the peak correlation index
    i = int(np.argmax(corr))
    lag = float(lags[i])
    
    # Parabolic interpolation: Guess the "true" peak between the digital samples
    if 0 < i < len(corr) - 1:
        y0, y1, y2 = corr[i - 1], corr[i], corr[i + 1]
        lag += (y0 - y2) / (2 * (y0 - 2 * y1 + y2) + 1e-8)
        
    return lag

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 4. SERIAL DATA INGESTION PIPELINE                                          │
#   └────────────────────────────────────────────────────────────────────────────┘

def serial_reader(port, baud):
    """Background worker thread responsible for reading frame packets safely."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[serial_reader] ERROR: {e}")
        stop_event.set()
        return

    def read_exact(n):
        """Helper to ensure we read exactly 'n' bytes, waiting if necessary."""
        buf = b''
        while len(buf) < n and not stop_event.is_set():
            chunk = ser.read(n - len(buf))
            if chunk: 
                buf += chunk
        return buf

    def expect_header(expected):
        """Scans the serial stream byte-by-byte until the specific header is found."""
        while not stop_event.is_set():
            b = ser.read(1)
            if not b: continue
            b2 = ser.read(1)
            if not b2: continue
            
            # Combine two bytes and check if they match the expected header
            if (b[0] | (b2[0] << 8)) == expected: 
                return True
        return False

    def read_payload():
        """Reads the raw sensor data and converts it into a numpy float array."""
        data = read_exact(PAYLOAD_SAMPLES * 2) # 2 bytes per sample (uint16)
        if len(data) != PAYLOAD_SAMPLES * 2: 
            return None
        return np.array(struct.unpack('<' + 'H' * PAYLOAD_SAMPLES, data), dtype=np.float32)

    # Main serial reading loop
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

            # Put the 4 signals into the queue for the main program to process
            if frame_queue.full():
                try: 
                    frame_queue.get_nowait() # Remove oldest if full
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

# Setup main figure and layout grid
fig = plt.figure(figsize=(12, 7), facecolor=BG)
gs  = fig.add_gridspec(4, 3, left=0.04, right=0.96, top=0.95, bottom=0.04, hspace=0.35, wspace=0.3)

# Create 8 subplots for signal waves and 1 for the compass
sig_axes   = [fig.add_subplot(gs[r, c]) for r in range(4) for c in range(2)]
ax_compass = fig.add_subplot(gs[:, 2])

LABELS = ['N RAW', 'N FILT', 'S RAW', 'S FILT', 'E RAW', 'E FILT', 'W RAW', 'W FILT']
sig_lines = []
x = np.arange(PAYLOAD_SAMPLES)

# Initialize lines for the signal plots
for i, ax in enumerate(sig_axes):
    ln, = ax.plot(x, np.zeros(PAYLOAD_SAMPLES), lw=0.7)
    ax.set_ylim(-100, 100)
    ax.set_title(LABELS[i], color=BLK, fontsize=7, pad=1)
    sig_lines.append(ln)

# Construct Compass GUI
ax_compass.set_facecolor(BG)
ax_compass.set_aspect('equal')
ax_compass.set_xlim(-1.3, 1.3)
ax_compass.set_ylim(-1.45, 1.45)
ax_compass.axis('off')

# Draw compass rings
for r, alpha in [(1.0, 0.25), (0.7, 0.15), (0.4, 0.10)]:
    ax_compass.add_patch(plt.Circle((0, 0), r, color=BLK, fill=False, lw=0.5, alpha=alpha))

# Draw cardinal directions (N, E, S, W, etc.)
CARDINALS = [('N', 0), ('NE', 45), ('E', 90), ('SE', 135), ('S', 180), ('SW', 225), ('W', 270), ('NW', 315)]
for label, deg in CARDINALS:
    rad = np.radians(deg)
    ax_compass.plot([np.sin(rad) * 0.90, np.sin(rad) * 1.00],
                    [np.cos(rad) * 0.90, np.cos(rad) * 1.00], color=BLK, lw=0.8)
    is_card = len(label) == 1
    ax_compass.text(np.sin(rad) * 1.15, np.cos(rad) * 1.15, label,
                    ha='center', va='center', fontsize=9 if is_card else 6,
                    color=BLK, fontweight='bold' if is_card else 'normal')

# Draw minor degree tick marks
for deg in range(0, 360, 10):
    if deg % 45 == 0: continue
    rad = np.radians(deg)
    ax_compass.plot([np.sin(rad) * 0.95, np.sin(rad) * 1.00],
                    [np.cos(rad) * 0.95, np.cos(rad) * 1.00], color=BLK, lw=0.4)

# Dynamic compass UI elements (updated in real-time)
speed_circle = plt.Circle((0, 0), 0.01, color=RED, alpha=0.15, fill=True)
ax_compass.add_patch(speed_circle)

arrow_line, = ax_compass.plot([0, 0], [0,  0.65], color=RED, lw=2.5, solid_capstyle='round')
arrow_head, = ax_compass.plot([0], [0.65], marker='^', ms=8, color=RED, markeredgewidth=0)
arrow_tail, = ax_compass.plot([0, 0], [0, -0.30], color=BLK, lw=1.2, linestyle='--', dash_capstyle='round')
ax_compass.plot(0, 0, 'o', color=BLK, ms=4, zorder=5)

spd_txt    = ax_compass.text(0, -1.08, '-- m/s',          ha='center', va='center', fontsize=10,  color=RED, fontfamily='monospace')
dir_txt    = ax_compass.text(0,  1.25, '--°',             ha='center', va='center', fontsize=8,   color=BLK, fontfamily='monospace')
lag_txt    = ax_compass.text(0, -1.20, 'N/S: --  E/W: --', ha='center', va='center', fontsize=6.5, color=BLK, fontfamily='monospace')
offset_txt = ax_compass.text(0, -1.32, 'offset: 0 / 0',    ha='center', va='center', fontsize=6.5, color=BLK, fontfamily='monospace')
calib_txt  = ax_compass.text(0, -1.43, '',                 ha='center', va='center', fontsize=7,   color=GRN, fontfamily='monospace')

ax_compass.text(0, 1.38, 'WIND', ha='center', va='center', fontsize=9, color=BLK, fontfamily='monospace', fontweight='bold')

def dir_label(deg):
    """Converts a degree reading into a compass direction string."""
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    return dirs[int((deg + 11.25) / 22.5) % 16]

def rotate_point(px, py, deg):
    """Rotates a 2D point around the origin by a specific degree."""
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return c * px - s * py, s * px + c * py

MAX_SPEED = 25.0




#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 6. INTERACTIVE CALIBRATION HANDLERS (KEYBOARD EVENTS)                      │
#   └────────────────────────────────────────────────────────────────────────────┘

def on_key(event):
    """Handles keyboard presses for calibration."""
    global calib_active_zero, calib_start_time_zero, calib_lags_ns_zero, calib_lags_ew_zero
    global asym_stage, asym_means, POS_SCALE_NS, NEG_SCALE_NS, POS_SCALE_EW, NEG_SCALE_EW

    # 'C' Key: Execute Local Zero-Wind Offset Auto-Calibration
    if event.key in ('c', 'C'):
        with calib_lock_zero:
            if calib_active_zero: 
                return  # Guard clause against concurrent triggers
            calib_active_zero     = True
            calib_start_time_zero = time.time()
            calib_lags_ns_zero    = []
            calib_lags_ew_zero    = []
        print(f"\n [CAL          ] Calibration started — keep still for {CALIB_DURATION:.0f}s ...")

    # 'V' Key: Progress Through Transducer Asymmetry Optimization Steps
    elif event.key in ('v', 'V'):
        with asym_lock:
            if asym_stage == -1:
                asym_stage = 0
                asym_buffer.clear()
                print(f"\n [ASYM CAL     ] ── V pressed 1/6 ── asymmetry calibration started")
                print(f"                  step 1 of 4 : measuring  +NS  lag")
                print(f"                  → blow wind steadily from  North → South")
                print(f"                  → buffer collects up to 30 samples automatically")
                print(f"                  → when ready, press V again to commit this reading\n")
                calib_txt.set_text(f'ASYM 1/4 {ASYM_STAGES[0]}')
                calib_txt.set_color(GRN)

            elif asym_stage == 4:
                # Apply the calculated asymmetry scales
                pos_ns, neg_ns, pos_ew, neg_ew = asym_means
                eps = 1e-6
                
                NEG_SCALE_NS = abs(pos_ns) / abs(neg_ns) if abs(neg_ns) > eps else 1.0
                if abs(neg_ns) <= eps:
                    print(" [ASYM CAL     ] WARNING: -NS mean ≈ 0, NEG_SCALE_NS left at 1.0")

                NEG_SCALE_EW = abs(pos_ew) / abs(neg_ew) if abs(neg_ew) > eps else 1.0
                if abs(neg_ew) <= eps:
                    print(" [ASYM CAL     ] WARNING: -EW mean ≈ 0, NEG_SCALE_EW left at 1.0")

                POS_SCALE_NS = 1.0
                POS_SCALE_EW = 1.0
                asym_stage   = -1  # Reset to idle tracking flow

                print(f"\n [ASYM CAL     ] ── V pressed 6/6 ── factors applied")
                print(f"                  pos_ns = {pos_ns:+.5f}   neg_ns = {neg_ns:+.5f}")
                print(f"                  pos_ew = {pos_ew:+.5f}   neg_ew = {neg_ew:+.5f}")
                print(f"                  NEG_SCALE_NS = {NEG_SCALE_NS:.5f}")
                print(f"                  NEG_SCALE_EW = {NEG_SCALE_EW:.5f}")
                print(f"                  → asymmetry compensation is now active")
                print(f"                  → run C-calibration next to re-zero the offset\n")

                calib_txt.set_text(f'ASYM DONE  Gns(-){NEG_SCALE_NS:.3f}  Gew(-){NEG_SCALE_EW:.3f}')
                calib_txt.set_color(GRN)

            else:
                # We are in the middle of calibration stages (0, 1, 2, or 3)
                n = len(asym_buffer)
                if n < 15:
                    print(f"\n [ASYM CAL     ] only {n}/15 samples collected — keep blowing, press V when ready\n")
                    return

                mean_val = float(np.mean(list(asym_buffer)))
                asym_means[asym_stage] = mean_val
                next_stage = asym_stage + 1
                press_num  = asym_stage + 2

                print(f"\n [ASYM CAL     ] ── V pressed {press_num}/6 ── {ASYM_STAGES[asym_stage]} committed | mean = {mean_val:+.5f} samples (n={n})")

                if next_stage < 4:
                    asym_stage = next_stage
                    asym_buffer.clear()
                    print(f"                  step {next_stage+1} of 4 : measuring  {ASYM_STAGES[next_stage]}")
                    print(f"                  → {ASYM_INSTRUCT[next_stage]}")
                    print(f"                  → buffer will collect up to 30 samples automatically")
                    print(f"                  → press V again when ready to commit\n")
                    calib_txt.set_text(f'ASYM {next_stage+1}/4 {ASYM_STAGES[next_stage]}')
                    calib_txt.set_color(GRN)
                else:
                    pos_ns, neg_ns, pos_ew, neg_ew = asym_means
                    eps = 1e-6
                    preview_neg_ns = abs(pos_ns) / abs(neg_ns) if abs(neg_ns) > eps else 1.0
                    preview_neg_ew = abs(pos_ew) / abs(neg_ew) if abs(neg_ew) > eps else 1.0
                    asym_stage = 4

                    print(f"                  all 4 readings collected — preview of factors:")
                    print(f"                    pos_ns = {pos_ns:+.5f}   neg_ns = {neg_ns:+.5f}   →  NEG_SCALE_NS = {preview_neg_ns:.5f}")
                    print(f"                    pos_ew = {pos_ew:+.5f}   neg_ew = {neg_ew:+.5f}   →  NEG_SCALE_EW = {preview_neg_ew:.5f}")
                    print(f"                  → press V one more time to APPLY these factors")
                    print(f"                  → or press V from the start to redo the whole process\n")
                    calib_txt.set_text(f'ASYM PREVIEW  Gns(-){preview_neg_ns:.3f}  Gew(-){preview_neg_ew:.3f}  — V to apply')
                    calib_txt.set_color(GRN)

# Register the keyboard listener
fig.canvas.mpl_connect('key_press_event', on_key)

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 7. MAIN RUNTIME ITERATION (ANIMATION UPDATE LOOP)                          │
#   └────────────────────────────────────────────────────────────────────────────┘

def update(frame_num):
    """This function is called continuously by Matplotlib to update the graphs."""
    global calib_active_zero, offset_ns_zero, offset_ew_zero
    global smooth_ns, smooth_ew, last_direction_deg
    global POS_SCALE_NS, NEG_SCALE_NS, POS_SCALE_EW, NEG_SCALE_EW

    # Artist elements tracking handle map
    artists = [arrow_line, arrow_head, arrow_tail, speed_circle, spd_txt, dir_txt, lag_txt, offset_txt, calib_txt]

    # Attempt to pull a new packet of data from the queue
    try:
        s1, s2, s3, s4 = frame_queue.get_nowait()
    except queue.Empty:
        return sig_lines + artists

    # --- SIGNAL PRE-PROCESSING ---
    # Demodulate DC offsets natively (center the signal around 0)
    s1 = s1 - s1.mean()
    s2 = s2 - s2.mean()
    s3 = s3 - s3.mean()
    s4 = s4 - s4.mean()

    # Apply Hanning Window to soften edges, then Bandpass Filter for ultrasonic freq
    s1f = bandpass_filter(s1 * _hanning, 30e3, 50e3, FS)
    s2f = bandpass_filter(s2 * _hanning, 30e3, 50e3, FS)
    s3f = bandpass_filter(s3 * _hanning, 30e3, 50e3, FS)
    s4f = bandpass_filter(s4 * _hanning, 30e3, 50e3, FS)

    # Calculate raw phase lag between opposing transducers
    raw_lag_ns = get_lag(s1f, s2f)
    raw_lag_ew = get_lag(s3f, s4f)

    # --- CALIBRATION LOGIC ---
    # Ingest baseline calibration samples dynamically
    with asym_lock:
        if asym_stage in (0, 1):
            asym_buffer.append(raw_lag_ns)
        elif asym_stage in (2, 3):
            asym_buffer.append(raw_lag_ew)

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
                
                # Drop out-of-sync legacy values
                lag_buffer_ns.clear(); wind_buffer_ns.clear()
                lag_buffer_ew.clear(); wind_buffer_ew.clear()
                calib_txt.set_text(f'CAL DONE  ns={offset_ns_zero:+.2f}  ew={offset_ew_zero:+.2f}')
                calib_txt.set_color(GRN)
                print(f"\n [CAL          ] Done — offset N/S: {offset_ns_zero:+.5f} samples | E/W: {offset_ew_zero:+.5f} samples\n\n ------>\n")

    # --- WIND VECTOR CALCULATIONS ---
    # Primary structural compensation calculations (Remove Zero Offset)
    corrected_lag_ns = raw_lag_ns - offset_ns_zero
    corrected_lag_ew = raw_lag_ew - offset_ew_zero

    # Apply Asymmetry Multipliers based on Wind Direction
    corrected_lag_ns *= POS_SCALE_NS if corrected_lag_ns >= 0 else NEG_SCALE_NS
    corrected_lag_ew *= POS_SCALE_EW if corrected_lag_ew >= 0 else NEG_SCALE_EW

    # Scale phase lag to physical wind speed metrics
    tmp_wind_ns = -(corrected_lag_ns / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)
    tmp_wind_ew =  (corrected_lag_ew / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)
    tmp_speed   = np.sqrt(tmp_wind_ns**2 + tmp_wind_ew**2)

    # Check if signal is physically stable enough for auto-tracking
    stable_ns = len(lag_buffer_ns) > 5 and np.std(lag_buffer_ns) < LAG_STABLE_TH
    stable_ew = len(lag_buffer_ew) > 5 and np.std(lag_buffer_ew) < LAG_STABLE_TH

    # Dynamic baseline drift tracker loop execution
    if tmp_speed < ZERO_WIND_TH and stable_ns and stable_ew and not calib_active_zero:
        offset_ns_zero = (1 - OFFSET_ALPHA) * offset_ns_zero + OFFSET_ALPHA * raw_lag_ns
        offset_ew_zero = (1 - OFFSET_ALPHA) * offset_ew_zero + OFFSET_ALPHA * raw_lag_ew

    # Re-verify and re-process metrics against newly tracked baseline vectors
    corrected_lag_ns = raw_lag_ns - offset_ns_zero
    corrected_lag_ew = raw_lag_ew - offset_ew_zero
    corrected_lag_ns *= POS_SCALE_NS if corrected_lag_ns >= 0 else NEG_SCALE_NS
    corrected_lag_ew *= POS_SCALE_EW if corrected_lag_ew >= 0 else NEG_SCALE_EW

    # Final Wind Calculation
    wind_ns = -(corrected_lag_ns / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)
    wind_ew =  (corrected_lag_ew / FS) * (SOUND_SPEED**2 / SENSOR_DISTANCE)

    # Filter out sporadic spikes
    lag_sn  = smooth_median(lag_buffer_ns,  corrected_lag_ns)
    wind_sn = smooth_median(wind_buffer_ns, wind_ns)
    lag_se  = smooth_median(lag_buffer_ew,  corrected_lag_ew)
    wind_se = smooth_median(wind_buffer_ew, wind_ew)

    # Apply low-pass vector smoothing
    smooth_ns  = (1 - DIR_VECTOR_ALPHA) * smooth_ns + DIR_VECTOR_ALPHA * wind_sn
    smooth_ew  = (1 - DIR_VECTOR_ALPHA) * smooth_ew + DIR_VECTOR_ALPHA * wind_se
    wind_speed = np.sqrt(smooth_ns**2 + smooth_ew**2)

    # Calculate Compass Direction
    if wind_speed > MIN_DIR_SPEED:
        direction_rad      = np.arctan2(smooth_ew, smooth_ns)
        direction_deg      = (np.degrees(direction_rad) + 360) % 360
        last_direction_deg = direction_deg
    else:
        direction_deg = last_direction_deg

    # --- UPDATE USER INTERFACE & TERMINAL ---
    
    # Terminal Telemetry Output
    cal_marker = '[CAL          ]' if calib_active_zero else f'[off {offset_ns_zero:+.1f}/{offset_ew_zero:+.1f}]'
    sys.stdout.write(
        f"\r {cal_marker} Wind: {wind_speed:3.5f} m/s | Dir: {direction_deg:6.2f}° "
        f"{dir_label(direction_deg):<3s} | Lag N/S: {lag_sn:+3.3f} | Lag E/W: {lag_se:+3.3f}   "
    )
    sys.stdout.flush()

    # Re-map visual canvas vector directions
    ax_px, ay_px   = rotate_point(0,  0.65, direction_deg)
    tail_x, tail_y = rotate_point(0, -0.30, direction_deg)
    arrow_line.set_data([0, ax_px], [0, ay_px])
    arrow_head.set_data([ax_px], [ay_px])
    arrow_head.set_marker((3, 0, direction_deg))
    arrow_tail.set_data([0, tail_x], [0, tail_y])

    # Dynamic speed indicator circle
    r = min(wind_speed / MAX_SPEED, 1.0) * 0.85
    speed_circle.set_radius(max(r, 0.01))

    # Update Text Elements
    spd_txt.set_text(f'{wind_speed:.1f} m/s')
    dir_txt.set_text(f'{direction_deg:.0f}° {dir_label(direction_deg)}')
    lag_txt.set_text(f'N/S {lag_sn:+.2f}  E/W {lag_se:+.2f}')
    offset_txt.set_text(f'off ns:{offset_ns_zero:+.2f} ew:{offset_ew_zero:+.2f}  Gns(+){POS_SCALE_NS:.2f} Gns(-){NEG_SCALE_NS:.2f}')

    # Update Multi-channel Signal Traces (Raw and Filtered Waveforms)
    raw_lim  = max(np.max(np.abs(s1)), np.max(np.abs(s2)), np.max(np.abs(s3)), np.max(np.abs(s4))) * 1.2 + 1
    filt_lim = max(np.max(np.abs(s1f)), np.max(np.abs(s2f)), np.max(np.abs(s3f)), np.max(np.abs(s4f))) * 1.2 + 1

    plot_configs = [
        (0, s1, raw_lim),   (1, s1f, filt_lim), (2, s2, raw_lim),   (3, s2f, filt_lim),
        (4, s3, raw_lim),   (5, s3f, filt_lim), (6, s4, raw_lim),   (7, s4f, filt_lim)
    ]
    for idx, data, lim in plot_configs:
        sig_lines[idx].set_ydata(data)
        sig_axes[idx].set_ylim(-lim, lim)

    return sig_lines + artists

#   ┌── [INFO] ──────────────────────────────────────────────────────────────────┐
#   │ 8. EXECUTION CONTEXT ENTRYPOINT                                            │
#   └────────────────────────────────────────────────────────────────────────────┘

if __name__ == '__main__':
    print("Ultrasonic Wind Anemometer Telemetry Processor")
    print("  C  →  Zero-wind offset calibration  (Keep system stable)")
    print("  V  →  Asymmetry calibration         (Follow instructions closely)\n")
    print(f"C-calibration samples {CALIB_DURATION:.0f}s of raw packet phase lags to establish an offset.")
    print("V-calibration steps through geometric directions to balance relative transducer sensitivity.")
    print("Recommended sequencing: Perform geometric asymmetry alignment [V] first, followed by baseline zeroing [C].\n")

    # Start looking for serial data in the background
    reader = threading.Thread(target=serial_reader, args=(PORT, BAUD), daemon=True)
    reader.start()

    try:
        # Start the GUI Animation loop
        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up threads when the window is closed
        stop_event.set()
        plt.close('all')
        print("\n\nExecution terminated successfully.\n\n")