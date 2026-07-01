"""Run anti-cogging calibration on COM28 and verify curves."""
import serial, time, re, sys

PORT = "COM28"
BAUD = 115200

def connect():
    ser = serial.Serial(PORT, BAUD, timeout=0.3, write_timeout=2.0)
    time.sleep(2)
    ser.reset_input_buffer()
    # Drain boot messages
    for _ in range(20):
        try:
            l = ser.readline().decode('ascii', errors='replace').strip()
        except:
            pass
    return ser

def cmd(ser, msg, timeout=2.0):
    """Send command and return replies."""
    ser.reset_input_buffer()
    ser.write((msg + "\n").encode())
    ser.flush()
    replies = []
    start = time.time()
    while time.time() - start < timeout:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line:
                replies.append(line)
        except:
            continue
    return replies

def wait_calibration(ser, max_wait=300):
    """Wait for calibration to complete, printing progress. Returns True on success."""
    start = time.time()
    while time.time() - start < max_wait:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line:
                time.sleep(0.1)
                continue
            print("  " + line)
            if "Cogging detection finished" in line:
                return True
            if "Calibration aborted" in line or "Abort" in line:
                print("CALIBRATION ABORTED!")
                return False
        except:
            time.sleep(0.1)
            continue
    print("TIMEOUT waiting for calibration")
    return False

def read_curve(ser, cmd_name):
    r = cmd(ser, "tmc.0.%s?;" % cmd_name, 3)
    for l in r:
        if cmd_name in l:
            m = re.search(r'\[.*\|(.+)\]', l)
            if m:
                data = m.group(1)
                pairs = []
                for p in data.split(","):
                    if ":" in p:
                        rpm, v = p.split(":", 1)
                        pairs.append((int(rpm), int(v)))
                return pairs
    return []

print("Connecting...")
ser = connect()

# Check state
r = cmd(ser, "tmc.0.state?;", 2)
print("State:", r)

# Verify calib profiles
r = cmd(ser, "tmc.0.coggingCalibCount?;", 2)
print("CalibCount:", r)

# Read pre-calibration curves
print("\n=== PRE-CALIBRATION CURVES ===")
scale_pre = read_curve(ser, "scaleCurve")
phase_pre = read_curve(ser, "phaseAdvCurve")
print("Scale non-zero:")
for rpm, v in scale_pre:
    if v != 0:
        print("  RPM %3d: %.3f" % (rpm, v/1000.0))
print("Phase non-zero:")
for rpm, v in phase_pre:
    if v != 0:
        print("  RPM %3d: %.2f deg" % (rpm, v/100.0))

# Start calibration
print("\n=== STARTING CALIBRATION ===")
r = cmd(ser, "tmc.0.calibrateCogging?;", 3)
print("Start reply:", r)

# Wait for completion
print("\n=== CALIBRATION PROGRESS ===")
success = wait_calibration(ser, max_wait=300)

# Wait a bit for firmware to stabilize
time.sleep(3)

# Read post-calibration curves
print("\n=== POST-CALIBRATION CURVES ===")
scale_post = read_curve(ser, "scaleCurve")
phase_post = read_curve(ser, "phaseAdvCurve")

print("\nALL 24 SCALE VALUES:")
all_ok = True
for rpm, v in scale_post:
    marker = ""
    if rpm == 0 and v != 1000:
        marker = " *** BAD: plateau should be 1.0 (1000)"
        all_ok = False
    elif rpm > 0 and v == 0:
        marker = " (unset)"
    elif rpm > 0 and v != 0:
        pass
    print("  RPM %4d: %.3f%s" % (rpm, v/1000.0, marker))

print("\nALL 24 PHASE VALUES:")
for rpm, v in phase_post:
    phase_val = v / 100.0
    marker = ""
    if rpm == 0 and phase_val != 0:
        marker = " *** BAD: plateau should be 0"
        all_ok = False
    print("  RPM %4d: %.2f deg%s" % (rpm, phase_val, marker))

# Verify non-zero values exist beyond plateau
scale_nz = [(r,v) for r,v in scale_post if v != 0 and r > 0]
phase_nz = [(r,v) for r,v in phase_post if v != 0 and r > 0]
print("\n=== VERIFICATION ===")
print("Scale non-zero entries (excl plateau): %d" % len(scale_nz))
print("Phase non-zero entries (excl plateau): %d" % len(phase_nz))

if len(scale_nz) < 2:
    print("FAIL: Expected at least 2 non-zero scale points after calibration")
    all_ok = False
if len(phase_nz) < 1:
    print("FAIL: Expected at least 1 non-zero phase point after calibration")
    all_ok = False

if all_ok:
    print("PASS: Calibration curves look correct!")
else:
    print("FAIL: Issues found - see markers above")

ser.close()
