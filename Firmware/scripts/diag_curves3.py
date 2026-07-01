"""Diagnose scale/phase curves on COM28."""
import serial, time, re, sys

PORT = "COM28"
BAUD = 115200

def send_cmd(ser, msg, timeout=2.0):
    try:
        ser.reset_input_buffer()
        ser.write(msg.encode('ascii'))
        ser.flush()
    except Exception as e:
        print("  WriteErr:", e)
        time.sleep(0.5)
        return []
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

def pb(text):
    m = re.search(r'\[.*\|(.+)\]', text)
    return m.group(1) if m else text

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=1.0)
except Exception as e:
    print("ERROR:", e)
    sys.exit(1)

print("Waiting for firmware...")
time.sleep(3.0)
ser.reset_input_buffer()

# Read boot messages
print("Boot messages:")
for _ in range(10):
    try:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line:
            print(" ", line)
    except:
        pass

# Try the correct format: tmc.1.cmd?;
print("\n--- Sending tmc.1.state?; ---")
r = send_cmd(ser, "tmc.1.state?;\n", 3)
print("REPLY:", r)

# Scale
print("\n--- Scale curve ---")
r = send_cmd(ser, "tmc.1.scaleCurve?;\n", 3)
print("RAW:", r[:3] if r else "EMPTY")

scale_raw = None
for l in r:
    if "scaleCurve" in l or "Scale" in l:
        scale_raw = pb(l) if '[' in l else l
        break
if scale_raw:
    print("DATA:", scale_raw[:200])
    nz = 0
    for p in scale_raw.split(","):
        if ":" in p:
            rpm, v = p.split(":", 1)
            rpm = int(rpm)
            v = float(v) / 1000.0
            if v > 0:
                nz += 1
                print("  RPM %3d: %.3f" % (rpm, v))
    print("Non-zero: %d" % nz)
else:
    print("No scale data found in reply")

# Phase
print("\n--- Phase curve ---")
r = send_cmd(ser, "tmc.1.phaseAdvCurve?;\n", 3)
phase_raw = None
for l in r:
    if "phaseAdvCurve" in l or "Phase" in l:
        phase_raw = pb(l) if '[' in l else l
        break
if phase_raw:
    print("DATA:", phase_raw[:200])
    nz = 0
    for p in phase_raw.split(","):
        if ":" in p:
            rpm, v = p.split(":", 1)
            rpm = int(rpm)
            v = float(v) / 100.0
            if v != 0:
                nz += 1
                print("  RPM %3d: %.2f deg" % (rpm, v))
    print("Non-zero: %d" % nz)

# Profiles
print("\n--- Profiles ---")
for i in range(3):
    r = send_cmd(ser, "tmc.1.coggingCalibRPM?%d;\n" % i, 2)
    print("Profile %d RPM: %s" % (i, r[:2]))
    r = send_cmd(ser, "tmc.1.coggingCalibIters?%d;\n" % i, 2)
    print("Profile %d Iters: %s" % (i, r[:2]))

ser.close()
print("Done.")
