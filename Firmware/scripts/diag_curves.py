"""Diagnose scale/phase curves on COM28 OpenFFBoard."""
import serial, time, re, sys

PORT = "COM28"
BAUD = 115200

def send_cmd(ser, msg, timeout=2.0):
    ser.reset_input_buffer()
    ser.write((msg + "\n").encode('ascii'))
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
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
except Exception as e:
    print(f"ERROR opening {PORT}: {e}")
    sys.exit(1)
ser.dtr = True
time.sleep(0.5)
ser.reset_input_buffer()

# Check connection
r = send_cmd(ser, "[tmc.1.state?]", 1)
print("STATE:", r)

# Scale curve
r = send_cmd(ser, "[tmc.1.scaleCurve?]", 2)
s = ""
for l in r:
    if "scaleCurve" in l:
        s = pb(l)
        break
print("SCALE RAW:", s[:300])
sd = []
nz = 0
for p in s.split(","):
    if ":" in p:
        rpm, v = p.split(":", 1)
        rpm = int(rpm)
        v = float(v) / 1000.0
        sd.append((rpm, v))
        if v > 0:
            nz += 1
            print(f"  RPM {rpm:4d}: {v:.3f}")
print("Non-zero: %d/%d" % (nz, len(sd)))

# Phase curve
r = send_cmd(ser, "[tmc.1.phaseAdvCurve?]", 2)
s = ""
for l in r:
    if "phaseAdvCurve" in l:
        s = pb(l)
        break
print("PHASE RAW:", s[:300])
pd = []
nz = 0
for p in s.split(","):
    if ":" in p:
        rpm, v = p.split(":", 1)
        rpm = int(rpm)
        v = float(v) / 100.0
        pd.append((rpm, v))
        if v != 0:
            nz += 1
            print(f"  RPM {rpm:4d}: {v:.2f} deg")
print("Non-zero: %d/%d" % (nz, len(pd)))

# Cogging state
for cmd in ["cogging", "coggingScale", "coggingCalibCount"]:
    r = send_cmd(ser, "[tmc.1.%s?]" % cmd, 1)
    v = pb(r[0]) if r else "NONE"
    print("%s: %s" % (cmd, v))

# Profiles
for i in range(3):
    r = send_cmd(ser, "[tmc.1.coggingCalibRPM!?%d]" % i, 1)
    rpm = pb(r[0]) if r else "NONE"
    r = send_cmd(ser, "[tmc.1.coggingCalibIters!?%d]" % i, 1)
    itr = pb(r[0]) if r else "NONE"
    print("Profile %d: RPM=%s Iters=%s" % (i, rpm, itr))

ser.close()
print("\nDone.")
