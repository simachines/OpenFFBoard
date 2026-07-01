"""Diagnose scale/phase curves on COM28 OpenFFBoard."""
import serial, time, re, sys

PORT = "COM28"
BAUD = 115200

def send_cmd(ser, msg, timeout=2.0):
    try:
        ser.reset_input_buffer()
        ser.write((msg + "\n").encode('ascii'))
    except Exception as e:
        print("  Write error:", e)
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
    ser = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=2.0)
except Exception as e:
    print("ERROR opening port:", e)
    sys.exit(1)

# Wait for firmware to boot
print("Waiting for firmware...")
time.sleep(3.0)
ser.reset_input_buffer()

# Read any boot messages
for _ in range(5):
    try:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line:
            print("BOOT:", line)
    except:
        pass

print("\n--- Sending command ---")
r = send_cmd(ser, "[tmc.1.state?]", 3)
print("STATE REPLY:", r)

if not r:
    print("NO REPLY - trying alternate format...")
    r = send_cmd(ser, "tmc.1.state?;", 3)
    print("STATE REPLY (fmt2):", r)

# Scale curve
print("\n--- Scale curve ---")
r = send_cmd(ser, "[tmc.1.scaleCurve?]", 3)
if not r:
    r = send_cmd(ser, "tmc.1.scaleCurve?;", 3)
print("RAW:", r[:3] if r else "EMPTY")

for l in r:
    if "scaleCurve" in l:
        s = pb(l)
        print("PARSED:", s[:200])
        nz = 0
        for p in s.split(","):
            if ":" in p:
                rpm, v = p.split(":", 1)
                rpm = int(rpm)
                v = float(v) / 1000.0
                if v != 0:
                    nz += 1
                    print("  RPM %3d: %.3f" % (rpm, v))
        print("Non-zero: %d" % nz)
        break

ser.close()
print("Done.")
