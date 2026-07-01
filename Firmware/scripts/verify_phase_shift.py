"""Connect to OpenFFBoard on COM28, read scale/phase curves, run calibration, verify."""
import serial
import time
import re
import sys

PORT = "COM28"
BAUD = 115200
TIMEOUT = 1.0

def send_cmd(ser, cls, instance, cmd, val=None, adr=None, timeout=2.0):
    """Send a command and return the reply list."""
    if val is not None and adr is not None:
        # setat: [cls.instance.cmd!val?adr]
        msg = f"[{cls}.{instance}.{cmd}!{val}?{adr}]\n"
    elif val is not None:
        # set: [cls.instance.cmd=val]
        msg = f"[{cls}.{instance}.{cmd}={val}]\n"
    else:
        # get: [cls.instance.cmd?]
        msg = f"[{cls}.{instance}.{cmd}?]\n"
    
    ser.reset_input_buffer()
    ser.write(msg.encode('ascii'))
    
    replies = []
    start = time.time()
    while time.time() - start < timeout:
        line = ""
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
        except:
            continue
        if not line:
            time.sleep(0.01)
            continue
        replies.append(line)
        # Commands may produce multiple replies; stop if we see a non-broadcast reply
        # or after collecting enough lines
        if len(replies) > 20:
            break
    return replies

def get_float(ser, cls, instance, cmd, timeout=2.0):
    """Get a float value."""
    replies = send_cmd(ser, cls, instance, cmd, timeout=timeout)
    for r in replies:
        # Try to parse as simple value
        try:
            return float(r.strip())
        except:
            pass
        # Try to parse bracketed reply
        m = re.search(r'\[.*\|(.+)\]', r)
        if m:
            parts = m.group(1).split(':')
            try:
                return float(parts[0])
            except:
                pass
    return None

def get_string(ser, cls, instance, cmd, timeout=2.0):
    """Get a string value from bracketed reply."""
    replies = send_cmd(ser, cls, instance, cmd, timeout=timeout)
    for r in replies:
        m = re.search(r'\[.*\|(.+)\]', r)
        if m:
            return m.group(1)
        if r and not r.startswith('['):
            return r.strip()
    return None

def parse_curve(data_str):
    """Parse 'rpm:val,rpm:val,...' into list of (rpm, val) tuples."""
    result = []
    if not data_str:
        return result
    for pair in data_str.split(','):
        if ':' in pair:
            r, v = pair.split(':', 1)
            result.append((int(r), int(v)))
    return result

def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.dtr = True  # Required by OpenFFBoard firmware
    time.sleep(0.5)
    
    # Flush any pending data
    ser.reset_input_buffer()
    
    cls = "tmc"
    inst = 1
    
    # ===== 1. Check connection =====
    print("\n=== 1. Checking connection ===")
    state = get_float(ser, cls, inst, "state")
    print(f"  TMC state: {state}")
    
    tmctype = get_string(ser, cls, inst, "tmctype")
    print(f"  TMC type: {tmctype}")
    
    # ===== 2. Read current curves BEFORE calibration =====
    print("\n=== 2. Current curves BEFORE calibration ===")
    
    scale_str = get_string(ser, cls, inst, "scaleCurve")
    print(f"  scaleCurve raw: {scale_str[:200] if scale_str else 'None'}...")
    scale_curve = parse_curve(scale_str) if scale_str else []
    
    phase_str = get_string(ser, cls, inst, "phaseAdvCurve")
    print(f"  phaseAdvCurve raw: {phase_str[:200] if phase_str else 'None'}...")
    phase_curve = parse_curve(phase_str) if phase_str else []
    
    # ===== 3. Check calibration profiles =====
    print("\n=== 3. Calibration profiles ===")
    count = get_float(ser, cls, inst, "coggingCalibCount")
    print(f"  Profile count: {count}")
    
    for i in range(int(count) if count else 0):
        rpm = get_float(ser, cls, inst, "coggingCalibRPM", adr=i)
        iters = get_float(ser, cls, inst, "coggingCalibIters", adr=i)
        print(f"  Profile {i}: RPM={rpm}, Iters={iters}")
    
    # ===== 4. Print current curve values =====
    print("\n=== 4. Current scale curve values ===")
    if scale_curve:
        for rpm, val in scale_curve:
            print(f"  RPM {rpm:3d}: scale={val/1000.0:.3f}")
    else:
        print("  (empty or invalid)")
    
    print("\n=== 5. Current phase-advance curve values ===")
    if phase_curve:
        for rpm, val in phase_curve:
            print(f"  RPM {rpm:3d}: phase={val/100.0:.2f} deg")
    else:
        print("  (empty or invalid)")
    
    # ===== 6. Check cogging state =====
    print("\n=== 6. Cogging state ===")
    cogging = get_float(ser, cls, inst, "cogging")
    print(f"  Cogging enabled: {cogging}")
    cscale = get_float(ser, cls, inst, "coggingScale")
    print(f"  Cogging scale (*10000): {cscale}")
    
    # ===== 7. Verify our code assumptions =====
    print("\n=== 7. Verification ===")
    if scale_curve:
        # Check index 0 (plateau)
        idx0_scale = None
        idx1_scale = None
        for rpm, val in scale_curve:
            if rpm == 0:
                idx0_scale = val / 1000.0
            if rpm == 3:
                idx1_scale = val / 1000.0
        print(f"  Index 0 (RPM=0) scale: {idx0_scale} (should be 1.0)")
        print(f"  Index 1 (RPM=3) scale: {idx1_scale} (should be 1.0)")
    
    if phase_curve:
        idx0_phase = None
        for rpm, val in phase_curve:
            if rpm == 0:
                idx0_phase = val / 100.0
        print(f"  Index 0 (RPM=0) phase: {idx0_phase} (should be 0.0)")
    
    # ===== Show how many non-zero values exist =====
    print("\n=== 8. Non-zero values ===")
    if scale_curve:
        nonzero = [(r, v/1000.0) for r, v in scale_curve if v != 0]
        print(f"  Scale: {len(nonzero)} non-zero points out of {len(scale_curve)}")
        for r, v in nonzero:
            print(f"    RPM {r:3d}: {v:.3f}")
    if phase_curve:
        nonzero = [(r, v/100.0) for r, v in phase_curve if v != 0]
        print(f"  Phase: {len(nonzero)} non-zero points out of {len(phase_curve)}")
        for r, v in nonzero:
            print(f"    RPM {r:3d}: {v:.2f} deg")
    
    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
