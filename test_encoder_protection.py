import sys
import time

import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0


def wait_prefix(ser, prefix, timeout=3.0):
    deadline = time.monotonic() + timeout
    seen = []
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        seen.append(line)
        if line.startswith(prefix):
            return line, seen
    return "", seen


with serial.Serial(PORT, 115200, timeout=0.10) as ser:
    time.sleep(7.0)
    ser.reset_input_buffer()
    for command, prefix in (("n protect reset", "n PROTECT RESET"),
                            ("n protect on", "n PROTECT ON"),
                            ("p 20", "p OK")):
        ser.write((command + "\n").encode())
        ser.flush()
        reply, _ = wait_prefix(ser, prefix)
        print(reply or f"TIMEOUT {command}", flush=True)

    started = time.monotonic()
    trip_line = ""
    while time.monotonic() - started < DURATION:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"EVENT {line}", flush=True)
            if line.startswith("n TRIP"):
                trip_line = line
                break

    ser.write(b"p\nSTOP\nINHABILITAR\n")
    ser.flush()
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"n\n")
    ser.flush()
    status, _ = wait_prefix(ser, "n STATUS")
    print(status or "TIMEOUT status", flush=True)
    print(f"RESULT trip={trip_line or 'NONE'} elapsed={time.monotonic()-started:.2f}s", flush=True)
