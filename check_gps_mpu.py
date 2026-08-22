import sys
import time

import serial

sys.stdout.reconfigure(errors="backslashreplace")


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 18.0


with serial.Serial(PORT, 115200, timeout=0.20) as ser:
    started = time.monotonic()
    while time.monotonic() - started < DURATION:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(line, flush=True)
