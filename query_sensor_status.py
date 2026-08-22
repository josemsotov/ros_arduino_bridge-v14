import sys
import time

import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
sys.stdout.reconfigure(errors="backslashreplace")


with serial.Serial(PORT, 115200, timeout=0.15) as ser:
    time.sleep(8.0)
    ser.reset_input_buffer()
    for command in ("hc stat", "z", "e", "o", "n"):
        print(f"> {command}", flush=True)
        ser.write((command + "\n").encode())
        ser.flush()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(line, flush=True)
                if line.startswith(("hc ", "z ", "e ", "o ", "n ")):
                    break
