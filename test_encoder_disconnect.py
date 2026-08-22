import sys
import time

import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"


def response(ser, command, prefix, timeout=4.0):
    ser.reset_input_buffer()
    ser.write((command + "\n").encode())
    ser.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line
    return f"TIMEOUT command={command}"


with serial.Serial(PORT, 115200, timeout=0.10) as ser:
    time.sleep(7.0)
    print(response(ser, "j reset", "j RESET"), flush=True)
    print(response(ser, "q L 20", "q "), flush=True)
    print(response(ser, "n check", "n CHECK"), flush=True)
    print(response(ser, "j stat", "j STAT"), flush=True)
    ser.write(b"STOP\nINHABILITAR\n")
    ser.flush()
