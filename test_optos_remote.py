import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"

with serial.Serial(PORT, 115200, timeout=0.25) as ser:
    time.sleep(7.0)
    ser.reset_input_buffer()

    for repeat in range(1, 4):
        for side in ("L", "R"):
            command = f"q {side} 60"
            print(f"SEND rep={repeat} {command}", flush=True)
            ser.write((command + "\n").encode())
            ser.flush()
            deadline = time.time() + 6.0
            found = False
            while time.time() < deadline:
                line = ser.readline().decode(errors="replace").strip()
                if line.startswith("q "):
                    print(line, flush=True)
                    found = True
                    break
            if not found:
                print(f"q FAIL side={side} no_response", flush=True)
            ser.write(b"STOP\n")
            ser.flush()
            time.sleep(1.0)

    ser.write(b"STOP\nINHABILITAR\n")
    ser.flush()
