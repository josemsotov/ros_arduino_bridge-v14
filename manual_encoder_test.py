import time
import serial


PORT = "COM4"
DURATION_S = 50.0


def request_pair(ser, command):
    ser.write((command + "\n").encode())
    deadline = time.monotonic() + 0.35
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(command + " "):
            values = line.split()
            if len(values) >= 3:
                return int(values[-2]), int(values[-1])
    return None


with serial.Serial(PORT, 115200, timeout=0.05) as ser:
    time.sleep(7.0)
    ser.reset_input_buffer()
    start = time.monotonic()
    first = {"e": None, "o": None}
    last = {"e": None, "o": None}
    while time.monotonic() - start < DURATION_S:
        for command in ("e", "o"):
            value = request_pair(ser, command)
            if value is None:
                continue
            if first[command] is None:
                first[command] = value
            if value != last[command]:
                print(f"t={time.monotonic()-start:5.1f}s {command} L={value[0]} R={value[1]}", flush=True)
                last[command] = value
        time.sleep(0.10)

    print("RESULTADO")
    for command, label in (("e", "HALL"), ("o", "OPTO")):
        if first[command] is None or last[command] is None:
            print(f"{label}: sin respuesta")
            continue
        delta_l = last[command][0] - first[command][0]
        delta_r = last[command][1] - first[command][1]
        print(f"{label} inicial={first[command]} final={last[command]} delta=({delta_l},{delta_r})")
