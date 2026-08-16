import re
import statistics
import sys
import time

import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
DELAYS_US = (5000, 5500, 6000, 6500, 7000, 7500)
REPEATS = 3
HALL_PPR = 45.0
OPTO_PPR = 60.0
Q_RE = re.compile(r"q OK side=([LR]) pwm=(\d+) L=(\d+) R=(\d+) OL=(\d+) OR=(\d+)")


def command_response(ser, command, prefix, timeout):
    ser.reset_input_buffer()
    ser.write((command + "\n").encode())
    ser.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line
    raise RuntimeError(f"Sin respuesta para {command!r}")


results = []
with serial.Serial(PORT, 115200, timeout=0.10) as ser:
    time.sleep(7.0)
    for delay_us in DELAYS_US:
        reply = command_response(ser, f"j {delay_us}", "j ", 2.0)
        print(reply, flush=True)
        for repeat in range(1, REPEATS + 1):
            for side in ("L", "R"):
                line = command_response(ser, f"q {side} 60", "q ", 3.0)
                match = Q_RE.fullmatch(line)
                if not match:
                    raise RuntimeError(f"Respuesta inesperada: {line}")
                _, pwm, hall_l, hall_r, opto_l, opto_r = match.groups()
                hall = int(hall_l if side == "L" else hall_r)
                opto = int(opto_l if side == "L" else opto_r)
                expected = hall * OPTO_PPR / HALL_PPR
                error_pct = 100.0 * abs(opto - expected) / expected if expected else 999.0
                results.append((delay_us, side, hall, opto, error_pct))
                print(
                    f"delay={delay_us:5d} rep={repeat} side={side} pwm={pwm} "
                    f"hall={hall:3d} expected={expected:6.1f} opto={opto:3d} error={error_pct:6.1f}%",
                    flush=True,
                )
                time.sleep(0.75)

    ser.write(b"STOP\nINHABILITAR\n")
    ser.flush()

print("RESUMEN")
for delay_us in DELAYS_US:
    rows = [row for row in results if row[0] == delay_us]
    errors = [row[4] for row in rows]
    ratios = [row[3] / (row[2] * OPTO_PPR / HALL_PPR) for row in rows if row[2]]
    print(
        f"delay={delay_us:5d} error_medio={statistics.mean(errors):6.1f}% "
        f"error_max={max(errors):6.1f}% opto_esperado_ratio={statistics.mean(ratios):.3f}"
    )
