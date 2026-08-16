import re
import statistics
import sys
import time

import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
PWM_LEVELS = (20, 25, 30, 35, 40)
REPEATS = 3
HALL_PPR = 45.0
OPTO_PPR = 60.0
Q_RE = re.compile(r"q OK side=([LR]) pwm=(\d+) L=(\d+) R=(\d+) OL=(\d+) OR=(\d+)")
STAT_RE = re.compile(
    r"j STAT Lus=(\d+) Rus=(\d+) Lraw=(\d+) Lacc=(\d+) Lrej=(\d+) "
    r"Rraw=(\d+) Racc=(\d+) Rrej=(\d+)"
)


def response(ser, command, prefix, timeout=3.0):
    ser.reset_input_buffer()
    ser.write((command + "\n").encode())
    ser.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line
    raise RuntimeError(f"Sin respuesta para {command!r}")


def pair(ser, command):
    line = response(ser, command, command + " ")
    values = line.split()
    return int(values[-2]), int(values[-1])


rows = []
with serial.Serial(PORT, 115200, timeout=0.10) as ser:
    time.sleep(7.0)
    opto_start = pair(ser, "o")
    hall_start = pair(ser, "e")
    time.sleep(5.0)
    opto_end = pair(ser, "o")
    hall_end = pair(ser, "e")
    print(
        f"IDLE_5S hall_delta=({hall_end[0]-hall_start[0]},{hall_end[1]-hall_start[1]}) "
        f"opto_delta=({opto_end[0]-opto_start[0]},{opto_end[1]-opto_start[1]})",
        flush=True,
    )

    for pwm in PWM_LEVELS:
        for repeat in range(1, REPEATS + 1):
            for side in ("L", "R"):
                response(ser, "j reset", "j RESET")
                line = response(ser, f"q {side} {pwm}", "q ")
                match = Q_RE.fullmatch(line)
                if not match:
                    raise RuntimeError(f"Respuesta inesperada: {line}")
                _, actual_pwm, hl, hr, ol, orange = match.groups()
                hall = int(hl if side == "L" else hr)
                opto = int(ol if side == "L" else orange)
                expected = hall * OPTO_PPR / HALL_PPR
                error = 100.0 * (opto - expected) / expected if expected else None
                rows.append((pwm, repeat, side, hall, opto, expected, error))
                stat_line = response(ser, "j stat", "j STAT")
                stat_match = STAT_RE.fullmatch(stat_line)
                if not stat_match:
                    raise RuntimeError(f"Estadistica inesperada: {stat_line}")
                lfilter, rfilter, lraw, lacc, lrej, rraw, racc, rrej = map(int, stat_match.groups())
                raw = lraw if side == "L" else rraw
                accepted = lacc if side == "L" else racc
                rejected = lrej if side == "L" else rrej
                health_line = response(ser, "n check", "n CHECK")
                error_text = "n/a" if error is None else f"{error:+.1f}%"
                print(
                    f"PWM={pwm:2d} REP={repeat} SIDE={side} HALL={hall:3d} "
                    f"OPTO={opto:3d} EXPECTED={expected:6.1f} ERROR={error_text} "
                    f"RAW={raw} ACCEPTED={accepted} REJECTED={rejected} "
                    f"FILTER_US={lfilter if side == 'L' else rfilter} HEALTH={health_line}",
                    flush=True,
                )
                time.sleep(0.50)

    ser.write(b"STOP\nINHABILITAR\n")
    ser.flush()

print("SUMMARY")
for pwm in PWM_LEVELS:
    for side in ("L", "R"):
        selected = [row for row in rows if row[0] == pwm and row[2] == side]
        halls = [row[3] for row in selected]
        optos = [row[4] for row in selected]
        errors = [abs(row[6]) for row in selected if row[6] is not None]
        moving = sum(value > 0 for value in halls)
        hall_cv = 0.0
        if statistics.mean(halls) > 0:
            hall_cv = 100.0 * statistics.pstdev(halls) / statistics.mean(halls)
        error_text = "n/a" if not errors else f"{statistics.mean(errors):.2f}%"
        print(
            f"PWM={pwm:2d} SIDE={side} moving={moving}/{REPEATS} "
            f"hall_mean={statistics.mean(halls):6.2f} hall_cv={hall_cv:5.2f}% "
            f"opto_mean={statistics.mean(optos):6.2f} abs_error_mean={error_text}"
        )
