import re
import sys
import time

import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
REPEATS = 5
RESULT_RE = re.compile(
    r"u (OK|TIMEOUT) target=(\d+) OL=(\d+) OR=(\d+) HL=(\d+) HR=(\d+) "
    r"errL=(-?\d+) errR=(-?\d+) revL=([\d.]+) revR=([\d.]+) ms=(\d+)"
)


with serial.Serial(PORT, 115200, timeout=0.10) as ser:
    time.sleep(7.0)
    rows = []
    for repeat in range(1, REPEATS + 1):
        ser.reset_input_buffer()
        ser.write(b"u 60\n")
        ser.flush()
        deadline = time.monotonic() + 10.0
        line = ""
        while time.monotonic() < deadline:
            candidate = ser.readline().decode(errors="replace").strip()
            if candidate.startswith("u "):
                line = candidate
                break
        match = RESULT_RE.fullmatch(line)
        if not match:
            print(f"REP={repeat} FAIL response={line!r}", flush=True)
            continue
        status, target, ol, orange, hl, hr, err_l, err_r, rev_l, rev_r, elapsed = match.groups()
        row = tuple(map(int, (target, ol, orange, hl, hr, err_l, err_r, elapsed)))
        rows.append(row)
        print(
            f"REP={repeat} status={status} OL={ol} OR={orange} HL={hl} HR={hr} "
            f"opto_rev=({rev_l},{rev_r}) ms={elapsed}",
            flush=True,
        )
        time.sleep(1.5)
    ser.write(b"STOP\nINHABILITAR\n")
    ser.flush()

if rows:
    expected_hall = 45.0
    left_errors = [100.0 * (row[3] - expected_hall) / expected_hall for row in rows]
    right_errors = [100.0 * (row[4] - expected_hall) / expected_hall for row in rows]
    print(
        f"SUMMARY runs={len(rows)} hall_expected={expected_hall:.0f} "
        f"HL_range={min(row[3] for row in rows)}-{max(row[3] for row in rows)} "
        f"HR_range={min(row[4] for row in rows)}-{max(row[4] for row in rows)} "
        f"HL_error_mean={sum(left_errors)/len(left_errors):+.1f}% "
        f"HR_error_mean={sum(right_errors)/len(right_errors):+.1f}%"
    )
