import re
import statistics
import sys
import time

import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
TARGETS = (15, 30, 60, 120)
REPEATS = 3
RESULT_RE = re.compile(
    r"u (OK|TIMEOUT) target=(\d+) OL=(\d+) OR=(\d+) HL=(\d+) HR=(\d+) "
    r"errL=(-?\d+) errR=(-?\d+) revL=([\d.]+) revR=([\d.]+) ms=(\d+)"
)


rows = []
with serial.Serial(PORT, 115200, timeout=0.10) as ser:
    time.sleep(7.0)
    for target in TARGETS:
        for repeat in range(1, REPEATS + 1):
            ser.reset_input_buffer()
            ser.write(f"u {target}\n".encode())
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
                print(f"TARGET={target} REP={repeat} FAIL response={line!r}", flush=True)
                continue
            status, target_s, ol, orange, hl, hr, err_l, err_r, rev_l, rev_r, elapsed = match.groups()
            values = tuple(map(int, (target_s, ol, orange, hl, hr, err_l, err_r, elapsed)))
            _, ol_i, or_i, hl_i, hr_i, _, _, elapsed_i = values
            hall_expected = target * 45.0 / 60.0
            hall_error_l = 100.0 * (hl_i - hall_expected) / hall_expected
            hall_error_r = 100.0 * (hr_i - hall_expected) / hall_expected
            sync_hall = hl_i - hr_i
            rows.append((target, repeat, status, ol_i, or_i, hl_i, hr_i,
                         hall_error_l, hall_error_r, sync_hall, elapsed_i))
            print(
                f"TARGET={target:3d} REP={repeat} STATUS={status} "
                f"OL={ol_i:3d} OR={or_i:3d} HL={hl_i:3d} HR={hr_i:3d} "
                f"H_EXPECTED={hall_expected:5.1f} HERR_L={hall_error_l:+6.1f}% "
                f"HERR_R={hall_error_r:+6.1f}% SYNC_H={sync_hall:+3d} MS={elapsed_i}",
                flush=True,
            )
            time.sleep(1.25)

    ser.write(b"STOP\nINHABILITAR\n")
    ser.flush()

print("SUMMARY")
for target in TARGETS:
    selected = [row for row in rows if row[0] == target]
    if not selected:
        continue
    successes = sum(row[2] == "OK" for row in selected)
    hall_l = [row[5] for row in selected]
    hall_r = [row[6] for row in selected]
    errors_l = [abs(row[7]) for row in selected]
    errors_r = [abs(row[8]) for row in selected]
    sync = [abs(row[9]) for row in selected]
    times = [row[10] for row in selected]
    print(
        f"TARGET={target:3d} success={successes}/{len(selected)} "
        f"HL_mean={statistics.mean(hall_l):6.2f} HR_mean={statistics.mean(hall_r):6.2f} "
        f"abs_err_L={statistics.mean(errors_l):6.2f}% abs_err_R={statistics.mean(errors_r):6.2f}% "
        f"sync_abs_mean={statistics.mean(sync):4.1f} hall_pulses "
        f"time_mean={statistics.mean(times):7.1f}ms"
    )
