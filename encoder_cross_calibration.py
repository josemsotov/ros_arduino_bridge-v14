#!/usr/bin/env python3
"""Cross-calibrate 60-PPR optoencoders against 45-PPR Hall sensors."""
import argparse
import csv
import json
import re
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

HALL_PPR = 45.0
OPTO_PPR = 60.0
EXPECTED_RATIO = OPTO_PPR / HALL_PPR
PWMS = (10, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80)
REPEATS = 3
Q_RE = re.compile(r"q OK side=([LR]) pwm=(\d+) L=(\d+) R=(\d+) OL=(\d+) OR=(\d+)")
O_RE = re.compile(r"o (\d+) (\d+)")


def response(ser, command, prefix, timeout=5.0, attempts=3):
    seen = []
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write((command + "\n").encode())
        ser.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                seen.append(line)
            if line.startswith(prefix):
                return line
    raise RuntimeError(f"No response for {command!r} after {attempts} attempts; tail={seen[-8:]}")


def stop(ser):
    ser.write(b"STOP\nINHABILITAR\n")
    ser.flush()
    time.sleep(0.25)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--output", default="encoder_cross_calibration.json")
    ap.add_argument("--rest-seconds", type=float, default=10.0)
    args = ap.parse_args()
    output = Path(args.output)
    rows = []
    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "hall_ppr": HALL_PPR,
        "opto_ppr": OPTO_PPR,
        "expected_opto_per_hall": EXPECTED_RATIO,
        "pwms": list(PWMS),
        "repeats": REPEATS,
    }

    with serial.Serial(args.port, 115200, timeout=0.10) as ser:
        time.sleep(7.0)
        try:
            stop(ser)
            rest_start_line = response(ser, "o", "o ", 5.0)
            rest_start = O_RE.fullmatch(rest_start_line)
            if not rest_start:
                raise RuntimeError(f"Unexpected rest start: {rest_start_line}")
            time.sleep(args.rest_seconds)
            rest_end_line = response(ser, "o", "o ", 5.0)
            rest_end = O_RE.fullmatch(rest_end_line)
            if not rest_end:
                raise RuntimeError(f"Unexpected rest end: {rest_end_line}")
            l0, r0 = map(int, rest_start.groups())
            l1, r1 = map(int, rest_end.groups())
            report["rest"] = {
                "seconds": args.rest_seconds,
                "left_start": l0, "right_start": r0,
                "left_end": l1, "right_end": r1,
                "left_delta": l1 - l0, "right_delta": r1 - r0,
            }

            for pwm in PWMS:
                for repeat in range(1, REPEATS + 1):
                    for side in ("L", "R"):
                        q_line = response(ser, f"q {side} {pwm}", "q ", 5.0)
                        q = Q_RE.fullmatch(q_line)
                        if not q:
                            raise RuntimeError(f"Unexpected q response: {q_line}")
                        _, actual_pwm, hall_l, hall_r, opto_l, opto_r = q.groups()
                        hall = int(hall_l if side == "L" else hall_r)
                        opto = int(opto_l if side == "L" else opto_r)
                        expected = hall * EXPECTED_RATIO
                        signed_error = None if expected == 0 else 100.0 * (opto - expected) / expected
                        row = {
                            "pwm": int(actual_pwm), "repeat": repeat, "side": side,
                            "hall": hall, "opto": opto, "expected_opto": expected,
                            "signed_error_pct": signed_error,
                            "ratio_opto_per_hall": None if hall == 0 else opto / hall,
                        }
                        rows.append(row)
                        print(json.dumps(row), flush=True)
                        time.sleep(0.5)
        finally:
            stop(ser)

    report["rows"] = rows
    summary = []
    for pwm in PWMS:
        for side in ("L", "R"):
            group = [r for r in rows if r["pwm"] == pwm and r["side"] == side]
            errors = [r["signed_error_pct"] for r in group if r["signed_error_pct"] is not None]
            summary.append({
                "pwm": pwm, "side": side, "samples": len(group),
                "hall_mean": statistics.mean(r["hall"] for r in group),
                "opto_mean": statistics.mean(r["opto"] for r in group),
                "error_mean_pct": statistics.mean(errors) if errors else None,
                "error_stdev_pct": statistics.stdev(errors) if len(errors) > 1 else 0.0,
                "error_max_abs_pct": max(map(abs, errors)) if errors else None,
            })
    report["summary"] = summary
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    csv_path = output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"REPORT_JSON={output}")
    print(f"REPORT_CSV={csv_path}")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
