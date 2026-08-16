#!/usr/bin/env python3
"""Protected straight-line A/B test: heading control off versus on."""

import json
import math
import re
import time
import argparse
from pathlib import Path

import serial


PORT = "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_85438333036351A040D0-if00"
OUT = Path("/home/josemsotov/robot_ws/straight_heading_test.json")
T_RE = re.compile(
    r"\bT lin=.*?\bhcw=([-+0-9.eE]+).*?\bgyr=([-+0-9.eE]+).*?"
    r"\bLpwm=(\d+).*?\bRpwm=(\d+)"
)


def send(port, command):
    port.write((command + "\n").encode("ascii"))
    port.flush()


def stop(port):
    for _ in range(3):
        send(port, "v 0.0 0.0")
        time.sleep(0.06)


def run_phase(port, name, enabled, velocity=0.18, duration=2.5):
    send(port, "hc on" if enabled else "hc off")
    time.sleep(0.15)
    samples = []
    angle_rad = 0.0
    start = time.monotonic()
    last_sample = start
    next_command = 0.0
    try:
        while time.monotonic() - start < duration:
            now = time.monotonic()
            if now >= next_command:
                send(port, f"v {velocity:.4f} 0.0000")
                next_command = now + 0.15
            line = port.readline().decode("utf-8", errors="replace").strip()
            match = T_RE.search(line)
            if not match:
                if time.monotonic() - start > 0.7 and not samples:
                    raise RuntimeError("No heading telemetry within 0.7s")
                continue
            now = time.monotonic()
            dt = min(0.1, max(0.0, now - last_sample))
            last_sample = now
            gyro = float(match.group(2))
            angle_rad += gyro * dt
            samples.append({
                "t": now - start,
                "hcw_rad_s": float(match.group(1)),
                "gyro_rad_s": gyro,
                "yaw_change_deg": math.degrees(angle_rad),
                "left_pwm": int(match.group(3)),
                "right_pwm": int(match.group(4)),
            })
    finally:
        stop(port)
    return {
        "name": name,
        "heading_control": enabled,
        "velocity_m_s": velocity,
        "duration_s": time.monotonic() - start,
        "estimated_distance_m": velocity * min(duration, time.monotonic() - start),
        "yaw_change_deg": math.degrees(angle_rad),
        "samples": samples,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--velocity", type=float, default=0.18)
    parser.add_argument("--distance", type=float, default=0.45)
    args = parser.parse_args()
    if not 0.08 <= args.velocity <= 0.20:
        raise SystemExit("Protected velocity range is 0.08..0.20 m/s")
    if not 0.10 <= args.distance <= 0.45:
        raise SystemExit("Protected phase distance range is 0.10..0.45 m")
    duration = args.distance / args.velocity
    results = []
    with serial.Serial(PORT, 115200, timeout=0.05, write_timeout=0.5) as port:
        time.sleep(7.0)
        port.reset_input_buffer()
        try:
            send(port, "hb off")
            stop(port)
            results.append(run_phase(
                port, "open_loop", False,
                velocity=args.velocity, duration=duration,
            ))
            time.sleep(2.0)
            results.append(run_phase(
                port, "closed_loop", True,
                velocity=args.velocity, duration=duration,
            ))
        finally:
            stop(port)
            send(port, "hc off")
    OUT.write_text(json.dumps({"results": results}, indent=2) + "\n")
    for result in results:
        print(
            f"{result['name']}: distance={result['estimated_distance_m']:.2f}m "
            f"yaw_change={result['yaw_change_deg']:+.3f}deg "
            f"samples={len(result['samples'])}"
        )
    print(f"Saved: {OUT}")


if __name__ == "__main__":
    main()
