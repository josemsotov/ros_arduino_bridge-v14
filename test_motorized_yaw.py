#!/usr/bin/env python3
"""Protected low-angle motorized yaw sign/response test."""

import json
import math
import re
import time
import argparse
from pathlib import Path

import serial


PORT = "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_85438333036351A040D0-if00"
OUT = Path("/home/josemsotov/robot_ws/motorized_yaw_test.json")
GYRO_RE = re.compile(
    r"\bT .*?\bgyr=([-+0-9.eE]+).*?\bLpwm=(\d+).*?\bRpwm=(\d+)"
)


def send(port, command):
    port.write((command + "\n").encode("ascii"))
    port.flush()


def stop(port):
    for _ in range(3):
        send(port, "v 0.0 0.0")
        time.sleep(0.06)


def turn(
    port,
    name,
    angular,
    target_deg=12.0,
    timeout=3.0,
    decelerate_at_deg=None,
    slow_angular=None,
):
    samples = []
    angle_rad = 0.0
    start = time.monotonic()
    last_sample = start
    next_command = 0.0
    active_angular = angular
    deceleration_started_at = None
    while time.monotonic() - start < timeout:
        now = time.monotonic()
        if now >= next_command:
            send(port, f"v 0.0000 {active_angular:.4f}")
            next_command = now + 0.15
        line = port.readline().decode("utf-8", errors="replace").strip()
        match = GYRO_RE.search(line)
        if not match:
            if time.monotonic() - start > 0.6 and not samples:
                raise RuntimeError("No T gyro/PWM telemetry within 0.6s; protected abort")
            continue
        now = time.monotonic()
        gyro = float(match.group(1))
        left_pwm = int(match.group(2))
        right_pwm = int(match.group(3))
        dt = min(0.1, max(0.0, now - last_sample))
        last_sample = now
        angle_rad += gyro * dt
        angle_deg = math.degrees(angle_rad)
        if (
            decelerate_at_deg is not None
            and slow_angular is not None
            and abs(angle_deg) >= decelerate_at_deg
            and abs(active_angular) > abs(slow_angular)
        ):
            active_angular = math.copysign(abs(slow_angular), angular)
            deceleration_started_at = angle_deg
            next_command = 0.0
        samples.append({
            "t": now - start,
            "gyro_rad_s": gyro,
            "angle_deg": angle_deg,
            "left_pwm": left_pwm,
            "right_pwm": right_pwm,
            "command_rad_s": active_angular,
        })
        if abs(angle_deg) >= target_deg:
            break
    stop(port)
    result = {
        "name": name,
        "command_rad_s": angular,
        "measured_angle_deg": math.degrees(angle_rad),
        "elapsed_s": time.monotonic() - start,
        "samples": samples,
        "target_reached": abs(math.degrees(angle_rad)) >= target_deg,
        "deceleration_started_at_deg": deceleration_started_at,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--closed-loop", action="store_true")
    parser.add_argument("--angular", type=float, default=0.30)
    parser.add_argument("--negative-angular", type=float, default=None)
    parser.add_argument("--timeout", type=float, default=None)
    parser.add_argument("--decelerate-at", type=float, default=None)
    parser.add_argument("--negative-decelerate-at", type=float, default=None)
    parser.add_argument("--slow-angular", type=float, default=0.25)
    parser.add_argument("--target-deg", type=float, default=12.0)
    args = parser.parse_args()
    results = []
    with serial.Serial(PORT, 115200, timeout=0.05, write_timeout=0.5) as port:
        time.sleep(7.0)
        port.reset_input_buffer()
        try:
            send(port, "hb off")
            send(port, "hc on" if args.closed_loop else "hc off")
            stop(port)
            timeout = args.timeout if args.timeout is not None else (4.0 if args.closed_loop else 3.0)
            results.append(turn(
                port, "positive_command", args.angular,
                target_deg=args.target_deg, timeout=timeout,
                decelerate_at_deg=args.decelerate_at,
                slow_angular=args.slow_angular,
            ))
            time.sleep(2.0)
            negative_angular = (
                args.negative_angular
                if args.negative_angular is not None
                else args.angular
            )
            negative_decelerate_at = (
                args.negative_decelerate_at
                if args.negative_decelerate_at is not None
                else args.decelerate_at
            )
            results.append(turn(
                port, "negative_command", -negative_angular,
                target_deg=args.target_deg, timeout=timeout,
                decelerate_at_deg=negative_decelerate_at,
                slow_angular=args.slow_angular,
            ))
        finally:
            stop(port)
    OUT.write_text(json.dumps({"closed_loop": args.closed_loop, "results": results}, indent=2) + "\n")
    for result in results:
        print(
            f"{result['name']}: angle={result['measured_angle_deg']:.2f}deg "
            f"elapsed={result['elapsed_s']:.2f}s reached={result['target_reached']}"
        )
    print(f"Saved: {OUT}")


if __name__ == "__main__":
    main()
