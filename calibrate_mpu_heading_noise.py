#!/usr/bin/env python3
"""Long protected MPU noise calibration with a representative wheel profile.

The Arduino serial port is owned directly during the test. The profile covers
straight forward/reverse motion, several effective PWM levels, arcs, pivots,
and fast direction changes. Only rest and steady straight segments determine
the noise filter; intentional yaw is retained for validation. STOP is sent on
normal completion, Ctrl-C, serial failure, or any other exception.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial


STATUS_RE = re.compile(r"(?:\bhc enabled=.*?\braw=|\bT .*?\bgyr=)([-+0-9.eE]+)")


@dataclass(frozen=True)
class Phase:
    name: str
    seconds: float
    linear: float
    angular: float
    expected_left_pwm: int
    expected_right_pwm: int
    calibration: bool = False


# FF is currently about 100 PWM/(m/s), wheelbase/2 = 0.41 m. Arc phases
# therefore exercise approximately 30/10 PWM while retaining forward motion.
PROFILE = (
    Phase("rest", 10.0, 0.00, 0.00, 0, 0, True),
    Phase("forward_pwm10", 7.0, 0.10, 0.00, 10, 10, True),
    Phase("forward_pwm20", 7.0, 0.20, 0.00, 20, 20, True),
    Phase("forward_pwm30", 7.0, 0.30, 0.00, 30, 30, True),
    Phase("brief_stop_before_reverse", 0.8, 0.00, 0.00, 0, 0),
    Phase("reverse_pwm10", 7.0, -0.10, 0.00, -10, -10, True),
    Phase("reverse_pwm20", 7.0, -0.20, 0.00, -20, -20, True),
    Phase("reverse_pwm30", 7.0, -0.30, 0.00, -30, -30, True),
    Phase("brief_stop_before_steering", 0.8, 0.00, 0.00, 0, 0),
    Phase("forward_arc_left_30_10", 4.0, 0.20, 0.25, 30, 10),
    Phase("forward_arc_right_10_30", 4.0, 0.20, -0.25, 10, 30),
    Phase("reverse_arc_left", 4.0, -0.20, 0.25, -10, -30),
    Phase("reverse_arc_right", 4.0, -0.20, -0.25, -30, -10),
    Phase("brief_stop_before_pivots", 0.8, 0.00, 0.00, 0, 0),
    Phase("pivot_left", 3.0, 0.00, 0.25, 10, -10),
    Phase("pivot_right_sudden", 3.0, 0.00, -0.25, -10, 10),
    Phase("slalom_left_1", 2.0, 0.20, 0.25, 30, 10),
    Phase("slalom_right_1", 2.0, 0.20, -0.25, 10, 30),
    Phase("slalom_left_2", 2.0, 0.20, 0.25, 30, 10),
    Phase("slalom_right_2", 2.0, 0.20, -0.25, 10, 30),
    Phase("final_stop", 2.0, 0.00, 0.00, 0, 0),
)


def send(port: serial.Serial, command: str) -> None:
    port.write((command.strip() + "\n").encode("ascii"))
    port.flush()


def stop(port: serial.Serial) -> None:
    for _ in range(3):
        send(port, "v 0.0 0.0")
        time.sleep(0.05)


def read_raw_gyro(port: serial.Serial, timeout: float = 0.60) -> float | None:
    send(port, "hc stat")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        match = STATUS_RE.search(line)
        if match:
            return float(match.group(1))
    return None


def run_phase(port: serial.Serial, phase: Phase, scale: float) -> list[float]:
    samples: list[float] = []
    duration = phase.seconds * scale
    deadline = time.monotonic() + duration
    next_drive = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_drive:
            send(port, f"v {phase.linear:.4f} {phase.angular:.4f}")
            next_drive = now + 0.20
        value = read_raw_gyro(port)
        if value is not None and math.isfinite(value):
            samples.append(value)
    return samples


def describe(values: list[float], required: bool = True) -> dict[str, float | int | None]:
    if len(values) < 3:
        if required:
            raise RuntimeError(f"Only {len(values)} valid MPU samples were received")
        return {
            "samples": len(values),
            "mean_rad_s": None,
            "sigma_rad_s": None,
            "peak_residual_rad_s": None,
            "rms_rad_s": None,
        }
    mean = statistics.fmean(values)
    sigma = statistics.stdev(values)
    residuals = [abs(value - mean) for value in values]
    return {
        "samples": len(values),
        "mean_rad_s": mean,
        "sigma_rad_s": sigma,
        "peak_residual_rad_s": max(residuals),
        "rms_rad_s": math.sqrt(statistics.fmean(value * value for value in values)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_85438333036351A040D0-if00")
    parser.add_argument("--alpha", type=float, default=0.18)
    parser.add_argument("--duration-scale", type=float, default=1.0)
    parser.add_argument("--output", default="/home/josemsotov/robot_ws/mpu_heading_noise.json")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if not args.execute:
        total = sum(phase.seconds for phase in PROFILE) * args.duration_scale
        print(f"DRY RUN: {len(PROFILE)} phases, {total:.1f}s total. Add --execute after confirming suspended wheels.")
        return 0
    if not 0.5 <= args.duration_scale <= 3.0:
        raise SystemExit("--duration-scale must be between 0.5 and 3.0")
    if not 0.02 <= args.alpha <= 1.0:
        raise SystemExit("--alpha must be between 0.02 and 1.0")

    phase_results: dict[str, dict[str, object]] = {}
    calibration_groups: list[list[float]] = []
    rest_values: list[float] | None = None

    with serial.Serial(args.port, 115200, timeout=0.06, write_timeout=0.5) as port:
        # Opening USB serial resets the Mega. MPU auto-calibration and the
        # initial gyro bias settling need several seconds before hc responses.
        time.sleep(7.0)
        port.reset_input_buffer()
        try:
            send(port, "hc off")
            stop(port)
            for number, phase in enumerate(PROFILE, start=1):
                print(
                    f"[{number:02d}/{len(PROFILE)}] {phase.name}: "
                    f"v={phase.linear:+.2f} w={phase.angular:+.2f} "
                    f"expected PWM L/R={phase.expected_left_pwm:+d}/{phase.expected_right_pwm:+d}",
                    flush=True,
                )
                values = run_phase(port, phase, args.duration_scale)
                # Short protective stops exist to protect the motor drivers,
                # not to provide statistically useful MPU samples.
                details = describe(
                    values,
                    required=(phase.calibration or phase.linear != 0.0 or phase.angular != 0.0),
                )
                details.update({
                    "linear_m_s": phase.linear,
                    "angular_rad_s": phase.angular,
                    "expected_left_pwm": phase.expected_left_pwm,
                    "expected_right_pwm": phase.expected_right_pwm,
                    "used_for_filter": phase.calibration,
                })
                phase_results[phase.name] = details
                if phase.name == "rest":
                    rest_values = values
                elif phase.calibration:
                    calibration_groups.append(values)

            stop(port)
            if rest_values is None:
                raise RuntimeError("Stationary phase was not recorded")

            # Estimate bias at rest. Noise threshold uses the worst straight
            # segment sigma, never intentional arcs/pivots. The cap of 0.035
            # rad/s (~2 deg/s) prevents hiding genuine slow orientation changes.
            bias = statistics.fmean(rest_values)
            straight_sigmas = [statistics.stdev(group) for group in calibration_groups]
            worst_sigma = max(statistics.stdev(rest_values), *straight_sigmas)
            deadband = min(0.035, max(0.003, 3.0 * worst_sigma))
            send(port, f"hc filter {bias:.6f} {args.alpha:.3f} {deadband:.6f}")
            send(port, "hc stat")

            result = {
                "profile_duration_s": sum(phase.seconds for phase in PROFILE) * args.duration_scale,
                "phases": phase_results,
                "applied_filter": {
                    "bias_rad_s": bias,
                    "ema_alpha": args.alpha,
                    "deadband_rad_s": deadband,
                    "worst_straight_sigma_rad_s": worst_sigma,
                },
            }
            Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result["applied_filter"], indent=2))
            print(f"Full results saved: {args.output}")
        finally:
            stop(port)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyboardInterrupt, serial.SerialException, RuntimeError) as exc:
        print(f"ERROR/STOP: {exc}", file=sys.stderr)
        raise SystemExit(1)
