#!/usr/bin/env python3
import json
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field

import serial
import serial.tools.list_ports


BAUD = 115200
PPR = 45.0
WHEEL_DIAM_M = 0.20
WHEEL_CIRC_M = math.pi * WHEEL_DIAM_M
BOTH_PWM_LEVELS = [40, 50, 60, 70]
SINGLE_PWM_LEVELS = [40, 60]
BOTH_DURATION_S = 1.25
SINGLE_DURATION_S = 1.0


def run(cmd, timeout=8):
    try:
        return subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
    except Exception as exc:
        return exc


def find_port():
    candidates = []
    for p in serial.tools.list_ports.comports():
        text = f"{p.device} {p.description} {p.hwid}".lower()
        score = 0
        if "arduino" in text:
            score += 5
        if "ch340" in text or "usb-serial" in text or "usb serial" in text:
            score += 4
        if "ttyacm" in p.device.lower() or "ttyusb" in p.device.lower():
            score += 3
        candidates.append((score, p.device, p.description, p.hwid))
    candidates.sort(reverse=True)
    if not candidates:
        return None, []
    return candidates[0][1], candidates


def parse_encoder(line):
    m = re.match(r"^e\s+(-?\d+)\s+(-?\d+)", line.strip())
    if not m:
        return None
    return int(m.group(1)), int(m.group(2))


def parse_z_line(line):
    out = {}
    if line.startswith("z L "):
        out["side"] = "L"
    elif line.startswith("z R "):
        out["side"] = "R"
    elif line.startswith("z HALL "):
        out["side"] = "HALL"
    elif line.startswith("z STATE="):
        out["side"] = "STATE"
    else:
        return None
    for key, val in re.findall(r"([A-Za-z_]+)=([^\s|]+)", line):
        if val.replace(".", "", 1).replace("-", "", 1).isdigit():
            out[key] = float(val) if "." in val else int(val)
        else:
            out[key] = val
    return out


@dataclass
class SerialArduino:
    ser: serial.Serial
    log: list[str] = field(default_factory=list)

    def drain(self, seconds=0.1):
        end = time.time() + seconds
        lines = []
        while time.time() < end:
            while self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    lines.append(line)
                    self.log.append(line)
            time.sleep(0.01)
        return lines

    def send(self, cmd, wait=0.15):
        self.ser.write((cmd.strip() + "\n").encode("ascii"))
        self.ser.flush()
        return self.drain(wait)

    def read_encoder(self, wait=0.12):
        lines = self.send("e", wait)
        enc = None
        for line in lines:
            parsed = parse_encoder(line)
            if parsed is not None:
                enc = parsed
        return enc, lines

    def pin_status(self):
        lines = self.send("z", 0.25)
        parsed = {}
        for line in lines:
            z = parse_z_line(line)
            if z:
                parsed[z.pop("side")] = z
        return parsed, lines

    def stop(self):
        lines = []
        for cmd in ("p", "v 0.0 0.0", "L 0", "R 0"):
            lines.extend(self.send(cmd, 0.08))
        return lines


def pulses_to_speed(pulses, elapsed_s):
    if elapsed_s <= 0:
        return 0.0
    rpm = (pulses / PPR) * (60.0 / elapsed_s)
    return rpm * WHEEL_CIRC_M / 60.0


def summarize_pair(l_pulses, r_pulses, elapsed_s):
    l_speed = pulses_to_speed(l_pulses, elapsed_s)
    r_speed = pulses_to_speed(r_pulses, elapsed_s)
    avg = (abs(l_speed) + abs(r_speed)) / 2.0
    mismatch_pct = None if avg < 1e-6 else abs(l_speed - r_speed) / avg * 100.0
    return {
        "left_pulses": l_pulses,
        "right_pulses": r_pulses,
        "left_speed_mps": round(l_speed, 4),
        "right_speed_mps": round(r_speed, 4),
        "left_position_m": round((l_pulses / PPR) * WHEEL_CIRC_M, 4),
        "right_position_m": round((r_pulses / PPR) * WHEEL_CIRC_M, 4),
        "right_left_speed_ratio": None if abs(l_speed) < 1e-6 else round(r_speed / l_speed, 3),
        "mismatch_pct": None if mismatch_pct is None else round(mismatch_pct, 1),
    }


def run_pwm_phase(ard, label, command, duration_s):
    ard.stop()
    ard.send("r", 0.2)
    start_status, start_status_raw = ard.pin_status()
    enc0, enc0_raw = ard.read_encoder()
    if enc0 is None:
        enc0 = (0, 0)

    ack = ard.send(command, 0.18)
    samples = []
    t0 = time.time()
    next_sample = t0
    while time.time() - t0 < duration_s:
        if time.time() >= next_sample:
            enc, raw = ard.read_encoder(0.08)
            if enc:
                samples.append({"t": round(time.time() - t0, 3), "left": enc[0], "right": enc[1]})
            next_sample += 0.22
        time.sleep(0.01)
    elapsed = time.time() - t0

    stop_raw = ard.stop()
    time.sleep(0.15)
    enc1, enc1_raw = ard.read_encoder()
    if enc1 is None:
        enc1 = (samples[-1]["left"], samples[-1]["right"]) if samples else enc0
    end_status, end_status_raw = ard.pin_status()

    dl = enc1[0] - enc0[0]
    dr = enc1[1] - enc0[1]
    result = {
        "label": label,
        "command": command,
        "duration_s": round(elapsed, 3),
        "ack": ack,
        "encoder_start": list(enc0),
        "encoder_end": list(enc1),
        "samples": samples,
        "metrics": summarize_pair(dl, dr, elapsed),
        "pin_status_start": start_status,
        "pin_status_end": end_status,
        "raw": {
            "start_status": start_status_raw,
            "encoder_start": enc0_raw,
            "encoder_end": enc1_raw,
            "stop": stop_raw,
            "end_status": end_status_raw,
        },
    }
    return result


def main():
    report = {
        "mode": "arduino_direct_serial_only",
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "service_actions": [],
        "port_candidates": [],
        "tests": [],
        "final_stop_ok": False,
    }

    for cmd in (
        ["systemctl", "--user", "stop", "robot-follower.service"],
    ):
        res = run(cmd)
        report["service_actions"].append({
            "cmd": " ".join(cmd),
            "returncode": getattr(res, "returncode", None),
            "stdout": getattr(res, "stdout", ""),
            "stderr": getattr(res, "stderr", str(res) if not hasattr(res, "stderr") else res.stderr),
        })

    time.sleep(1.0)
    port, candidates = find_port()
    report["port_candidates"] = [
        {"score": s, "device": d, "description": desc, "hwid": hwid}
        for s, d, desc, hwid in candidates
    ]
    if not port:
        raise SystemExit(json.dumps({"error": "no_serial_port_found", **report}, indent=2))
    report["port"] = port

    ard = None
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
        ard = SerialArduino(ser)
        time.sleep(3.0)
        ard.drain(0.5)
        ard.send("hb off", 0.2)
        ard.stop()
        ard.send("HABILITAR", 0.35)

        initial_status, raw = ard.pin_status()
        report["initial_pin_status"] = initial_status
        report["initial_pin_status_raw"] = raw

        for pwm in BOTH_PWM_LEVELS:
            report["tests"].append(run_pwm_phase(
                ard, f"both_forward_pwm_{pwm}", f"p {pwm}", BOTH_DURATION_S
            ))

        for pwm in SINGLE_PWM_LEVELS:
            report["tests"].append(run_pwm_phase(
                ard, f"left_only_pwm_{pwm}", f"L {pwm}", SINGLE_DURATION_S
            ))
            report["tests"].append(run_pwm_phase(
                ard, f"right_only_pwm_{pwm}", f"R {pwm}", SINGLE_DURATION_S
            ))

        final_stop = ard.stop()
        ard.send("INHABILITAR", 0.25)
        final_status, final_raw = ard.pin_status()
        report["final_stop_raw"] = final_stop
        report["final_pin_status"] = final_status
        report["final_pin_status_raw"] = final_raw
        report["serial_tail"] = ard.log[-200:]
        report["final_stop_ok"] = True
    finally:
        if ard is not None:
            try:
                ard.stop()
                ard.send("INHABILITAR", 0.15)
                ard.ser.close()
            except Exception:
                pass
        for cmd in (
            ["systemctl", "--user", "start", "robot-follower.service"],
        ):
            res = run(cmd)
            report["service_actions"].append({
                "cmd": " ".join(cmd),
                "returncode": getattr(res, "returncode", None),
                "stdout": getattr(res, "stdout", ""),
                "stderr": getattr(res, "stderr", str(res) if not hasattr(res, "stderr") else res.stderr),
            })

    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
