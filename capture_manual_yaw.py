#!/usr/bin/env python3
"""Capture filtered MPU yaw during a manual chassis-rotation sign test."""

import json
import re
import time
from pathlib import Path

import serial


PORT = "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_85438333036351A040D0-if00"
OUT = Path("/home/josemsotov/robot_ws/manual_yaw_test.json")
LINE_RE = re.compile(
    r"\bhc enabled=.*?\byaw=([-+0-9.eE]+).*?\braw=([-+0-9.eE]+).*?\bgyro=([-+0-9.eE]+)"
)


def send(port, command):
    port.write((command + "\n").encode("ascii"))
    port.flush()


def main():
    samples = []
    with serial.Serial(PORT, 115200, timeout=0.08, write_timeout=0.5) as port:
        time.sleep(7.0)
        port.reset_input_buffer()
        send(port, "hc off")
        for _ in range(3):
            send(port, "v 0.0 0.0")
        start = time.monotonic()
        deadline = start + 50.0
        while time.monotonic() < deadline:
            send(port, "hc stat")
            response_deadline = time.monotonic() + 0.5
            while time.monotonic() < response_deadline:
                line = port.readline().decode("utf-8", errors="replace").strip()
                match = LINE_RE.search(line)
                if match:
                    samples.append({
                        "t": round(time.monotonic() - start, 3),
                        "yaw_deg": float(match.group(1)),
                        "raw_rad_s": float(match.group(2)),
                        "filtered_rad_s": float(match.group(3)),
                    })
                    break
        for _ in range(3):
            send(port, "v 0.0 0.0")
            time.sleep(0.05)
    OUT.write_text(json.dumps({"samples": samples}, indent=2) + "\n")
    print(f"captured={len(samples)} output={OUT}")


if __name__ == "__main__":
    main()
