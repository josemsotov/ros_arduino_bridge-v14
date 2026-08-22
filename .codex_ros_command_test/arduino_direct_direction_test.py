#!/usr/bin/env python3
import json
import math
import re
import subprocess
import time

import serial
import serial.tools.list_ports


BAUD = 115200
PPR = 45.0
WHEEL_CIRC_M = math.pi * 0.20


def sh(args):
    return subprocess.run(args, text=True, capture_output=True, timeout=8)


def port():
    ports = []
    for p in serial.tools.list_ports.comports():
        text = f"{p.device} {p.description} {p.hwid}".lower()
        score = ("arduino" in text) * 5 + ("ch340" in text or "usb-serial" in text) * 4
        score += ("ttyacm" in p.device.lower() or "ttyusb" in p.device.lower()) * 3
        ports.append((score, p.device, p.description))
    ports.sort(reverse=True)
    return ports[0][1], ports


class A:
    def __init__(self, dev):
        self.s = serial.Serial(dev, BAUD, timeout=0.05)
        time.sleep(3.0)
        self.drain(0.4)

    def drain(self, sec):
        out = []
        end = time.time() + sec
        while time.time() < end:
            while self.s.in_waiting:
                line = self.s.readline().decode(errors="replace").strip()
                if line:
                    out.append(line)
            time.sleep(0.01)
        return out

    def send(self, cmd, wait=0.12):
        self.s.write((cmd + "\n").encode())
        self.s.flush()
        return self.drain(wait)

    def stop(self):
        out = []
        for c in ("p", "v 0 0", "L 0", "R 0"):
            out += self.send(c, 0.07)
        return out

    def enc(self):
        lines = self.send("e", 0.1)
        val = None
        for line in lines:
            m = re.match(r"e\s+(-?\d+)\s+(-?\d+)", line)
            if m:
                val = int(m.group(1)), int(m.group(2))
        return val, lines

    def z(self):
        lines = self.send("z", 0.2)
        return lines

    def close(self):
        self.s.close()


def speed(pulses, seconds):
    return round(((pulses / PPR) * WHEEL_CIRC_M) / seconds, 4)


def run_case(a, label, cmd, seconds=0.9):
    a.stop()
    a.send("r", 0.15)
    z0 = a.z()
    e0, _ = a.enc()
    if e0 is None:
        e0 = (0, 0)
    ack = a.send(cmd, 0.15)
    t0 = time.time()
    samples = []
    while time.time() - t0 < seconds:
        e, _ = a.enc()
        if e:
            samples.append([round(time.time() - t0, 3), e[0], e[1]])
        time.sleep(0.12)
    elapsed = time.time() - t0
    stop = a.stop()
    e1, _ = a.enc()
    if e1 is None:
        e1 = e0
    z1 = a.z()
    dl = e1[0] - e0[0]
    dr = e1[1] - e0[1]
    return {
        "label": label,
        "command": cmd,
        "duration_s": round(elapsed, 3),
        "ack": ack,
        "encoder_delta": [dl, dr],
        "speed_mps": [speed(dl, elapsed), speed(dr, elapsed)],
        "position_m": [round((dl / PPR) * WHEEL_CIRC_M, 4), round((dr / PPR) * WHEEL_CIRC_M, 4)],
        "z_before": z0,
        "z_after": z1,
        "stop": stop,
        "samples": samples,
    }


def main():
    report = {"mode": "arduino_direction_direct", "tests": [], "service": []}
    for c in (["systemctl", "--user", "stop", "robot-follower.service"],):
        r = sh(c)
        report["service"].append({"cmd": " ".join(c), "rc": r.returncode, "stderr": r.stderr})
    time.sleep(1.0)
    dev, candidates = port()
    report["port"] = dev
    report["port_candidates"] = candidates
    a = None
    try:
        a = A(dev)
        a.send("HABILITAR", 0.3)
        for label, cmd in (
            ("raw_p_forward_dir_right_false", "p 60"),
            ("high_level_forward_moveForward", "AD60"),
            ("high_level_backward_moveBackward", "AT60"),
            ("cmdvel_forward", "v 0.06 0"),
            ("cmdvel_backward", "v -0.06 0"),
        ):
            report["tests"].append(run_case(a, label, cmd))
        a.stop()
        a.send("INHABILITAR", 0.2)
    finally:
        if a:
            try:
                a.stop()
                a.send("INHABILITAR", 0.1)
                a.close()
            except Exception:
                pass
        for c in (["systemctl", "--user", "start", "robot-follower.service"],):
            r = sh(c)
            report["service"].append({"cmd": " ".join(c), "rc": r.returncode, "stderr": r.stderr})
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
