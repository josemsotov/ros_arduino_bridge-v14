#!/usr/bin/env python3
"""
motor_test_pi.py — Prueba motores via ROS2 desde la Pi
Envía HABILITAR y mueve adelante 1.5s luego para.
"""
import subprocess, time

ROS = "source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash 2>/dev/null && "

def run(cmd):
    return subprocess.run(
        ROS + cmd, shell=True, capture_output=True, text=True, executable='/bin/bash'
    )

def raw(cmd_str):
    r = run(f"ros2 topic pub --once /arduino/raw_command std_msgs/msg/String '{{data: {cmd_str}}}' 2>&1")
    ok = "publishing" in r.stdout
    print(f"  [{'+' if ok else '!'}] raw: {cmd_str}")

def vel(lin, ang, times=1, rate=10):
    r = run(
        f"ros2 topic pub --rate {rate} --times {times} /cmd_vel "
        f"geometry_msgs/msg/Twist "
        f"'{{linear: {{x: {lin}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {ang}}}}}' 2>&1"
    )
    n = r.stdout.count("publishing #")
    print(f"  [{'+'if n>0 else '!'}] cmd_vel lin={lin} ang={ang} → {n} msgs")

# ── escuchar raw_rx en background ────────────────────────────────────────
listener = subprocess.Popen(
    "bash -c '" + ROS + "timeout 15 ros2 topic echo /arduino/raw_rx'",
    shell=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
)
time.sleep(0.5)

print("=== PRUEBA MOTORES ===")
print("1) Desactivar balance...")
raw("hb off")
time.sleep(0.3)

print("2) Habilitar motores...")
raw("HABILITAR")
time.sleep(0.6)

print("3) Adelante 0.20 m/s por 1.5s...")
vel(0.20, 0.0, times=15, rate=10)

print("4) STOP")
vel(0.0, 0.0, times=1)
time.sleep(0.5)

# ── leer telemetría ───────────────────────────────────────────────────────
listener.terminate()
out = listener.stdout.read()

if out.strip():
    # filtrar solo las líneas de datos relevantes (data: ...)
    lines = [l for l in out.strip().split('\n') if 'data:' in l or l.startswith('T ')]
    print(f"\n--- Telemetría Arduino ({len(lines)} líneas) ---")
    for l in lines[:15]:
        print(" ", l.strip())
else:
    print("\n[!] Sin respuesta en /arduino/raw_rx — verificar conexión Arduino")

print("=== FIN ===")
