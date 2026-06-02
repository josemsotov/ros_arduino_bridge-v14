"""
test_right_motor.py — Solo activa el motor DERECHO
Calcula v/w para que v_left=0 y v_right>0 (giro puro CCW con pivot izquierdo).

  v_left  = linear + angular * wb2 = 0  →  linear = -angular * wb2
  v_right = linear - angular * wb2 > 0

Con wb2=0.41 y angular=-0.5: linear=0.205, v_left=0, v_right=0.41
"""

import serial, time, sys

PORT  = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD  = 115200
V_CMD = "v 0.205 -0.500\n"   # solo motor derecho
STOP  = "v 0.0 0.0\n"
SECS  = 4                     # duración del test

print(f"[TEST] Conectando a {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)  # esperar reset Arduino

print(f"[TEST] Enviando: {V_CMD.strip()}  ({SECS}s)")
t0 = time.time()
while time.time() - t0 < SECS:
    ser.write(V_CMD.encode())
    # Leer y mostrar telemetría
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line.startswith("T "):
            parts = {k: v for k, v in (t.split("=") for t in line[2:].split() if "=" in t)}
            print(f"  Lpwm={parts.get('Lpwm','?'):>3}  Rpwm={parts.get('Rpwm','?'):>3} "
                  f" Lrpm={parts.get('Lrpm','?'):>5}  Rrpm={parts.get('Rrpm','?'):>5} "
                  f" Ld={parts.get('Ld','?')} Rd={parts.get('Rd','?')}")
        elif line:
            print(f"  [ARD] {line}")
    time.sleep(0.1)

ser.write(STOP.encode())
time.sleep(0.3)
ser.close()
print("[TEST] Motor parado. Fin.")
