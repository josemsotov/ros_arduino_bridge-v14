"""
test_forward.py — Ambos motores adelante (v=0.5, w=0.0)
Muestra telemetría para diagnosticar por qué falla el motor derecho.
"""
import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200
CMD  = "v 0.500 0.000\n"
STOP = "v 0.0 0.0\n"
SECS = 4

print(f"[TEST] Conectando {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)

print(f"[TEST] Enviando: {CMD.strip()}  ({SECS}s)\n")
t0 = time.time()
while time.time() - t0 < SECS:
    ser.write(CMD.encode())
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line.startswith("T "):
            p = {k: v for k, v in (t.split("=") for t in line[2:].split() if "=" in t)}
            print(f"  Lpwm={p.get('Lpwm','?'):>3}  Rpwm={p.get('Rpwm','?'):>3}"
                  f"  Lrpm={p.get('Lrpm','?'):>5}  Rrpm={p.get('Rrpm','?'):>5}"
                  f"  Ld={p.get('Ld','?')} Rd={p.get('Rd','?')}")
        elif line:
            print(f"  [ARD] {line}")
    time.sleep(0.1)

ser.write(STOP.encode())
time.sleep(0.3)
ser.close()
print("\n[TEST] Fin.")
