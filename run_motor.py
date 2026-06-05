"""
run_motor.py  —  Test directo de motores
Uso:  python run_motor.py [PORT]   (default: COM4)

Controles:
  w / s   — aumentar / reducir velocidad  (+0.1 m/s)
  a / d   — giro izquierda / derecha
  SPACE   — parar
  q       — salir
"""

import sys
import serial
import threading
import time
import re
import msvcrt   # Windows keyboard non-blocking

PORT  = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD  = 115200
STEP  = 0.1        # m/s por pulsación
MAX_V = 0.8
MAX_W = 1.5

lin = 0.0
ang = 0.0
running = True

# ── Abrir serial ──────────────────────────────────────────────────────────
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
except serial.SerialException as e:
    print(f"[ERR] No se puede abrir {PORT}: {e}")
    sys.exit(1)

print(f"[OK]  Puerto {PORT} abierto")
print("[..]  Esperando boot Arduino (6s)...")
time.sleep(6)
while ser.in_waiting:
    ser.readline()

# Desactivar balance y forzar estado HABILITADO
ser.write(b"hb off\n");  time.sleep(0.2)
ser.write(b"HABILITAR\n"); time.sleep(0.3)
while ser.in_waiting:
    l = ser.readline().decode("utf-8", errors="replace").strip()
    if l: print(" ", l)

print()
print("─────────────────────────────────────────────────")
print("  CONTROL DE MOTORES")
print("  w/s = adelante/atrás   a/d = giro   SPACE = stop   q = salir")
print("─────────────────────────────────────────────────")
print()

# ── Hilo lector de telemetría ─────────────────────────────────────────────
def reader():
    while running:
        try:
            raw = ser.readline()
            if not raw:
                continue
            l = raw.decode("utf-8", errors="replace").strip()
            if l.startswith("T "):
                # Extraer campos — LmA/RmA opcionales (dependen de ENABLE_CURRENT_SENSORS)
                Lpwm = re.search(r"Lpwm=(\S+)", l)
                Rpwm = re.search(r"Rpwm=(\S+)", l)
                Lrpm = re.search(r"Lrpm=(\S+)", l)
                Rrpm = re.search(r"Rrpm=(\S+)", l)
                LmA  = re.search(r"LmA=(\S+)",  l)
                RmA  = re.search(r"RmA=(\S+)",  l)
                if Lpwm and Lrpm:
                    mA_str = ""
                    if LmA and RmA:
                        mA_str = f" | LmA={float(LmA.group(1)):>6.2f} RmA={float(RmA.group(1)):>6.2f}"
                    print(
                        f"\r  Lpwm={Lpwm.group(1):>3} Rpwm={Rpwm.group(1) if Rpwm else '?':>3} | "
                        f"Lrpm={Lrpm.group(1):>5} Rrpm={Rrpm.group(1) if Rrpm else '?':>5}"
                        f"{mA_str}     ",
                        end="", flush=True
                    )
        except Exception:
            pass

t = threading.Thread(target=reader, daemon=True)
t.start()

# ── Bucle principal ───────────────────────────────────────────────────────
try:
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            if key == "q":
                break
            elif key == "w":
                lin = min(lin + STEP, MAX_V)
            elif key == "s":
                lin = max(lin - STEP, -MAX_V)
            elif key == "a":
                ang = min(ang + STEP * 3, MAX_W)
            elif key == "d":
                ang = max(ang - STEP * 3, -MAX_W)
            elif key == " ":
                lin = 0.0
                ang = 0.0
            print(f"\n  >> v={lin:+.1f} m/s   w={ang:+.1f} rad/s", flush=True)

        try:
            ser.write(f"v {lin:.2f} {ang:.2f}\n".encode())
        except serial.SerialException as e:
            print(f"\n[ERR] Puerto perdido: {e}")
            break
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    running = False
    try:
        ser.write(b"v 0.0 0.0\n")
        time.sleep(0.1)
    except Exception:
        pass
    ser.close()
    print("\n[OK]  Motores parados. Saliendo.")
