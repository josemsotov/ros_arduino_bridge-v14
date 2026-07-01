#!/usr/bin/env python3
"""
stadia_pi.py — Control Stadia → Arduino Mega en Raspberry Pi 5
Usa evdev (Linux) en lugar de pywinusb (Windows)

Dependencias: python3-evdev (apt), pyserial (pip)
Uso: python3 stadia_pi.py [--scan] [--test]
"""
import sys, time, threading, serial, serial.tools.list_ports
import evdev
from evdev import ecodes

# ── Parámetros ──────────────────────────────────────────────────────────────
PORT          = "/dev/ttyACM0"
BAUD          = 115200
SEND_RATE_HZ  = 10
MAX_LINEAR    = 0.5     # m/s
MAX_ANGULAR   = 0.8     # rad/s
DEADZONE_LIN  = 0.12
DEADZONE_ANG  = 0.18    # Reducida: el mando ya distingue bien izq/der
ANGULAR_EXPO  = 2.0
# Rotación pura: si |angular| > ROTATION_DOMINANCE × |linear|, linear=0
# Valor 2.0 → el angular debe ser el doble del lineal para activar modo rotación
ROTATION_DOMINANCE = 2.0

# ── Ejes Stadia en Linux (ABS) ───────────────────────────────────────────────
ABS_LX   = ecodes.ABS_X
ABS_LY   = ecodes.ABS_Y
ABS_RX   = ecodes.ABS_Z
ABS_RY   = ecodes.ABS_RZ
BTN_A    = ecodes.BTN_A
BTN_B    = ecodes.BTN_B
BTN_X    = ecodes.BTN_X
BTN_Y    = ecodes.BTN_Y
BTN_MENU = ecodes.BTN_START
BTN_CAP  = ecodes.BTN_SELECT

AXIS_MID = 128.0
AXIS_MAX = 127.0

# ── Colores ANSI ──────────────────────────────────────────────────────────────
G="\033[92m"; R="\033[91m"; Y="\033[93m"; C="\033[96m"; W="\033[97m"; X="\033[0m"

# ── Utilidades ────────────────────────────────────────────────────────────────
def normalize(raw):
    return (raw - AXIS_MID) / AXIS_MAX

def deadzone(v, dz):
    if abs(v) < dz: return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - dz) / (1.0 - dz)

def expo(v, e):
    return (abs(v) ** e) * (1.0 if v >= 0 else -1.0)

def find_stadia():
    for dev in [evdev.InputDevice(p) for p in evdev.list_devices()]:
        if "stadia" in dev.name.lower() or "18d1" in dev.phys.lower() \
           or (hasattr(dev, 'info') and dev.info.vendor == 0x18d1):
            return dev
        dev.close()
    return None

# ── Scan ──────────────────────────────────────────────────────────────────────
if "--scan" in sys.argv:
    print("Gamepads:"); [print(f"  {evdev.InputDevice(p).name}  {p}") for p in evdev.list_devices()]
    print("\nPuertos COM:"); [print(f"  {p.device}  {p.description}") for p in serial.tools.list_ports.comports()]
    sys.exit(0)

# ── Test ──────────────────────────────────────────────────────────────────────
if "--test" in sys.argv:
    dev = find_stadia()
    if not dev: print(f"{R}Stadia no encontrado{X}"); sys.exit(1)
    dev.grab()
    print(f"{G}Stadia conectado: {dev.name}{X}\nMueve ejes y pulsa botones. Ctrl+C para salir.\n")
    ax = {}
    try:
        for ev in dev.read_loop():
            if ev.type == ecodes.EV_ABS:
                ax[ev.code] = ev.value
                lx = normalize(ax.get(ABS_LX, 128)); ly = normalize(ax.get(ABS_LY, 128))
                print(f"\r  LX={lx:+.2f} LY={ly:+.2f}   ", end="", flush=True)
            elif ev.type == ecodes.EV_KEY and ev.value:
                print(f"\n  BTN={ecodes.BTN.get(ev.code, ev.code)} code={ev.code}")
    except KeyboardInterrupt: pass
    finally: dev.ungrab(); dev.close()
    sys.exit(0)

# ── MAIN ──────────────────────────────────────────────────────────────────────
print(f"\n{W}{'='*50}")
print("  Smart Golf Trolley — Control Stadia (Pi)")
print(f"{'='*50}{X}")

# Conectar Stadia
dev = find_stadia()
if not dev: print(f"{R}Stadia no encontrado. Conecta por BT y reintenta.{X}"); sys.exit(1)
dev.grab()
print(f"{G}Stadia: {dev.name}{X}")

# Conectar Arduino
print(f"{Y}Conectando a {PORT} @ {BAUD}...{X}")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    dev.ungrab(); dev.close(); print(f"{R}Error serial: {e}{X}"); sys.exit(1)

time.sleep(7)
while ser.in_waiting: ser.readline()
ser.write(b"hb off\n"); time.sleep(0.2)
ser.write(b"HABILITAR\n"); time.sleep(0.3)
while ser.in_waiting: line = ser.readline().decode(errors="replace").strip(); print(f"  [INIT] {line}")

print(f"\n{W}  Left Stick ↑↓ → Lineal   max ±{MAX_LINEAR} m/s")
print(f"  Left Stick ←→ → Angular  max ±{MAX_ANGULAR} rad/s")
print(f"  Botón A     → PARADA EMERGENCIA")
print(f"  Botón Y     → Toggle balance")
print(f"  Ctrl+C      → Salir{X}\n")

# Hilo lector Arduino
balance_on = False
def read_arduino():
    global balance_on
    while True:
        try:
            if ser.in_waiting:
                l = ser.readline().decode(errors="replace").strip()
                if l.startswith("T "):
                    parts = {}
                    for tok in l[2:].split():
                        if "=" in tok: k,_,v = tok.partition("="); parts[k]=v
                    lin=parts.get("lin","0"); ang=parts.get("ang","0")
                    lpwm=parts.get("Lpwm","0"); rpwm=parts.get("Rpwm","0")
                    lrpm=parts.get("Lrpm","0"); rrpm=parts.get("Rrpm","0")
                    lma=parts.get("LmA","0"); rma=parts.get("RmA","0")
                    bal="[BAL]" if balance_on else "     "
                    print(f"\r  {bal} v={float(lin):+.3f} w={float(ang):+.3f} | "
                          f"Lpwm={lpwm:>3} Rpwm={rpwm:>3} | "
                          f"Lrpm={int(float(lrpm)):>4} Rrpm={int(float(rrpm)):>4} | "
                          f"I L={float(lma):+.2f}A R={float(rma):+.2f}A   ", end="", flush=True)
                elif "ACTIVADO" in l: balance_on=True; print(f"\n  [BAL] ACTIVO")
                elif "DESACTIVADO" in l: balance_on=False; print(f"\n  [BAL] inactivo")
                elif "CAIDA" in l: print(f"\n  [BAL] !!! CAÍDA !!!")
        except: pass
        time.sleep(0.02)

threading.Thread(target=read_arduino, daemon=True).start()

# Loop principal
axis = {ABS_LX: 128, ABS_LY: 128}
last_send = 0.0
interval = 1.0 / SEND_RATE_HZ
prev_keys = set()

try:
    for ev in dev.read_loop():
        now = time.monotonic()

        if ev.type == ecodes.EV_ABS:
            axis[ev.code] = ev.value

        elif ev.type == ecodes.EV_KEY:
            if ev.value == 1:   # key down
                if ev.code == BTN_A:
                    ser.write(b"v 0.0 0.0\n")
                    print(f"\n{R}  [A] PARADA DE EMERGENCIA{X}")
                elif ev.code == BTN_Y:
                    if balance_on:
                        ser.write(b"hb off\n"); print(f"\n  [Y] Balance OFF")
                    else:
                        ser.write(b"hb on\n");  print(f"\n  [Y] Balance ON")
                elif ev.code == BTN_X:
                    ser.write(b"z\n"); print(f"\n  [X] Diagnóstico pines")
                elif ev.code == BTN_B:
                    ser.write(b"p 40\n"); print(f"\n  [B] Raw PWM=40 test")
                elif ev.code == BTN_MENU:
                    ser.write(b"s\n"); print(f"\n  [MENU] Status")

        # Enviar a 10 Hz
        if now - last_send >= interval:
            lx_n = normalize(axis.get(ABS_LX, 128))
            ly_n = normalize(axis.get(ABS_LY, 128))
            linear  = deadzone(-ly_n, DEADZONE_LIN) * MAX_LINEAR
            ax_raw  = deadzone(-lx_n, DEADZONE_ANG)
            angular = expo(ax_raw, ANGULAR_EXPO) * MAX_ANGULAR

            # ── Modo rotación pura ──────────────────────────────────────────
            # Si el componente angular domina claramente sobre el lineal,
            # forzar linear=0 para que el robot gire en el sitio limpiamente
            # aunque el stick no esté perfectamente horizontal.
            if angular != 0 and (linear == 0 or abs(angular) > ROTATION_DOMINANCE * abs(linear)):
                linear = 0.0

            ser.write(f"v {linear:.3f} {angular:.3f}\n".encode())
            if linear != 0 or angular != 0:
                modo = "[ROT]" if (linear == 0 and angular != 0) else "     "
                print(f"  {modo} v lin={linear:+.3f} ang={angular:+.3f}    ", end="\r")
            else:
                print(f"  [en reposo]                              ", end="\r")
            last_send = now

except KeyboardInterrupt:
    print(f"\n{Y}Saliendo — parando motores...{X}")
    ser.write(b"v 0.0 0.0\n"); time.sleep(0.3)
    ser.write(b"INHABILITAR\n"); time.sleep(0.2)
finally:
    dev.ungrab(); dev.close(); ser.close()
    print(f"{G}Cerrado.{X}\n")
