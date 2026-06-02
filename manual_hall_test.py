"""
manual_hall_test.py — Cuenta pulsos Hall girando las ruedas a mano
Gira una rueda, ve los pulsos crecer en tiempo real.
Ctrl+C para salir.
"""
import serial
import serial.tools.list_ports
import time
import sys
import os

BAUD = 115200

def find_arduino():
    keywords = ("arduino", "mega", "ch340", "cp210", "ch341", "ftdi")
    for p in serial.tools.list_ports.comports():
        if any(k in p.description.lower() for k in keywords):
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None

def send_cmd(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\n").encode())
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        except Exception:
            break
    return lines

def read_encoders(ser):
    resp = send_cmd(ser, "e", wait=0.3)
    for l in resp:
        if l.startswith("e "):
            parts = l.split()
            if len(parts) == 3:
                try:
                    return int(parts[1]), int(parts[2])
                except ValueError:
                    pass
    return None

def clear_line():
    print("\r" + " " * 70 + "\r", end="", flush=True)

# ── MAIN ─────────────────────────────────────────────────────────────────────
port = find_arduino()
if not port:
    print("No se encontró Arduino.")
    sys.exit(1)

print(f"\n{'='*60}")
print("  PRUEBA MANUAL DE SENSORES HALL")
print(f"  Puerto: {port}  |  Baud: {BAUD}")
print(f"{'='*60}\n")
print("  Conectando...")

try:
    ser = serial.Serial(port, BAUD, timeout=2)
except serial.SerialException as e:
    print(f"Error: {e}")
    sys.exit(1)

time.sleep(3)  # boot
ser.reset_input_buffer()

# Reset contadores
send_cmd(ser, "r", wait=0.5)
print("  Contadores reseteados.\n")

print("  ┌─────────────────────────────────────────────────┐")
print("  │  GIRA LAS RUEDAS A MANO — pulsos en tiempo real │")
print("  │  Ctrl+C para salir                              │")
print("  └─────────────────────────────────────────────────┘\n")
print("  {:>6}  {:>8}  {:>8}  {:>10}  {:>10}".format(
    "t(s)", "IZQ(L)", "DER(R)", "delta_L", "delta_R"))
print("  " + "─"*56)

prev_l, prev_r = 0, 0
start = time.time()

try:
    while True:
        counts = read_encoders(ser)
        if counts is None:
            print("  [sin respuesta]")
            time.sleep(0.5)
            continue

        l, r = counts
        dl = l - prev_l
        dr = r - prev_r
        prev_l, prev_r = l, r
        t = time.time() - start

        # Indicador visual de actividad
        bar_l = "█" * min(dl, 20) if dl > 0 else ""
        bar_r = "█" * min(dr, 20) if dr > 0 else ""

        status_l = " ✓" if l > 0 else "  "
        status_r = " ✓" if r > 0 else "  "

        print(f"  {t:6.1f}s  IZQ={l:4d}{status_l}  DER={r:4d}{status_r}  "
              f"+{dl:<4d} {bar_l:<20}  +{dr:<4d} {bar_r}", flush=True)

        time.sleep(0.5)

except KeyboardInterrupt:
    print(f"\n\n  {'─'*56}")
    print(f"  RESULTADO FINAL:")
    counts = read_encoders(ser)
    if counts:
        l, r = counts
        print(f"  IZQ (pin 18 / INT5): {l:5d} pulsos  {'✓ FUNCIONA' if l > 5 else '✗ SIN SEÑAL'}")
        print(f"  DER (pin 19 / INT4): {r:5d} pulsos  {'✓ FUNCIONA' if r > 5 else '✗ SIN SEÑAL'}")
    print(f"  {'─'*56}\n")

ser.close()
