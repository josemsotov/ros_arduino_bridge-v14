"""
test_ki.py — Prueba Ki_v en runtime para verificar que el motor derecho
supera la stiction con control integral.
Uso: python test_ki.py
"""
import serial
import time

PORT = "COM4"
BAUD = 115200

def open_port():
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    time.sleep(3)
    ser.reset_input_buffer()
    return ser

def send(ser, cmd, wait=0.5):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    print(f"  >> {cmd}")
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        print(f"  << {line}")
        lines.append(line)
    return lines

def read_all(ser, duration=1.0):
    t_end = time.time() + duration
    while time.time() < t_end:
        if ser.in_waiting:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(f"  << {line}")
        else:
            time.sleep(0.02)

print("=" * 60)
print("  TEST Ki_v — MOTOR DERECHO STICTION")
print("=" * 60)
print()

ser = open_port()

# 1. Activar Ki_v=0.3 en runtime
print("[1] Activando Kp=0.13 Ki=0.30 (operacion en tierra) ...")
send(ser, "k 0.13 0.30", 0.5)

# 2. Reset contadores Hall
send(ser, "r", 0.3)

# 3. Habilitar
send(ser, "HABILITAR", 0.3)

# 4. Enviar v=0.5 durante 6 segundos re-enviando cada 0.8s
print()
print("[2] Aplicando v=1.0 durante 12 segundos (re-send cada 0.8s)...")
print("    v=1.0 requiere PWM~17 (por encima de MIN_PWM=10), evita oscilacion...")
print()

t_end = time.time() + 12.0
send(ser, "v 1.0 0.0", 0.1)
while time.time() < t_end:
    read_all(ser, 0.7)
    if time.time() < t_end:
        send(ser, "v 1.0 0.0", 0.1)

# 5. Leer contadores Hall
print()
print("[3] Leyendo contadores Hall acumulativos ...")
lines = send(ser, "e", 0.5)

# 6. Parada
send(ser, "v 0.0 0.0", 0.5)  # stop
send(ser, "INHABILITAR", 0.3)

ser.close()

# Analizar resultado
print()
print("=" * 60)
left_pulses = 0
right_pulses = 0
for l in lines:
    # Formato: "e <left> <right>"
    if l.startswith("e "):
        parts = l.split()
        if len(parts) >= 3:
            try:
                left_pulses  = int(parts[1])
                right_pulses = int(parts[2])
            except Exception:
                pass
    if "ENC_LEFT:" in l:
        try:
            left_pulses = int(l.split("ENC_LEFT:")[1].split(",")[0].strip())
        except Exception:
            pass
    if "ENC_RIGHT:" in l:
        try:
            right_pulses = int(l.split("ENC_RIGHT:")[1].split(",")[0].strip())
        except Exception:
            pass

print(f"  Pulsos Izquierdo : {left_pulses}")
print(f"  Pulsos Derecho   : {right_pulses}")
print()
if right_pulses > 10:
    print("  RESULTADO: Motor DERECHO arranco con Ki. Integral funcionando.")
else:
    print("  RESULTADO: Motor derecho sin movimiento detectable.")
    print("  -> Considerar aumentar Ki_v o revisar hardware.")
print("=" * 60)
