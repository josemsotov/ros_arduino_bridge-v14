"""
diag_hall_raw.py — Diagnóstico raw del sensor Hall derecho.
Envía comando 'd' al firmware para leer el estado digital del pin Hall
mientras el motor corre a PWM=60 durante 3 segundos.
"""
import serial
import time

PORT = "COM4"
BAUD = 115200

print("=" * 60)
print("  DIAGNÓSTICO HALL RAW — pin 18 (Motor Derecho)")
print("=" * 60)
print()

ser = serial.Serial(PORT, BAUD, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()

# Habilitar
ser.write(b"HABILITAR\n")
print(">> HABILITAR")
time.sleep(0.5)
while ser.in_waiting:
    print(f"<< {ser.readline().decode(errors='replace').strip()}")

# Enviar comando de diagnóstico 'd'
ser.write(b"d\n")
print(">> d  (diagnóstico 3 segundos...)")
print()

# Leer durante 5 segundos para capturar toda la respuesta
t_end = time.time() + 5.5
transitions_l = []
transitions_r = []
while time.time() < t_end:
    if ser.in_waiting:
        line = ser.readline().decode(errors='replace').strip()
        if line:
            print(f"<< {line}")
            if "d PIN" in line:
                # Registrar transiciones
                parts = {p.split('=')[0]: p.split('=')[1] for p in line.split() if '=' in p and p[0] != 'd'}
                if 'L' in parts:
                    transitions_l.append(parts['L'])
                if 'R' in parts:
                    transitions_r.append(parts['R'])
    else:
        time.sleep(0.01)

# Parar
ser.write(b"INHABILITAR\n")
print()
print(">> INHABILITAR")
time.sleep(0.3)
while ser.in_waiting:
    print(f"<< {ser.readline().decode(errors='replace').strip()}")

ser.close()

print()
print("=" * 60)
print(f"  Transiciones pin L (pin 19): {len(transitions_l)}")
print(f"  Transiciones pin R (pin 18): {len(transitions_r)}")
print()
if len(transitions_r) == 0:
    print("  DIAGNÓSTICO: Pin 18 NO cambia de estado en 3s.")
    print("  → Sensor Hall derecho desconectado o roto.")
    print("  → O motor derecho no gira físicamente.")
elif len(transitions_r) < 5:
    print(f"  DIAGNÓSTICO: Solo {len(transitions_r)} transiciones en 3s — MUY POCAS.")
    print("  → Posible stiction severa: motor arranca/para.")
else:
    print(f"  DIAGNÓSTICO: {len(transitions_r)} transiciones detectadas → sensor OK")
    print("  → Problema era de flanco ISR, no de hardware.")
print("=" * 60)
