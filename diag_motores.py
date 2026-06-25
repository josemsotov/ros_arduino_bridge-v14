"""
diag_motores.py — Diagnóstico cruzado motor↔Hall
Prueba cada motor por separado con PWM directo (sin PID) y mide
qué sensor Hall responde. Detecta si los sensores están cruzados.

Uso:  python diag_motores.py [PORT]
"""
import sys, serial, time, re

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=2)
print(f"[OK] {PORT} abierto")
print("[..] Esperando boot (6s)...\n")
time.sleep(6)
while ser.in_waiting:
    ser.readline()

# Asegurar estado correcto
ser.write(b"hb off\n");    time.sleep(0.3)
ser.write(b"HABILITAR\n"); time.sleep(0.3)
while ser.in_waiting:
    ser.readline()

def leer_encoders():
    """Pide contadores acumulados al firmware."""
    ser.write(b"e\n")
    time.sleep(0.15)
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line.startswith("e "):
            parts = line.split()
            if len(parts) >= 3:
                return int(parts[1]), int(parts[2])
    return None, None

def test_motor(nombre, pwm_cmd, dur=2.0):
    """
    Aplica PWM directo a UN solo motor durante dur segundos.
    pwm_cmd: bytes a enviar  ej. b'L 50\n'  o  b'R 50\n'
    Devuelve (delta_left, delta_right) pulsos Hall.
    """
    # Resetear encoders
    ser.write(b"r\n"); time.sleep(0.2)
    while ser.in_waiting: ser.readline()

    l0, r0 = leer_encoders()
    if l0 is None:
        l0, r0 = 0, 0

    print(f"  [{nombre}] Aplicando PWM directo {dur:.0f}s...", end="", flush=True)
    ser.write(pwm_cmd)
    time.sleep(dur)

    # Parar
    ser.write(b"v 0.0 0.0\n"); time.sleep(0.2)
    while ser.in_waiting: ser.readline()

    l1, r1 = leer_encoders()
    if l1 is None:
        l1, r1 = l0, r0

    dl = l1 - l0
    dr = r1 - r0
    print(f"  Hall_IZQ={dl:>5}  Hall_DER={dr:>5}")
    return dl, dr

print("=" * 55)
print("  DIAGNÓSTICO MOTOR ↔ HALL SENSOR")
print("  Observa físicamente qué motor gira en cada test.")
print("=" * 55)
print()

# Test 1: Solo motor izquierdo — usando p con motor derecho inhibido
# El firmware no tiene comando de un solo motor con DIR correcto salvo p (ambos)
# Usaremos 'v' con velocidad lineal para probar el sistema completo
print("  NOTA: Los tests usan 'p <pwm>' (PWM directo ambos motores)")
print("  Observa qué rueda produce pulsos Hall.")
print()

input("  Presiona ENTER para girar AMBOS MOTORES (PWM=50 adelante, 2s)...")
dl_both, dr_both = test_motor("AMBOS", b"p 50\n", dur=2.0)

time.sleep(1.0)

print()
print("=" * 55)
print("  RESULTADO:")
print("=" * 55)
print(f"  Hall_IZQ (pin 18): {dl_both:>5} pulsos")
print(f"  Hall_DER (pin 19): {dr_both:>5} pulsos")
print()
if dl_both > 10 and dr_both > 10:
    print("  [OK] Ambos Hall responden correctamente.")
elif dl_both > 10 and dr_both <= 3:
    print("  [!!] Solo Hall_IZQ (pin 18) responde.")
    print("       El Hall del motor derecho no genera pulsos.")
    print("       Verifica conexión del sensor Hall del motor derecho.")
elif dr_both > 10 and dl_both <= 3:
    print("  [!!] Solo Hall_DER (pin 19) responde.")
    print("       El Hall del motor izquierdo no genera pulsos.")
    print("       Verifica conexión del sensor Hall del motor izquierdo.")
else:
    print("  [!!] NINGÚN Hall responde. Verifica alimentación 5V de los sensores Hall.")

print()
ser.write(b"INHABILITAR\n")
ser.close()
print("  [OK] Motores detenidos.")
