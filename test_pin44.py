"""
test_pin44.py — Escribe PWM directo al pin 44 (LEFT motor driver) via comando serial
Bypasa todo el PID y estado machine para probar el hardware directamente.
"""
import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200

# Usamos el comando 'p' (direct PWM) si existe, o forzamos via 'h' (habilitado) + PWM manual
# Protocolo: enviamos el comando de prueba directo de motores
CMD_ENABLE = "h\n"      # habilitar robot
CMD_FWD    = "f 30\n"   # adelante PWM=30 (ambos motores, prueba directa)
CMD_STOP   = "x\n"      # parar

print(f"[TEST] Conectando {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=0.2)
time.sleep(2)

# Leer mensajes de inicio
while ser.in_waiting:
    print(" ", ser.readline().decode("utf-8", errors="replace").strip())

print("\n[TEST] Habilitando y enviando adelante PWM=30 (3s)...")
ser.write(CMD_ENABLE.encode()); time.sleep(0.3)
while ser.in_waiting:
    print(" ", ser.readline().decode("utf-8", errors="replace").strip())

ser.write(CMD_FWD.encode()); time.sleep(0.3)
while ser.in_waiting:
    print(" ", ser.readline().decode("utf-8", errors="replace").strip())

time.sleep(3)

ser.write(CMD_STOP.encode()); time.sleep(0.3)
while ser.in_waiting:
    print(" ", ser.readline().decode("utf-8", errors="replace").strip())

ser.close()
print("\n[TEST] ¿Cuáles ruedas se movieron? ¿Alguna? ¿Las dos?")
