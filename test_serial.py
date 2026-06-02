import serial, time

def read_all(s, secs=1.5):
    time.sleep(secs)
    lines = []
    while s.in_waiting:
        lines.append(s.readline().decode('utf-8', errors='replace').strip())
    return lines

s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(2)

# Leer arranque
startup = read_all(s, 2)
for l in startup:
    print('[BOOT]', l)

# Prueba VTEST - motores deben girar 2 segundos
print('--- Enviando VTEST ---')
s.write(b'VTEST\n')
for l in read_all(s, 0.5):
    print('[VTEST]', l)

print('--- Motores deben estar girando ahora! ---')
time.sleep(3)

# Detener
print('--- Enviando STOP ---')
s.write(b'STOP\n')
for l in read_all(s, 0.5):
    print('[STOP]', l)

# Prueba 'v' ROS2
print('--- Enviando v 0.3 0.0 ---')
s.write(b'v 0.3 0.0\n')
for l in read_all(s, 1.5):
    print('[ROS2-v]', l)

time.sleep(2)
print('--- Enviando v 0.0 0.0 ---')
s.write(b'v 0.0 0.0\n')
for l in read_all(s, 0.5):
    print('[STOP-v]', l)

s.close()
print('--- FIN ---')
