"""
diag_hall.py — Diagnóstico de sensores Hall por etapas
Smart Golf Trolley — MOTOR-INTERFACE-V14

Etapas:
  D1  Ping básico — verifica comunicación serial
  D2  Estado del sistema al arranque
  D3  Pulsos en reposo — ¿llegan pulsos sin que los motores giren?
  D4  Motor izquierdo solo — ¿Hall izquierdo cuenta?
  D5  Motor derecho solo  — ¿Hall derecho cuenta?
  D6  Ambos motores       — ¿Ambos Hall cuentan?
  D7  Monitor en tiempo real de RPM (HALL_DEBUG)

Protocolo usado:
  r        → reset encoders  → "r OK"
  e        → leer encoders   → "e <L> <R>"
  HABILITAR / INHABILITAR → STATE:HABILITADO / STATE:INHABILITADO
  v <l> <a>→ mover robot
  L <pwm>  → solo motor izquierdo (comando directo)
  R <pwm>  → solo motor derecho   (comando directo)
  HALL_DEBUG → activa debug continuo de RPM
  STOP_DEBUG → desactiva debug continuo
"""

import serial
import serial.tools.list_ports
import time
import sys

# ── Colores ─────────────────────────────────────────────────────────────────
R = "\033[91m"; G = "\033[92m"; Y = "\033[93m"
B = "\033[94m"; C = "\033[96m"; W = "\033[97m"
DIM = "\033[2m"; RST = "\033[0m"

BAUD = 115200

def find_arduino():
    keywords = ("arduino", "mega", "ch340", "cp210", "ch341", "ftdi", "uno")
    for p in serial.tools.list_ports.comports():
        if any(k in p.description.lower() for k in keywords):
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None

def flush_read(ser, secs=1.0):
    time.sleep(secs)
    lines = []
    while ser.in_waiting:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        except Exception:
            break
    return lines

def cmd(ser, command, wait=1.0, show=True):
    if show:
        print(f"  {C}>> {command}{RST}")
    ser.write((command.strip() + "\n").encode())
    resp = flush_read(ser, wait)
    for l in resp:
        print(f"  {DIM}<< {l}{RST}")
    return resp

def ok(m):  print(f"  {G}✓ {m}{RST}")
def fail(m):print(f"  {R}✗ {m}{RST}")
def warn(m):print(f"  {Y}⚠ {m}{RST}")
def info(m):print(f"  {B}ℹ {m}{RST}")
def hdr(n, t):
    print(f"\n{'─'*60}")
    print(f"{W} DIAGNÓSTICO {n}: {t}{RST}")
    print(f"{'─'*60}")

def read_encoders(ser):
    """Devuelve (left, right) o None si no llega respuesta."""
    cmd(ser, "r", wait=0.5, show=False)          # reset
    time.sleep(0.1)
    resp = cmd(ser, "e", wait=0.8)
    for l in resp:
        if l.startswith("e "):
            parts = l.split()
            if len(parts) == 3:
                try:
                    return int(parts[1]), int(parts[2])
                except ValueError:
                    pass
    return None

def stop_robot(ser):
    cmd(ser, "v 0.0 0.0", wait=0.3, show=False)
    cmd(ser, "INHABILITAR", wait=0.3, show=False)

# ── D1: Ping básico ──────────────────────────────────────────────────────────
def d1_ping(ser):
    hdr("D1", "Ping basico — comunicacion serial")
    resp = cmd(ser, "TEST")
    if any("FUNCIONANDO" in l.upper() or "OK" in l.upper() for l in resp):
        ok("Serial OK — Arduino responde al comando TEST")
        return True
    else:
        fail("Sin respuesta al TEST — revisa baud rate y que el firmware esté subido")
        return False

# ── D2: Estado al arranque ───────────────────────────────────────────────────
def d2_estado(ser):
    hdr("D2", "Estado del sistema al arranque")
    resp = cmd(ser, "ESTADO")
    if any("STATE:" in l for l in resp):
        for l in resp:
            if "STATE:" in l:
                ok(f"Estado reportado: {l}")
        return True
    else:
        warn("ESTADO no devuelve STATE: — puede ser firmware viejo")
        resp2 = cmd(ser, "STATUS")
        for l in resp2:
            print(f"  {DIM}   {l}{RST}")
        return False

# ── D3: Pulsos en reposo ──────────────────────────────────────────────────────
def d3_reposo(ser):
    hdr("D3", "Pulsos en REPOSO — motores parados")
    info("Si hay pulsos con motores parados → ruido EMI o cableado suelto")
    cmd(ser, "r", wait=0.3, show=False)
    print(f"  {Y}⏳ Esperando 3 segundos con motores parados...{RST}")
    time.sleep(3)
    resp = cmd(ser, "e", wait=0.5)
    counts = None
    for l in resp:
        if l.startswith("e "):
            parts = l.split()
            if len(parts) == 3:
                try:
                    counts = (int(parts[1]), int(parts[2]))
                except ValueError:
                    pass
    if counts is None:
        fail("Sin respuesta de encoders — HALL_SENSORS puede no estar activo")
        return False
    l_c, r_c = counts
    if l_c == 0 and r_c == 0:
        ok(f"Reposo limpio: L={l_c}, R={r_c} — sin ruido  ✓")
    else:
        warn(f"Pulsos en reposo: L={l_c}, R={r_c} — posible ruido o cableado")
    return True

# ── D4: Motor izquierdo solo ─────────────────────────────────────────────────
def d4_motor_izq(ser):
    hdr("D4", "Motor IZQUIERDO solo — Hall izquierdo")
    info("Comando 'HABILITAR' + 'L 40' = solo motor izquierdo a PWM=40")
    warn("El robot se moverá. Asegúrate de tenerlo elevado o con espacio.")
    input(f"  {Y}Pulsa ENTER cuando estés listo...{RST}")

    cmd(ser, "HABILITAR", wait=0.5)
    cmd(ser, "r", wait=0.3, show=False)
    cmd(ser, "L 40", wait=0.2)   # Solo motor izquierdo

    print(f"  {Y}⏳ Motor IZQ girando 3 segundos...{RST}")
    time.sleep(3)

    resp = cmd(ser, "e", wait=0.5)
    stop_robot(ser)

    counts = None
    for l in resp:
        if l.startswith("e "):
            parts = l.split()
            if len(parts) == 3:
                try:
                    counts = (int(parts[1]), int(parts[2]))
                except ValueError:
                    pass

    if counts is None:
        fail("Sin respuesta de encoders")
        return False

    l_c, r_c = counts
    info(f"Pulsos tras 3s motor IZQ: L={l_c}, R={r_c}")

    if l_c > 10:
        ok(f"Hall IZQUIERDO FUNCIONA — detectó {l_c} pulsos  ✓")
    elif l_c > 0:
        warn(f"Hall IZQ detectó solo {l_c} pulsos — sensor débil o velocidad muy baja")
    else:
        fail("Hall IZQUIERDO NO detectó pulsos — revisar:")
        fail("  1. Cable del sensor Hall pin 18")
        fail("  2. Alimentación del sensor (3.3V o 5V)")
        fail("  3. Imán del motor alineado con el sensor")
        fail("  4. Prueba con osciloscopio en pin 18")

    if r_c > 5:
        warn(f"Hall DERECHO también contó {r_c} pulsos con solo motor IZQ activo — revisar diafonia")

    return l_c > 0

# ── D5: Motor derecho solo ───────────────────────────────────────────────────
def d5_motor_der(ser):
    hdr("D5", "Motor DERECHO solo — Hall derecho")
    info("Comando 'HABILITAR' + 'R 40' = solo motor derecho a PWM=40")
    warn("El robot se moverá.")
    input(f"  {Y}Pulsa ENTER cuando estés listo...{RST}")

    cmd(ser, "HABILITAR", wait=0.5)
    cmd(ser, "r", wait=0.3, show=False)
    cmd(ser, "R 40", wait=0.2)   # Solo motor derecho

    print(f"  {Y}⏳ Motor DER girando 3 segundos...{RST}")
    time.sleep(3)

    resp = cmd(ser, "e", wait=0.5)
    stop_robot(ser)

    counts = None
    for l in resp:
        if l.startswith("e "):
            parts = l.split()
            if len(parts) == 3:
                try:
                    counts = (int(parts[1]), int(parts[2]))
                except ValueError:
                    pass

    if counts is None:
        fail("Sin respuesta de encoders")
        return False

    l_c, r_c = counts
    info(f"Pulsos tras 3s motor DER: L={l_c}, R={r_c}")

    if r_c > 10:
        ok(f"Hall DERECHO FUNCIONA — detectó {r_c} pulsos  ✓")
    elif r_c > 0:
        warn(f"Hall DER detectó solo {r_c} pulsos — sensor débil o velocidad muy baja")
    else:
        fail("Hall DERECHO NO detectó pulsos — revisar:")
        fail("  1. Cable del sensor Hall pin 19")
        fail("  2. Alimentación del sensor (3.3V o 5V)")
        fail("  3. Imán del motor alineado con el sensor")
        fail("  4. Prueba con osciloscopio en pin 19")

    if l_c > 5:
        warn(f"Hall IZQUIERDO también contó {l_c} pulsos con solo motor DER activo")

    return r_c > 0

# ── D6: Ambos motores ────────────────────────────────────────────────────────
def d6_ambos(ser):
    hdr("D6", "Ambos motores — v 0.3 0.0")
    warn("El robot avanzará ~3 segundos.")
    input(f"  {Y}Pulsa ENTER cuando estés listo...{RST}")

    cmd(ser, "r", wait=0.3, show=False)
    cmd(ser, "v 0.3 0.0", wait=0.5)

    print(f"  {Y}⏳ Avanzando 3 segundos...{RST}")
    time.sleep(3)

    resp = cmd(ser, "e", wait=0.5)
    stop_robot(ser)

    counts = None
    for l in resp:
        if l.startswith("e "):
            parts = l.split()
            if len(parts) == 3:
                try:
                    counts = (int(parts[1]), int(parts[2]))
                except ValueError:
                    pass

    if counts is None:
        fail("Sin respuesta de encoders")
        return

    l_c, r_c = counts
    info(f"Pulsos tras 3s avance: L={l_c}, R={r_c}")

    if l_c > 10 and r_c > 10:
        ok(f"Ambos Hall FUNCIONAN  ✓  L={l_c}, R={r_c}")
        ratio = max(l_c, r_c) / max(min(l_c, r_c), 1)
        if ratio > 1.5:
            warn(f"Diferencia importante entre ruedas (ratio={ratio:.2f}) — posible patinaje o sensor débil")
        else:
            ok(f"Simetría ruedas OK (ratio={ratio:.2f})")
    elif l_c > 10:
        warn(f"Solo Hall IZQ funciona: L={l_c}, R={r_c}")
    elif r_c > 10:
        warn(f"Solo Hall DER funciona: L={l_c}, R={r_c}")
    else:
        fail(f"Ningún Hall detectó pulsos: L={l_c}, R={r_c}")

# ── D7: Monitor RPM en tiempo real ───────────────────────────────────────────
def d7_monitor(ser):
    hdr("D7", "Monitor RPM en tiempo real (HALL_DEBUG)")
    info("Activa debug continuo del Arduino. Mueve el robot manualmente o usa v <vel>.")
    info("Ctrl+C para detener el monitor.")

    cmd(ser, "HALL_DEBUG", wait=0.3)
    cmd(ser, "HABILITAR",  wait=0.3)

    print(f"  {Y}Monitoreando... (Ctrl+C para salir){RST}")
    try:
        while True:
            time.sleep(0.1)
            while ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(f"  {C}{line}{RST}")
    except KeyboardInterrupt:
        pass

    cmd(ser, "STOP_DEBUG", wait=0.3)
    stop_robot(ser)

# ── MAIN ──────────────────────────────────────────────────────────────────────
def main():
    print(f"\n{W}{'='*60}")
    print("  SMART GOLF TROLLEY — DIAGNÓSTICO HALL SENSORS")
    print(f"  MOTOR-INTERFACE-V14  |  115200 baud")
    print(f"{'='*60}{RST}")

    port = find_arduino()
    if not port:
        fail("No se encontró Arduino."); sys.exit(1)

    print(f"\n{G}Puerto: {port}{RST}")
    input(f"  {Y}Pulsa ENTER para conectar...{RST}")

    try:
        ser = serial.Serial(port, BAUD, timeout=2)
    except serial.SerialException as e:
        fail(f"Error al abrir {port}: {e}"); sys.exit(1)

    print(f"{G}Conectado. Esperando boot (3s)...{RST}")
    boot = flush_read(ser, 3.0)
    for l in boot:
        print(f"  {DIM}[BOOT] {l}{RST}")

    diags = {
        "1": ("Ping basico",            d1_ping),
        "2": ("Estado al arranque",     d2_estado),
        "3": ("Pulsos en reposo",       d3_reposo),
        "4": ("Motor izquierdo solo",   d4_motor_izq),
        "5": ("Motor derecho solo",     d5_motor_der),
        "6": ("Ambos motores (v cmd)",  d6_ambos),
        "7": ("Monitor RPM tiempo real",d7_monitor),
        "t": ("TODOS (D1→D6)",          None),
    }

    while True:
        print(f"\n{C}{'─'*60}")
        print(" MENÚ DE DIAGNÓSTICO HALL")
        print("─"*60)
        for k, (name, _) in diags.items():
            print(f"  [{k}]  {name}")
        print("  [q]  Salir")
        print(f"{'─'*60}{RST}")
        choice = input("  Elige diagnóstico: ").strip().lower()

        if choice == "q":
            break
        elif choice == "t":
            for k in "123456":
                _, fn = diags[k]
                fn(ser)
        elif choice in diags and diags[choice][1]:
            diags[choice][1](ser)
        else:
            warn("Opción no válida")

    stop_robot(ser)
    ser.close()
    print(f"\n{G}Diagnóstico finalizado.{RST}\n")

if __name__ == "__main__":
    main()
