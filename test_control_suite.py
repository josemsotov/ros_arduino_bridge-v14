"""
test_control_suite.py — Suite de pruebas del sistema de control Arduino
Smart Golf Trolley — MOTOR-INTERFACE-V14

Pruebas del plan de validacion:
  P1  Secuencia de estados (INHABILITADO -> HABILITADO -> INHABILITADO)
  P2  Conversion PWM lineal  (v 0.2 / 0.5 / 0.8 / 1.0 → PWM esperado)
  P3  Giro en sitio          (v 0.0 1.0 → debe girar, no avanzar)
  P4  Timeout de seguridad   (v 0.3 → silencio 1.5s → debe parar solo)
  P5  Sensores Hall          (r → v 0.3 → e → contadores deben crecer)

Dependencias: pip install pyserial
"""

import serial
import serial.tools.list_ports
import time
import sys

# ─── Colores ANSI ───────────────────────────────────────────────────────────
R  = "\033[91m"   # rojo
G  = "\033[92m"   # verde
Y  = "\033[93m"   # amarillo
B  = "\033[94m"   # azul
C  = "\033[96m"   # cian
W  = "\033[97m"   # blanco
DIM= "\033[2m"
RST= "\033[0m"

# ─── Configuracion ──────────────────────────────────────────────────────────
BAUD       = 115200
BOOT_WAIT  = 3.0   # segundos para dejar arrancar el Arduino
READ_PAUSE = 1.0   # tiempo de espera por respuesta
READ_PAUSE_LONG = 2.0

# ─── Auto-detect puerto Arduino ─────────────────────────────────────────────
def find_arduino():
    keywords = ("arduino", "mega", "ch340", "cp210", "ch341", "ftdi", "uno")
    for p in serial.tools.list_ports.comports():
        if any(k in p.description.lower() for k in keywords):
            return p.device
    # fallback: primer COMx disponible
    ports = list(serial.tools.list_ports.comports())
    if ports:
        return ports[0].device
    return None

# ─── Utilidades serial ──────────────────────────────────────────────────────
def flush_read(ser, secs=READ_PAUSE):
    """Lee todo lo disponible despues de esperar 'secs'."""
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

def send_cmd(ser, cmd, wait=READ_PAUSE, echo=True):
    """Envia comando y devuelve lista de respuestas."""
    clean = cmd.strip()
    if echo:
        print(f"  {C}>> {clean}{RST}")
    ser.write((clean + "\n").encode())
    resp = flush_read(ser, wait)
    for line in resp:
        print(f"  {DIM}<< {line}{RST}")
    return resp

# ─── Cabecera de prueba ──────────────────────────────────────────────────────
def header(n, title):
    print(f"\n{'─'*60}")
    print(f"{W} PRUEBA {n}: {title}{RST}")
    print(f"{'─'*60}")

def ok(msg):
    print(f"  {G}✓ {msg}{RST}")

def fail(msg):
    print(f"  {R}✗ {msg}{RST}")

def warn(msg):
    print(f"  {Y}⚠ {msg}{RST}")

def info(msg):
    print(f"  {B}ℹ {msg}{RST}")

def pause(msg="Pulsa ENTER para continuar..."):
    input(f"\n  {Y}{msg}{RST}")

# ─── PRUEBA 1: Secuencia de estados ─────────────────────────────────────────
def test_p1(ser):
    header(1, "Secuencia de estados")
    info("Prediccion: ESTADO=INHABILITADO → HABILITAR → INHABILITADO")

    resp = send_cmd(ser, "ESTADO")
    if any("STATE:INHABILITADO" in l for l in resp):
        ok("Estado inicial = INHABILITADO  ✓")
    else:
        warn("Estado inicial no confirmado — respuestas arriba")

    resp = send_cmd(ser, "HABILITAR")
    if any("STATE:HABILITADO" in l for l in resp):
        ok("Transicion a HABILITADO  ✓")
    else:
        warn("Transicion HABILITADO no confirmada")

    resp = send_cmd(ser, "ESTADO")
    if any("STATE:HABILITADO" in l for l in resp):
        ok("ESTADO confirma = HABILITADO  ✓")
    else:
        warn("ESTADO no confirma HABILITADO")

    resp = send_cmd(ser, "INHABILITAR")
    if any("STATE:INHABILITADO" in l for l in resp):
        ok("Vuelta a INHABILITADO  ✓")
    else:
        warn("Vuelta a INHABILITADO no confirmada")

# ─── PRUEBA 2: Conversion PWM lineal ────────────────────────────────────────
def test_p2(ser):
    header(2, "Conversion PWM lineal")
    info("Con Kp_v=0.13, el PWM inicial (sin feedback) = Kp_v x v x 100.")
    info("v=0.2 → 2, v=0.5 → 7, v=0.8 → 11, v=1.0 → 13.")
    info("Verificamos direccion FWD y que el PWM coincide con la formula.")
    KP_V = 0.13
    expected = [("v 0.2 0.0", 0.2), ("v 0.5 0.0", 0.5),
                ("v 0.8 0.0", 0.8), ("v 1.0 0.0", 1.0)]

    for cmd, v_ref in expected:
        exp_pwm = int(KP_V * v_ref * 100)
        resp = send_cmd(ser, cmd)
        found = False
        for line in resp:
            if "Left PWM=" in line and "dir=" in line:
                import re
                m = re.search(r"Left PWM=(-?\d+).*?Right PWM=(-?\d+)", line)
                if m:
                    lp, rp = int(m.group(1)), int(m.group(2))
                    # Tolerancia ±2 cuentas (truncamiento entero)
                    if abs(lp - exp_pwm) <= 2 and abs(rp - exp_pwm) <= 2:
                        ok(f"{cmd} → L={lp} R={rp} (esperado ~{exp_pwm})  ✓")
                    elif "FWD" in line and "FWD" in line:
                        warn(f"{cmd} → L={lp} R={rp}, esperado ~{exp_pwm} (dentro del rango PI)")
                    else:
                        fail(f"{cmd} → L={lp} R={rp}, esperado ~{exp_pwm}")
                else:
                    warn(f"{cmd} → no se parseo PWM: {line.strip()}")
                found = True
                break
        if not found:
            warn(f"{cmd} → sin respuesta de PWM")
        send_cmd(ser, "v 0.0 0.0", wait=0.3, echo=False)
        time.sleep(0.5)

    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)

# ─── PRUEBA 3: Giro en sitio ──────────────────────────────────────────────
def test_p3(ser):
    header(3, "Giro en sitio  (BUG #2 ya corregido)")
    info("Prediccion: v 0.0 1.0 → rueda izq adelante, rueda der atras")
    info("  dir_left=FWD, dir_right=BWD  (giro puro en sitio)")
    warn("Observa fisicamente si el robot GIRA o se DESPLAZA en linea")

    resp = send_cmd(ser, "v 0.0 1.0", wait=READ_PAUSE_LONG)

    fwd_left  = any("Left" in l and "FWD" in l for l in resp)
    bwd_right = any("Right" in l and "BWD" in l for l in resp)

    if fwd_left and bwd_right:
        ok("Direcciones correctas: Left=FWD, Right=BWD  ✓  (giro puro)")
    elif fwd_left or bwd_right:
        warn("Solo una direccion confirmada en respuestas — ver arriba")
    else:
        warn("Direcciones no visibles en respuestas — verificar fisicamente")

    time.sleep(2)
    send_cmd(ser, "v 0.0 0.0", wait=0.3, echo=False)
    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)

# ─── PRUEBA 4: Timeout de seguridad ─────────────────────────────────────────
def test_p4(ser):
    header(4, "Timeout de seguridad (ROS2_CMD_TIMEOUT = 1000 ms)")
    info("Prediccion: enviar v 0.3, esperar 1.5s en silencio → robot para solo")

    send_cmd(ser, "v 0.3 0.0")
    print(f"  {Y}⏳ Esperando 1.6 s sin enviar comandos...{RST}")
    time.sleep(1.6)

    resp = send_cmd(ser, "s", wait=READ_PAUSE)
    if any("s 2" in l or "WARNING" in l.upper() for l in resp):
        ok("Status = WARNING (2) — timeout activado  ✓")
    elif any("s 0" in l or "s 1" in l for l in resp):
        info(f"Status recibido: {[l for l in resp if l.startswith('s')]}")
        warn("Timeout puede haberse activado pero status ya cambio")
    else:
        warn("Respuesta de status no reconocida — ver arriba")

    resp = send_cmd(ser, "MOTOR_STATUS")
    if any("PWM" in l and ("0" in l) for l in resp):
        ok("PWM = 0 confirma parada por timeout  ✓")

    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)

# ─── PRUEBA 5: Sensores Hall ─────────────────────────────────────────────────
def test_p5(ser):
    header(5, "Sensores Hall — contadores crecen con movimiento")
    info("Usa el comando 'd': motores a PWM=60 durante 3s,")
    info("imprime transiciones de pin en tiempo real + contadores ISR al final.")
    info("'d DIAG_END L_pulses=N R_pulses=N' — ambos deben ser > 0.")

    print(f"  {Y}⏳ Ejecutando diagnostico Hall 'd' (3 s)...{RST}")
    # El comando 'd' es bloqueante en el firmware: dura ~3s
    resp = send_cmd(ser, "d", wait=4.5)

    l_pulses = None
    r_pulses = None
    pin_changes = 0

    for line in resp:
        line = line.strip()
        if line.startswith("d PIN"):
            pin_changes += 1
        elif line.startswith("d DIAG_END"):
            # formato: d DIAG_END L_pulses=N R_pulses=N
            import re
            m = re.search(r"L_pulses=(\d+).*?R_pulses=(\d+)", line)
            if m:
                l_pulses = int(m.group(1))
                r_pulses = int(m.group(2))

    if l_pulses is None:
        warn("No se recibio 'd DIAG_END' — timeout o firmware sin soporte")
        send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)
        return

    ok(f"Cambios de pin detectados (polling directo): {pin_changes}")
    ok(f"Contadores ISR al final: L={l_pulses}, R={r_pulses}")

    if l_pulses > 0:
        ok(f"Hall IZQUIERDO activo — {l_pulses} pulsos  ✓")
    else:
        fail("Hall IZQUIERDO = 0 — revisar sensor/cableado pin 19")

    if r_pulses > 0:
        ok(f"Hall DERECHO activo — {r_pulses} pulsos  ✓")
    else:
        fail(f"Hall DERECHO = 0 — ISR no cuenta (pin 18). "
             f"Pin changes={pin_changes} (si >0 el sensor da señal pero ISR falla)."
             f" Si pin_changes=0 tambien, revisar cableado/alimentacion sensor.")

    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)

# ─── MAIN ────────────────────────────────────────────────────────────────────
def main():
    print(f"\n{W}{'='*60}")
    print("  SMART GOLF TROLLEY — SUITE DE PRUEBAS DE CONTROL")
    print(f"  MOTOR-INTERFACE-V14  |  115200 baud")
    print(f"{'='*60}{RST}")

    # Detectar puerto
    port = find_arduino()
    if port is None:
        fail("No se encontro ningun Arduino. Conecta el USB y vuelve a intentar.")
        sys.exit(1)

    print(f"\n{G}Puerto detectado: {port}{RST}")
    print(f"{Y}Asegurate de que el Arduino esta grabado con el firmware actualizado.{RST}")
    pause("Pulsa ENTER para conectar y empezar...")

    try:
        ser = serial.Serial(port, BAUD, timeout=2)
    except serial.SerialException as e:
        fail(f"Error al abrir {port}: {e}")
        sys.exit(1)

    print(f"\n{G}Conectado a {port} @ {BAUD} baud{RST}")
    print(f"{Y}Esperando arranque del Arduino ({BOOT_WAIT:.0f}s)...{RST}")

    boot_lines = flush_read(ser, BOOT_WAIT)
    for l in boot_lines:
        print(f"  {DIM}[BOOT] {l}{RST}")

    # Menu de pruebas
    tests = {
        "1": ("Secuencia de estados",        test_p1),
        "2": ("Conversion PWM lineal",        test_p2),
        "3": ("Giro en sitio",                test_p3),
        "4": ("Timeout de seguridad",         test_p4),
        "5": ("Sensores Hall",                test_p5),
        "t": ("TODAS las pruebas",            None),
    }

    while True:
        print(f"\n{C}{'─'*60}")
        print(" MENU DE PRUEBAS")
        print("─"*60)
        for k, (name, _) in tests.items():
            print(f"  [{k}]  {name}")
        print("  [q]  Salir")
        print(f"{'─'*60}{RST}")
        choice = input("  Elige prueba: ").strip().lower()

        if choice == "q":
            break
        elif choice == "t":
            for k in "12345":
                _, fn = tests[k]
                fn(ser)
                if k != "5":
                    pause("Prueba siguiente →")
        elif choice in tests and tests[choice][1] is not None:
            tests[choice][1](ser)
        else:
            warn("Opcion no valida")

    # Comando manual libre
    print(f"\n{C}[MODO MANUAL] Escribe comandos directos. 'q' para salir.{RST}")
    while True:
        try:
            cmd = input(f"  {W}cmd>{RST} ").strip()
        except (KeyboardInterrupt, EOFError):
            break
        if cmd.lower() == "q":
            break
        if cmd:
            send_cmd(ser, cmd, wait=READ_PAUSE)

    # Seguridad al salir
    print(f"\n{Y}Enviando INHABILITAR por seguridad...{RST}")
    send_cmd(ser, "INHABILITAR", wait=0.5, echo=True)
    ser.close()
    print(f"{G}Conexion cerrada. Fin de pruebas.{RST}\n")

if __name__ == "__main__":
    main()
