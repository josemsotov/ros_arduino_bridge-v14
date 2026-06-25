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
def find_arduino(forced=None):
    """Devuelve el puerto COM del Arduino.
    Si forced != None, verifica que ese puerto exista y lo usa.
    De lo contrario busca por descripcion del device.
    Uso: python test_control_suite.py COM4
    """
    if forced:
        forced_upper = forced.upper()
        available = [p.device.upper() for p in serial.tools.list_ports.comports()]
        # Tambien acepta COM4 aunque no aparezca en la lista (puede estar en Device Manager pero no en pyserial)
        print(f"  Usando puerto forzado: {forced_upper}")
        return forced_upper
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

# ─── PRUEBA 6: Motor básico directo — ADELANTE / ATRAS ──────────────────────
def test_p6(ser):
    header(6, "Motor básico — ADELANTE / ATRAS / L / R")
    info("Verifica que los motores responden a comandos directos de movimiento.")
    warn("Observa físicamente: el robot debe moverse ADELANTE, luego ATRÁS.")

    send_cmd(ser, "HABILITAR")

    info("--- ADELANTE a PWM=30 ---")
    send_cmd(ser, "AD 30")
    time.sleep(2.0)

    resp = send_cmd(ser, "MOTOR_STATUS")
    fwd = any("FWD" in l for l in resp)
    pwm_ok = any(("PWM:" in l and ":0" not in l) for l in resp)
    if fwd and pwm_ok:
        ok("ADELANTE confirmado: FWD + PWM > 0  ✓")
    elif fwd:
        ok("Dirección FWD confirmada (PWM no visible en respuesta)")
    else:
        warn("Verificar físicamente si el robot avanzó")

    send_cmd(ser, "STOP", wait=0.3, echo=False)
    time.sleep(0.5)

    info("--- ATRAS a PWM=30 ---")
    send_cmd(ser, "AT 30")
    time.sleep(2.0)

    resp = send_cmd(ser, "MOTOR_STATUS")
    bwd = any("BWD" in l for l in resp)
    if bwd:
        ok("ATRAS confirmado: BWD  ✓")
    else:
        warn("Verificar físicamente si el robot retrocedió")

    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)

    info("--- Motor izquierdo solo: L 25 ---")
    send_cmd(ser, "HABILITAR", wait=0.3, echo=False)
    send_cmd(ser, "L 25")
    time.sleep(1.5)
    send_cmd(ser, "L 0", wait=0.3, echo=False)

    info("--- Motor derecho solo: R 25 ---")
    send_cmd(ser, "R 25")
    time.sleep(1.5)
    send_cmd(ser, "R 0", wait=0.3, echo=False)
    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)
    ok("Prueba motores básicos completada — verificar observación física")


# ─── PRUEBA 7: MPU9250 IMU ────────────────────────────────────────────────────
def test_p7(ser):
    header(7, "MPU9250 — IMU activo y leyendo pitch/gyro")
    info("Comando: 'hb stat' — muestra estado del balance y lecturas del IMU.")
    info("Predicción: [BAL] MPU: OK | Pitch abs: XX.XX°")

    resp = send_cmd(ser, "hb stat", wait=READ_PAUSE_LONG)

    mpu_ok      = any("MPU" in l and "OK" in l for l in resp)
    pitch_found = any("Pitch abs" in l for l in resp)
    gyro_found  = any("GyroY" in l for l in resp)

    if mpu_ok:
        ok("MPU detectado: [BAL] MPU: OK  ✓")
    else:
        fail("MPU no detectado — revisar I2C (SDA=pin20, SCL=pin21, addr=0x68)")

    if pitch_found:
        for l in resp:
            if "Pitch abs" in l:
                ok(f"Pitch leído: {l.strip()}  ✓")
                break
    else:
        warn("Pitch no visible — puede que MPU no esté listo aún")

    if gyro_found:
        for l in resp:
            if "GyroY" in l:
                ok(f"GyroY leído: {l.strip()}  ✓")
                break
    else:
        warn("GyroY no visible en respuesta")


# ─── PRUEBA 8: Hoverboard / Balance anti-caída ───────────────────────────────
def test_p8(ser):
    header(8, "Hoverboard — activar / desactivar / calibrar balance")
    info("Comandos: hb off → hb on → hb cal")
    warn("El robot debe estar QUIETO y NIVELADO para 'hb cal'.")

    resp = send_cmd(ser, "hb off", wait=READ_PAUSE)
    if any("desactivad" in l.lower() or "off" in l.lower() for l in resp):
        ok("hb off: balance desactivado  ✓")
    else:
        warn("Respuesta de hb off no reconocida — ver arriba")

    resp = send_cmd(ser, "hb on", wait=READ_PAUSE)
    if any("activad" in l.lower() or " on" in l.lower() for l in resp):
        ok("hb on: balance reactivado  ✓")
    else:
        warn("Respuesta de hb on no reconocida — ver arriba")

    info("Recalibrando neutral (robot quieto y nivelado)...")
    resp = send_cmd(ser, "hb cal", wait=READ_PAUSE_LONG)
    if any("etpoint" in l or "calibrad" in l.lower() for l in resp):
        for l in resp:
            if "etpoint" in l or "calibrad" in l.lower():
                ok(f"Calibración OK: {l.strip()}  ✓")
                break
    else:
        warn("Respuesta de hb cal no reconocida — ver arriba")

    # Verificar estado final
    resp = send_cmd(ser, "hb stat", wait=READ_PAUSE)
    active = any("Activo" in l and ("SÍ" in l or "SI" in l) for l in resp)
    fallen = any("Caído" in l and "SÍ" in l for l in resp)

    if active:
        ok("Balance activo tras calibración  ✓")
    else:
        warn("Balance no activo — revisar MPU")

    if fallen:
        fail("Estado CAÍDO activo — usar 'hb on' para reactivar")
    else:
        ok("Estado CAÍDO = NO  ✓")


# ─── PRUEBA 9: ROS2 Bridge — encoders, status y velocidad ───────────────────
def test_p9(ser):
    header(9, "ROS2 Bridge — protocolo e/r/s/v completo")
    info("Protocolo: r=reset | e=encoders | s=status | v=cmd_vel")

    resp = send_cmd(ser, "r")
    ok("Reset encoders enviado") if resp is not None else warn("Sin respuesta a 'r'")

    resp = send_cmd(ser, "e")
    if any(l.startswith("e ") for l in resp):
        for l in resp:
            if l.startswith("e "):
                ok(f"Encoders tras reset: {l}  ✓")
                break
    else:
        warn("Respuesta 'e' no reconocida — ver arriba")

    resp = send_cmd(ser, "s")
    if any(l.startswith("s ") for l in resp):
        for l in resp:
            if l.startswith("s "):
                code = l.strip()
                if "s 0" in code:
                    ok(f"Status = OK (0)  ✓")
                elif "s 1" in code:
                    warn(f"Status = ERROR (1) — revisar sistema")
                else:
                    info(f"Status recibido: {code}")
                break
    else:
        warn("Respuesta 's' no reconocida — ver arriba")

    info("--- Velocidad: v 0.3 0.0 durante 1.5s, luego leer encoders ---")
    send_cmd(ser, "v 0.3 0.0")
    time.sleep(1.5)

    resp = send_cmd(ser, "e")
    l_enc, r_enc = None, None
    for l in resp:
        if l.startswith("e "):
            import re
            m = re.search(r"e (-?\d+) (-?\d+)", l)
            if m:
                l_enc, r_enc = int(m.group(1)), int(m.group(2))
    if l_enc is not None:
        if l_enc > 0 or r_enc > 0:
            ok(f"Encoders tras movimiento: L={l_enc} R={r_enc}  ✓")
        else:
            fail(f"Encoders en 0 tras movimiento — revisar Hall sensors")
    else:
        warn("No se pudo parsear respuesta de encoders")

    send_cmd(ser, "v 0.0 0.0", wait=0.3, echo=False)
    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)

    info("--- Kp actual (comando k) ---")
    resp = send_cmd(ser, "k")
    for l in resp:
        if "Kp_v" in l:
            ok(f"Parámetros PID: {l.strip()}  ✓")
            break


# ─── PRUEBA 10: Pin status — diagnóstico completo de pines ──────────────────
def test_p10(ser):
    header(10, "Pin status — comando 'z' y SYSTEM_PINS")
    info("Verifica que el firmware reporta correctamente el estado de pines.")
    info("Predicción: z PIN_STATUS_START...PIN_STATUS_END con datos de motores y Hall.")

    resp = send_cmd(ser, "z", wait=READ_PAUSE_LONG)

    start = any("PIN_STATUS_START" in l for l in resp)
    end   = any("PIN_STATUS_END" in l for l in resp)
    left  = any(l.startswith("z L") for l in resp)
    right = any(l.startswith("z R") for l in resp)
    hall  = any(l.startswith("z HALL") for l in resp)

    if start and end:
        ok("PIN_STATUS_START/END recibidos  ✓")
    else:
        warn("Delimitadores no encontrados — ver respuesta arriba")

    if left:
        for l in resp:
            if l.startswith("z L"):
                ok(f"Motor izquierdo: {l.strip()}")
                break

    if right:
        for l in resp:
            if l.startswith("z R"):
                ok(f"Motor derecho: {l.strip()}")
                break

    if hall:
        for l in resp:
            if l.startswith("z HALL"):
                ok(f"Hall sensors: {l.strip()}")
                break

    if not (left and right and hall):
        warn("Algunos bloques de pin status faltantes — verificar firmware")


# ─── PRUEBA 11: Sensores de corriente ACS712 ────────────────────────────────
def test_p11(ser):
    header(11, "Sensores de corriente ACS712 — inicialización y offset")
    info("Los sensores ACS712 se inicializan al boot y reportan offsets.")
    info("Esta prueba verifica el boot log y lee el estado con 'z' durante movimiento.")
    warn("NOTA: No hay comando directo para leer corriente en V14.")
    warn("      Verificar en boot log: '[CURR] Sensores ACS712 inicializados'")
    warn("      Si el boot log no está disponible, reinicar el Arduino.")

    info("--- Boot log esperado (si fue capturado al conectar): ---")
    info("  [CURR] Sensores ACS712 inicializados")
    info("  [CURR] Offset R=XX.X mV  L=XX.X mV")
    info("")
    info("--- Activando motores para observar corriente en movimiento ---")

    send_cmd(ser, "HABILITAR", wait=0.3, echo=False)
    send_cmd(ser, "v 0.3 0.0")
    time.sleep(2.0)

    resp = send_cmd(ser, "z", wait=READ_PAUSE)
    # z muestra L_rpm y R_rpm — si son > 0 los motores están girando
    for l in resp:
        if "L_rpm" in l:
            import re
            m = re.search(r"L_rpm=([\d.]+).*?R_rpm=([\d.]+)", l)
            if m:
                lr, rr = float(m.group(1)), float(m.group(2))
                if lr > 0.0 or rr > 0.0:
                    ok(f"Motores en movimiento: L_rpm={lr:.1f} R_rpm={rr:.1f}  ✓")
                    info("(Con motores girando los ACS712 deben mostrar corriente en boot-log)")
                else:
                    warn(f"RPM = 0 durante movimiento — revisar Hall/corriente")

    send_cmd(ser, "v 0.0 0.0", wait=0.3, echo=False)
    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)
    ok("Prueba ACS712 completada — validar boot log manualmente")


# ─── PRUEBA 12: Giros Hall — CRUCE-DER / CRUCE-IZQ ──────────────────────────
def test_p12(ser):
    header(12, "Giros Hall — CRUCE-DER / CRUCE-IZQ por pulsos")
    info("Giro diferencial controlado por conteo de pulsos Hall.")
    info("Predicción: robot gira un ángulo fijo y para automáticamente.")
    warn("Asegura espacio libre alrededor del robot antes de continuar.")
    pause("Pulsa ENTER cuando el área esté despejada...")

    send_cmd(ser, "HABILITAR", wait=0.3, echo=False)

    info("--- CRUCE-DER 54 20 (54 pulsos, PWM=20, ~giro derecha) ---")
    resp = send_cmd(ser, "CRUCE-DER 54 20", wait=4.0)
    cruce_ok = any("CRUCE" in l.upper() or "pulsos" in l.lower() or "OK" in l for l in resp)
    if cruce_ok:
        ok("CRUCE-DER ejecutado  ✓")
    else:
        warn("Sin confirmación de cruce — verificar físicamente")

    time.sleep(1.5)
    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)
    time.sleep(0.5)
    send_cmd(ser, "HABILITAR", wait=0.3, echo=False)

    info("--- CRUCE-IZQ 54 20 (volver a posición original) ---")
    resp = send_cmd(ser, "CRUCE-IZQ 54 20", wait=4.0)
    cruce_ok = any("CRUCE" in l.upper() or "pulsos" in l.lower() or "OK" in l for l in resp)
    if cruce_ok:
        ok("CRUCE-IZQ ejecutado  ✓")
    else:
        warn("Sin confirmación de cruce — verificar físicamente")

    time.sleep(1.5)
    send_cmd(ser, "INHABILITAR", wait=0.3, echo=False)
    ok("Prueba de giros Hall completada")


# ─── MAIN ────────────────────────────────────────────────────────────────────
def main():
    print(f"\n{W}{'='*60}")
    print("  SMART GOLF TROLLEY — SUITE DE PRUEBAS DE CONTROL")
    print(f"  MOTOR-INTERFACE-V14  |  115200 baud")
    print(f"{'='*60}{RST}")

    # Puerto opcional como argumento: python test_control_suite.py COM4
    forced_port = sys.argv[1] if len(sys.argv) > 1 else None

    # Detectar puerto
    port = find_arduino(forced_port)
    if port is None:
        fail("No se encontro ningun Arduino. Conecta el USB y vuelve a intentar.")
        sys.exit(1)

    if forced_port:
        print(f"\n{G}Puerto forzado: {port}{RST}")
    else:
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
        "1":  ("Secuencia de estados",               test_p1),
        "2":  ("Conversion PWM lineal",               test_p2),
        "3":  ("Giro en sitio",                       test_p3),
        "4":  ("Timeout de seguridad",                test_p4),
        "5":  ("Sensores Hall — contadores",          test_p5),
        "6":  ("Motor básico — ADELANTE/ATRAS/L/R",   test_p6),
        "7":  ("MPU9250 — pitch y gyro",              test_p7),
        "8":  ("Hoverboard — on/off/cal",             test_p8),
        "9":  ("ROS2 Bridge — e/r/s/v/k",            test_p9),
        "10": ("Pin status — comando z",              test_p10),
        "11": ("Corriente ACS712 — offset boot",      test_p11),
        "12": ("Giros Hall — CRUCE-DER/IZQ",         test_p12),
        "t":  ("TODAS las pruebas",                   None),
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
            for k in ["1","2","3","4","5","6","7","8","9","10","11","12"]:
                _, fn = tests[k]
                fn(ser)
                if k != "12":
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
