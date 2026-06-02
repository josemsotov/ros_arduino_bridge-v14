"""
calibrate_kpv.py
================
Calibración automática de Kp_v para el Smart Golf Trolley.

FASES:
  1. Medición de planta open-loop: comando 'p <pwm>' aplica PWM directo sin PID.
     Mide velocidad real de cada motor en m/s por PWM unit.

  2. Barrido de Kp_v: para cada candidato, envía 'v 0.5' re-enviando cada 0.8s
     durante 5 ciclos, registra los PWM aplicados a cada rueda y calcula la
     varianza (menor varianza = más estable).

  3. Recomienda el Kp_v óptimo y lo guarda como valor por defecto.

Requiere: pyserial  (pip install pyserial)
Arduino debe estar grabado con firmware MOTOR-INTERFACE-V14 actualizado.
"""

import serial
import serial.tools.list_ports
import time
import math
import sys

# ── Colores ────────────────────────────────────────────────────────────────
R  = "\033[91m"; G  = "\033[92m"; Y  = "\033[93m"
B  = "\033[94m"; M  = "\033[95m"; C  = "\033[96m"; W  = "\033[0m"

# ── Parámetros del robot ────────────────────────────────────────────────────
PPR             = 45      # pulsos por revolución Hall
WHEEL_DIAM_M    = 0.20    # diámetro rueda (m)
WHEEL_CIRC_M    = math.pi * WHEEL_DIAM_M  # circunferencia = 0.6283 m
OPEN_LOOP_TIME  = 2.0     # segundos en cada nivel PWM open-loop
STEP_VELOCITY   = 0.5     # m/s usado para el barrido Kp_v
STEP_CYCLES     = 5       # número de re-envíos del comando v por Kp candidato
RESEND_INTERVAL = 0.8     # s entre re-envíos del comando v
KPV_CANDIDATES  = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
OPEN_LOOP_PWMS  = [30, 50, 70]   # niveles para medir planta


def find_arduino():
    for p in serial.tools.list_ports.comports():
        if "CH340" in p.description or "Arduino" in p.description or "USB-SERIAL" in p.description.upper():
            return p.device
    ports = serial.tools.list_ports.comports()
    if ports:
        return ports[0].device
    return None


def send(ser: serial.Serial, cmd: str, wait=0.5, echo=True) -> list[str]:
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    if echo:
        print(f"  {B}>>{W} {cmd}")
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            lines.append(line)
            if echo:
                print(f"  {C}<<{W} {line}")
    return lines


def pulses_to_ms(pulses: int, elapsed_s: float) -> float:
    """Convierte pulsos Hall en m/s dado el tiempo transcurrido."""
    if elapsed_s <= 0 or pulses == 0:
        return 0.0
    rpm = (pulses / PPR) * (60.0 / elapsed_s)
    return rpm * WHEEL_CIRC_M / 60.0


def header(text):
    print(f"\n{W}{'─'*60}")
    print(f" {M}{text}{W}")
    print(f"{'─'*60}")


def info(text):  print(f"  {Y}ℹ{W} {text}")
def ok(text):    print(f"  {G}✓{W} {text}")
def warn(text):  print(f"  {Y}⚠{W} {text}")
def fail(text):  print(f"  {R}✗{W} {text}")


# ── FASE 1: Medición de planta open-loop ────────────────────────────────────
def measure_plant(ser: serial.Serial) -> dict:
    """
    Aplica PWM directo a ambos motores con el comando 'p <pwm>',
    espera OPEN_LOOP_TIME segundos midiendo pulsos Hall, y devuelve
    el diccionario {pwm: (v_left_ms, v_right_ms)}.
    """
    header("FASE 1 — Medición de planta (open-loop)")
    info("Comando 'p <pwm>' aplica PWM directo sin PID, ambos motores FWD.")
    info(f"Niveles: {OPEN_LOOP_PWMS} — {OPEN_LOOP_TIME}s por nivel")

    results = {}

    # Asegura robot habilitado
    send(ser, "HABILITAR", wait=0.3, echo=False)

    for pwm in OPEN_LOOP_PWMS:
        # Reset counters
        send(ser, "r", wait=0.3, echo=False)
        time.sleep(0.1)

        # Aplicar PWM
        resp = send(ser, f"p {pwm}", wait=0.2)
        if not any("p OK" in l for l in resp):
            warn(f"PWM={pwm}: sin confirmación del firmware, continuando...")

        t0 = time.time()
        time.sleep(OPEN_LOOP_TIME)
        elapsed = time.time() - t0

        # Leer contadores
        resp = send(ser, "e", wait=0.4, echo=False)
        l_pulses, r_pulses = 0, 0
        for line in resp:
            if line.startswith("e "):
                parts = line.split()
                if len(parts) == 3:
                    try:
                        l_pulses = int(parts[1])
                        r_pulses = int(parts[2])
                    except ValueError:
                        pass

        v_left  = pulses_to_ms(l_pulses, elapsed)
        v_right = pulses_to_ms(r_pulses, elapsed)

        results[pwm] = (v_left, v_right)
        ok(f"PWM={pwm:2d} → Left={l_pulses:4d} pulsos ({v_left:.3f} m/s)  "
           f"Right={r_pulses:4d} pulsos ({v_right:.3f} m/s)  [t={elapsed:.2f}s]")

        # Parar entre niveles
        send(ser, "p", wait=0.3, echo=False)  # p sin arg = STOP
        time.sleep(0.5)

    send(ser, "INHABILITAR", wait=0.3, echo=False)
    return results


def compute_plant_gain(plant: dict) -> tuple[float, float]:
    """
    Regresión lineal simple V = K * PWM para left y right.
    Retorna (K_left, K_right) en m/s/PWM.
    """
    pwms   = list(plant.keys())
    v_left  = [plant[p][0] for p in pwms]
    v_right = [plant[p][1] for p in pwms]

    def linreg(xs, ys):
        n = len(xs)
        sx = sum(xs); sy = sum(ys)
        sxy = sum(x*y for x,y in zip(xs, ys))
        sxx = sum(x*x for x in xs)
        denom = n*sxx - sx*sx
        if denom == 0:
            return 0.0
        return (n*sxy - sx*sy) / denom

    K_L = linreg(pwms, v_left)
    K_R = linreg(pwms, v_right)
    return K_L, K_R


# ── FASE 2: Barrido de Kp_v ─────────────────────────────────────────────────
def sweep_kpv(ser: serial.Serial, candidates: list[float]) -> dict:
    """
    Para cada Kp_v candidato:
      - Setea con 'k <kp>'
      - Envía 'v <STEP_VELOCITY> 0.0' STEP_CYCLES veces cada RESEND_INTERVAL s
      - Registra los PWM de Left y Right de cada respuesta
      - Calcula la varianza del PWM (indica oscilación / inestabilidad)
    Retorna {kp: {'pwm_left': [...], 'pwm_right': [...], 'var': float}}
    """
    header("FASE 2 — Barrido de Kp_v")
    info(f"Velocidad de prueba: v={STEP_VELOCITY} m/s")
    info(f"Candidatos: {candidates}")
    info("Menor varianza de PWM = respuesta más estable")

    import re
    results = {}

    for kp in candidates:
        print(f"\n  {Y}Kp_v = {kp:.3f}{W}")

        # Setear Kp_v
        resp = send(ser, f"k {kp}", wait=0.3, echo=False)
        confirmed = any(f"Kp_v=" in l for l in resp)
        if not confirmed:
            warn(f"Sin confirmación de k {kp}")

        # Reset Hall
        send(ser, "r", wait=0.3, echo=False)
        # Habilitar robot
        send(ser, "HABILITAR", wait=0.3, echo=False)

        pwm_left_list  = []
        pwm_right_list = []

        for cycle in range(STEP_CYCLES):
            resp = send(ser, f"v {STEP_VELOCITY} 0.0", wait=RESEND_INTERVAL, echo=False)
            for line in resp:
                m = re.search(r"Left PWM=(-?\d+).*?Right PWM=(-?\d+)", line)
                if m:
                    pwm_left_list.append(int(m.group(1)))
                    pwm_right_list.append(int(m.group(2)))

        # Parar
        send(ser, "v 0.0 0.0", wait=0.3, echo=False)
        send(ser, "INHABILITAR", wait=0.3, echo=False)
        time.sleep(0.5)

        # Calcular varianza
        def variance(lst):
            if len(lst) < 2:
                return 9999.0
            mean = sum(lst) / len(lst)
            return sum((x - mean)**2 for x in lst) / len(lst)

        var_L = variance(pwm_left_list)
        var_R = variance(pwm_right_list)
        total_var = var_L + var_R

        results[kp] = {
            'pwm_left':  pwm_left_list,
            'pwm_right': pwm_right_list,
            'var_L':     var_L,
            'var_R':     var_R,
            'var_total': total_var,
        }

        mean_L = sum(pwm_left_list) / max(len(pwm_left_list), 1)
        mean_R = sum(pwm_right_list) / max(len(pwm_right_list), 1)

        color = G if total_var < 100 else (Y if total_var < 500 else R)
        print(f"  {color}Kp={kp:.3f} | Left  PWMs={pwm_left_list} mean={mean_L:.1f} var={var_L:.1f}")
        print(f"           | Right PWMs={pwm_right_list} mean={mean_R:.1f} var={var_R:.1f}")
        print(f"           | Varianza total={total_var:.1f}{W}")

    return results


# ── FASE 3: Elegir y aplicar el mejor Kp_v ──────────────────────────────────
def choose_best_kpv(sweep_results: dict, K_L: float, K_R: float) -> float:
    header("FASE 3 — Análisis y recomendación")

    # Recomendación teórica (loop gain = 0.5 para critically-damped):
    # pwm = Kp * error * 100  →  loop_gain = Kp * 100 * K
    # Queremos loop_gain ≈ 0.5  →  Kp = 0.5 / (100 * K)
    K_avg = (K_L + K_R) / 2.0 if K_L > 0 and K_R > 0 else max(K_L, K_R)
    if K_avg > 0:
        kp_theory = 0.5 / (100.0 * K_avg)
        info(f"Ganancia planta: K_left={K_L:.5f}  K_right={K_R:.5f}  K_avg={K_avg:.5f} m/s/PWM")
        info(f"Kp_v teórico (loop_gain=0.5): {kp_theory:.4f}")
    else:
        kp_theory = None
        warn("No se pudo calcular ganancia de planta (datos insuficientes)")

    # Mejor según varianza mínima del barrido
    if sweep_results:
        best_kp = min(sweep_results, key=lambda k: sweep_results[k]['var_total'])
        best_var = sweep_results[best_kp]['var_total']
        ok(f"Menor varianza en barrido: Kp_v={best_kp:.3f}  (var={best_var:.1f})")
    else:
        best_kp = kp_theory or 0.1

    # Tabla resumen
    print(f"\n  {'Kp_v':>6}  {'Var_Left':>10}  {'Var_Right':>10}  {'Var_Total':>10}  {'PWMs Left':>30}")
    print(f"  {'─'*80}")
    for kp in sorted(sweep_results.keys()):
        r = sweep_results[kp]
        marker = " ← MEJOR" if kp == best_kp else ""
        print(f"  {kp:6.3f}  {r['var_L']:10.1f}  {r['var_R']:10.1f}  {r['var_total']:10.1f}  "
              f"{str(r['pwm_left']):>30}{marker}")

    # Recomendación final
    print()
    if kp_theory:
        info(f"Recomendación teórica:  Kp_v = {kp_theory:.4f}")
    ok(f"Recomendación empírica: Kp_v = {best_kp:.3f}")

    return best_kp


def apply_and_verify(ser: serial.Serial, kp: float):
    header(f"APLICANDO Kp_v = {kp:.4f}")
    send(ser, f"k {kp}", wait=0.3)

    info("Verificando con v=0.5 por 3s (re-envío cada 0.8s)...")
    send(ser, "HABILITAR", wait=0.3, echo=False)
    t_end = time.time() + 3.0
    send(ser, f"v {STEP_VELOCITY} 0.0", wait=0.1)
    while time.time() < t_end:
        time.sleep(0.8)
        send(ser, f"v {STEP_VELOCITY} 0.0", wait=0.1)
    send(ser, "v 0.0 0.0", wait=0.3, echo=False)
    send(ser, "INHABILITAR", wait=0.3, echo=False)

    ok(f"Kp_v={kp:.4f} activo en memoria RAM del Arduino.")
    warn("Para que sea permanente, actualiza 'Kp_v' en pid_control.h y re-flashea.")


def patch_kpv_in_firmware(kp: float):
    """Actualiza el valor en pid_control.h."""
    import re
    path = r"C:\Users\josem\Desktop\MOTOR-INTERFACE-V14\pid_control.h"
    try:
        with open(path, "r") as f:
            content = f.read()
        new_content = re.sub(
            r"(float\s+Kp_v\s*=\s*)[0-9.]+",
            f"\\g<1>{kp:.4f}",
            content
        )
        if new_content != content:
            with open(path, "w") as f:
                f.write(new_content)
            print(f"\n  {G}✓{W} pid_control.h actualizado: Kp_v = {kp:.4f}")
            return True
        else:
            warn("No se encontró el patrón 'float Kp_v = ...' en pid_control.h")
            return False
    except Exception as e:
        fail(f"Error actualizando pid_control.h: {e}")
        return False


# ── MAIN ────────────────────────────────────────────────────────────────────
def main():
    print(f"\n{W}{'='*60}")
    print("  SMART GOLF TROLLEY — CALIBRACIÓN Kp_v")
    print("  MOTOR-INTERFACE-V14  |  Arduino Mega 2560")
    print(f"{'='*60}{W}")

    port = find_arduino()
    if port is None:
        fail("No se encontró Arduino. Conecta el USB.")
        sys.exit(1)

    print(f"\n  Puerto: {G}{port}{W}")
    input("  Pulsa ENTER para conectar y empezar...")

    ser = serial.Serial(port, 115200, timeout=1.0)
    time.sleep(3.0)  # boot Arduino
    ser.reset_input_buffer()

    print(f"\n  {G}Conectado @ 115200 baud{W}")

    try:
        # ── FASE 1: Planta ──────────────────────────────────────────────────
        plant = measure_plant(ser)
        K_L, K_R = compute_plant_gain(plant)

        print(f"\n  {M}Ganancia de planta:{W}")
        print(f"    Left:  K = {K_L:.5f} m/s/PWM")
        print(f"    Right: K = {K_R:.5f} m/s/PWM")

        if K_L < 0.0001 and K_R < 0.0001:
            fail("Ambos motores sin movimiento detectado. Verifica sensores Hall y cableado.")
            ser.close()
            sys.exit(1)

        # ── FASE 2: Barrido ─────────────────────────────────────────────────
        sweep = sweep_kpv(ser, KPV_CANDIDATES)

        # ── FASE 3: Análisis ────────────────────────────────────────────────
        best_kp = choose_best_kpv(sweep, K_L, K_R)

        # ── APLICAR ─────────────────────────────────────────────────────────
        print()
        resp = input(f"  ¿Aplicar Kp_v={best_kp:.3f} y guardar en pid_control.h? [s/n] ").strip().lower()
        if resp == "s":
            apply_and_verify(ser, best_kp)
            if patch_kpv_in_firmware(best_kp):
                print(f"\n  {Y}Ahora ejecuta el flash para que el valor sea permanente:{W}")
                print(f"  arduino-cli compile --fqbn arduino:avr:mega "
                      f"\"C:\\Users\\josem\\Desktop\\MOTOR-INTERFACE-V14\" --upload --port {port}")
        else:
            # Aplicar solo en RAM
            resp2 = input(f"  ¿Quieres probar un Kp_v personalizado? (vacío = no): ").strip()
            if resp2:
                try:
                    custom_kp = float(resp2)
                    apply_and_verify(ser, custom_kp)
                except ValueError:
                    warn("Valor no válido, sin cambios.")

    finally:
        send(ser, "INHABILITAR", wait=0.3, echo=False)
        ser.close()
        print(f"\n  {W}Conexión cerrada.{W}")


if __name__ == "__main__":
    main()
