"""
test_hall_direcciones.py
Smart Golf Trolley — MOTOR-INTERFACE-V14

Prueba sistemática: cada motor por separado y juntos, en ambas direcciones.
Captura conteo de pulsos Hall por fase para análisis de simetría.

Cálculo de combinaciones v+w para aislar cada motor:
  Motor IZQ solo:   v=+0.5, w=+1.22  →  v_L=+1.0  v_R≈0
  Motor IZQ atras:  v=-0.5, w=-1.22  →  v_L=-1.0  v_R≈0
  Motor DER solo:   v=+0.5, w=-1.22  →  v_L≈0     v_R=+1.0
  Motor DER atras:  v=-0.5, w=+1.22  →  v_L≈0     v_R=-1.0
  Ambos adelante:   v=+0.5, w=0
  Ambos atras:      v=-0.5, w=0
"""

import serial, time, sys

PORT    = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD    = 115200
DURACION = 3.0   # segundos por fase
PWM_TARGET = 60  # velocidad lineal en cm que da ~60 PWM

# ── Colores ANSI ──────────────────────────────────────────────────────────────
G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"
C = "\033[96m"; W = "\033[97m"; D = "\033[2m"; X = "\033[0m"

# ── Utilidades ────────────────────────────────────────────────────────────────
def flush(ser, secs=0.5):
    time.sleep(secs)
    while ser.in_waiting:
        ser.readline()

def cmd(ser, text, wait=0.5, show=True):
    if show: print(f"  {C}>> {text}{X}")
    ser.write((text + "\n").encode())
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        l = ser.readline().decode("utf-8", errors="replace").strip()
        if l: lines.append(l)
    for l in lines:
        print(f"  {D}<< {l}{X}")
    return lines

def sep(title):
    print(f"\n{W}{'═'*60}")
    print(f"  {title}")
    print(f"{'═'*60}{X}")

def leer_encoders(ser):
    """Devuelve (L_total, R_total) del comando e"""
    ser.write(b"e\n")
    time.sleep(0.4)
    while ser.in_waiting:
        l = ser.readline().decode("utf-8", errors="replace").strip()
        if l.startswith("e "):
            parts = l.split()
            if len(parts) >= 3:
                try:
                    return int(parts[1]), int(parts[2])
                except:
                    pass
    return 0, 0

def resetear_encoders(ser):
    ser.write(b"r\n"); time.sleep(0.4)
    while ser.in_waiting: ser.readline()

def correr_fase(ser, nombre, vcmd, wcmd, duracion=DURACION):
    """
    Corre una fase de movimiento y recoge estadísticas.
    vcmd/wcmd: velocidad lineal/angular en m/s y rad/s.
    Devuelve dict con max_lrpm, max_rrpm, l_pulses, r_pulses, l_ma_max, r_ma_max.
    """
    sep(nombre)
    print(f"  {Y}Comando: v {vcmd:.3f} {wcmd:.3f}  |  Duración: {duracion:.0f}s{X}")

    resetear_encoders(ser)

    cmd_str = f"v {vcmd:.3f} {wcmd:.3f}"
    ser.write((cmd_str + "\n").encode())
    time.sleep(0.05)

    max_lrpm = max_rrpm = 0
    max_lma = max_rma = 0.0
    lpwm_list = []
    rpwm_list = []
    n_T = 0

    t0 = time.time()
    while time.time() - t0 < duracion:
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            parts = {}
            for tok in line.split():
                if "=" in tok:
                    k, _, v = tok.partition("=")
                    parts[k] = v

            if line.startswith("T "):
                n_T += 1
                try:
                    lrpm = int(float(parts.get("Lrpm", "0")))
                    rrpm = int(float(parts.get("Rrpm", "0")))
                    lma  = abs(float(parts.get("LmA", "0")))
                    rma  = abs(float(parts.get("RmA", "0")))
                    lp   = int(parts.get("Lpwm", "0"))
                    rp   = int(parts.get("Rpwm", "0"))
                    if lrpm > max_lrpm: max_lrpm = lrpm
                    if rrpm > max_rrpm: max_rrpm = rrpm
                    if lma  > max_lma:  max_lma  = lma
                    if rma  > max_rma:  max_rma  = rma
                    if lp > 0: lpwm_list.append(lp)
                    if rp > 0: rpwm_list.append(rp)
                except:
                    pass
            elif line.startswith("B "):
                try:
                    lrpm = int(float(parts.get("Lrpm", "0")))
                    rrpm = int(float(parts.get("Rrpm", "0")))
                    lma  = abs(float(parts.get("LmA", "0")))
                    rma  = abs(float(parts.get("RmA", "0")))
                    if lrpm > max_lrpm: max_lrpm = lrpm
                    if rrpm > max_rrpm: max_rrpm = rrpm
                    if lma  > max_lma:  max_lma  = lma
                    if rma  > max_rma:  max_rma  = rma
                except:
                    pass

    # Parar motores
    ser.write(b"v 0.0 0.0\n")
    time.sleep(0.5)
    # Leer encoders acumulados
    l_pulses, r_pulses = leer_encoders(ser)
    time.sleep(0.3)

    avg_lpwm = sum(lpwm_list)//len(lpwm_list) if lpwm_list else 0
    avg_rpwm = sum(rpwm_list)//len(rpwm_list) if rpwm_list else 0

    print(f"\n  {'Métrica':<22} {'IZQ':>8} {'DER':>8}")
    print(f"  {'─'*40}")
    print(f"  {'PWM promedio (T line)':<22} {avg_lpwm:>8} {avg_rpwm:>8}")
    print(f"  {'RPM máximo':<22} {max_lrpm:>8} {max_rrpm:>8}")
    print(f"  {'Corriente máx (A)':<22} {max_lma:>8.2f} {max_rma:>8.2f}")
    print(f"  {'Pulsos Hall total':<22} {l_pulses:>8} {r_pulses:>8}")

    # Diagnóstico automático
    if l_pulses == 0 and max_lrpm == 0 and max_lma < 0.05:
        print(f"  {R}  ✗ IZQ: no giró (0 pulsos, 0 RPM, corriente≈0){X}")
    elif l_pulses < 10:
        print(f"  {Y}  ⚠ IZQ: giro débil ({l_pulses} pulsos){X}")
    else:
        print(f"  {G}  ✓ IZQ: {l_pulses} pulsos{X}")

    if r_pulses == 0 and max_rrpm == 0 and max_rma < 0.05:
        print(f"  {R}  ✗ DER: no giró (0 pulsos, 0 RPM, corriente≈0){X}")
    elif r_pulses < 10:
        print(f"  {Y}  ⚠ DER: giro débil ({r_pulses} pulsos){X}")
    else:
        print(f"  {G}  ✓ DER: {r_pulses} pulsos{X}")

    return {
        "nombre":    nombre,
        "l_pulses":  l_pulses,
        "r_pulses":  r_pulses,
        "max_lrpm":  max_lrpm,
        "max_rrpm":  max_rrpm,
        "max_lma":   max_lma,
        "max_rma":   max_rma,
        "avg_lpwm":  avg_lpwm,
        "avg_rpwm":  avg_rpwm,
    }

# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
print(f"\n{W}{'═'*60}")
print("  TEST HALL DIRECCIONES — MOTOR-INTERFACE-V14")
print("  Análisis de pulsos por motor y dirección")
print(f"{'═'*60}{X}")

# Conexión
try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except Exception as e:
    print(f"{R}ERROR: {e}{X}"); sys.exit(1)

print(f"\n{G}Conectado a {PORT}. Esperando boot (5s)...{X}")
flush(ser, 5.0)

# Inicialización
cmd(ser, "hb off",    wait=0.5)  # balance OFF — prueba con PID de velocidad puro
cmd(ser, "HABILITAR", wait=0.5)

input(f"\n  {Y}Robot en posición segura — ENTER para comenzar...{X}")

resultados = []

# ── 1. IZQ adelante (DER parado) ─────────────────────────────────────────────
# v=0.5, w=1.22 → v_L=0.5+1.22*0.41≈1.0, v_R=0.5-0.5=0.0
resultados.append(correr_fase(ser, "1. IZQ ADELANTE (DER parado)",  0.5,  1.22))
time.sleep(1.0)

# ── 2. IZQ atrás (DER parado) ─────────────────────────────────────────────────
# v=-0.5, w=-1.22 → v_L=-1.0, v_R≈0.0
resultados.append(correr_fase(ser, "2. IZQ ATRAS  (DER parado)", -0.5, -1.22))
time.sleep(1.0)

# ── 3. DER adelante (IZQ parado) ─────────────────────────────────────────────
# v=0.5, w=-1.22 → v_L≈0, v_R=1.0
resultados.append(correr_fase(ser, "3. DER ADELANTE (IZQ parado)",  0.5, -1.22))
time.sleep(1.0)

# ── 4. DER atrás (IZQ parado) ────────────────────────────────────────────────
# v=-0.5, w=1.22 → v_L≈0, v_R=-1.0
resultados.append(correr_fase(ser, "4. DER ATRAS  (IZQ parado)", -0.5,  1.22))
time.sleep(1.0)

# ── 5. AMBOS adelante ────────────────────────────────────────────────────────
resultados.append(correr_fase(ser, "5. AMBOS ADELANTE",  0.5,  0.0))
time.sleep(1.0)

# ── 6. AMBOS atrás ───────────────────────────────────────────────────────────
resultados.append(correr_fase(ser, "6. AMBOS ATRAS",    -0.5,  0.0))
time.sleep(1.0)

# Parar y deshabilitar
cmd(ser, "INHABILITAR", wait=0.4, show=False)
ser.close()

# ══════════════════════════════════════════════════════════════════════════════
# RESUMEN FINAL
# ══════════════════════════════════════════════════════════════════════════════
print(f"\n\n{W}{'═'*60}")
print("  RESUMEN COMPLETO — PULSOS HALL POR FASE")
print(f"{'═'*60}{X}")
print(f"\n  {'Fase':<32} {'Lpwm':>6} {'Rpwm':>6} {'Lrpm':>6} {'Rrpm':>6} {'L_pulsos':>9} {'R_pulsos':>9}")
print(f"  {'─'*80}")
for r in resultados:
    print(f"  {r['nombre']:<32} {r['avg_lpwm']:>6} {r['avg_rpwm']:>6} "
          f"{r['max_lrpm']:>6} {r['max_rrpm']:>6} {r['l_pulses']:>9} {r['r_pulses']:>9}")

print(f"\n{W}Análisis:{X}")
# Ratio de pulsos IZQ vs DER en marcha recta
fwd = next((r for r in resultados if "AMBOS ADELANTE" in r['nombre']), None)
bwd = next((r for r in resultados if "AMBOS ATRAS"    in r['nombre']), None)
if fwd and fwd['r_pulses'] > 0:
    ratio_fwd = fwd['l_pulses'] / fwd['r_pulses']
    print(f"  Ratio pulsos L/R en FWD: {ratio_fwd:.2f}  (1.0 = perfecto, >1 = IZQ más rápido)")
if bwd and bwd['r_pulses'] > 0:
    ratio_bwd = bwd['l_pulses'] / bwd['r_pulses']
    print(f"  Ratio pulsos L/R en BWD: {ratio_bwd:.2f}")

izq_fwd = next((r for r in resultados if "IZQ ADELANTE" in r['nombre']), None)
izq_bwd = next((r for r in resultados if "IZQ ATRAS"    in r['nombre']), None)
der_fwd = next((r for r in resultados if "DER ADELANTE" in r['nombre']), None)
der_bwd = next((r for r in resultados if "DER ATRAS"    in r['nombre']), None)
if izq_fwd and izq_bwd and izq_bwd['l_pulses'] > 0:
    print(f"  IZQ FWD/BWD ratio: {izq_fwd['l_pulses']}/{izq_bwd['l_pulses']} = {izq_fwd['l_pulses']/izq_bwd['l_pulses']:.2f}")
if der_fwd and der_bwd and der_bwd['r_pulses'] > 0:
    print(f"  DER FWD/BWD ratio: {der_fwd['r_pulses']}/{der_bwd['r_pulses']} = {der_fwd['r_pulses']/der_bwd['r_pulses']:.2f}")

print(f"\n{G}Prueba completada.{X}\n")
