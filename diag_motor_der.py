"""
diag_motor_der.py — Diagnóstico motor derecho
Envía comandos directo al Arduino sin necesitar el mando Stadia.
Responde: ¿es software o hardware el problema?
"""
import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200

G  = "\033[92m"
R  = "\033[91m"
Y  = "\033[93m"
C  = "\033[96m"
W  = "\033[97m"
D  = "\033[2m"
X  = "\033[0m"

def flush(ser, wait=1.2):
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        l = ser.readline().decode("utf-8", errors="replace").strip()
        if l: lines.append(l)
    return lines

def cmd(ser, text, wait=1.0, show=True):
    if show: print(f"  {C}>> {text}{X}")
    ser.write((text + "\n").encode())
    resp = flush(ser, wait)
    for l in resp:
        print(f"  {D}<< {l}{X}")
    return resp

def sep(title):
    print(f"\n{W}{'─'*55}")
    print(f"  {title}")
    print(f"{'─'*55}{X}")

# ─── Conectar ──────────────────────────────────────────────────────
print(f"\n{W}DIAGNÓSTICO MOTOR DERECHO — {PORT}{X}")
try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except Exception as e:
    print(f"{R}ERROR: {e}{X}"); sys.exit(1)

print(f"{G}Conectado. Esperando boot Arduino (3s)...{X}")
boot = flush(ser, 3.0)
for l in boot:
    print(f"  {D}[BOOT] {l}{X}")

# ─── TEST 1: Estado de pines ────────────────────────────────────────
sep("TEST 1 — Estado de pines (z)")
resp_z = cmd(ser, "z", wait=0.8)

stop_right_mode = None
brake_right_mode = None
for l in resp_z:
    if "z R" in l:
        print(f"\n  {Y}Motor DERECHO: {l}{X}")
        if "STOP_mode=OUTPUT" in l:
            stop_right_mode = "OUTPUT (LOW=DISABLE ← BUG)"
        elif "STOP_mode=INPUT" in l:
            stop_right_mode = "INPUT (FLOAT=ENABLE ✓)"
        if "BRAKE_mode=OUTPUT" in l:
            brake_right_mode = "OUTPUT"
        elif "BRAKE_mode=INPUT" in l:
            brake_right_mode = "INPUT (FLOAT ✓)"

if stop_right_mode:
    print(f"  STOP_RIGHT  modo: {stop_right_mode}")
if brake_right_mode:
    print(f"  BRAKE_RIGHT modo: {brake_right_mode}")

# ─── TEST 2: Habilitar + PWM directo ambos motores ─────────────────
sep("TEST 2 — Raw PWM=60 ambos motores (p 60)")
print(f"  {Y}Observa físicamente si el motor DERECHO gira.{X}")
cmd(ser, "hb off", wait=0.5)        # desactivar balance anti-caída ANTES de prueba directa
cmd(ser, "HABILITAR", wait=0.5)
resp_p = cmd(ser, "p 60", wait=0.3)
for l in resp_p:
    if "p OK" in l:
        print(f"  {G}✓ Firmware confirmó PWM aplicado{X}")

print(f"  {Y}⏳ Motores corriendo 3 segundos...{X}")
lines_during = flush(ser, 3.0)
lpwm = rpwm = lrpm = rrpm = lma = rma = None
for l in lines_during:
    if l.startswith("T "):
        parts = {}
        for tok in l[2:].split():
            if "=" in tok:
                k,_,v = tok.partition("=")
                parts[k] = v
        lpwm = parts.get("Lpwm","?")
        rpwm = parts.get("Rpwm","?")
        lrpm = parts.get("Lrpm","?")
        rrpm = parts.get("Rrpm","?")
        lma  = parts.get("LmA","?")
        rma  = parts.get("RmA","?")

cmd(ser, "INHABILITAR", wait=0.4, show=False)

# ─── Resultado TEST 2 ───────────────────────────────────────────────
sep("RESULTADO TEST 2")
if rpwm is not None and rpwm != "?":
    rpwm_i = int(rpwm)
    rrpm_i = int(rrpm) if rrpm and rrpm != "?" else 0
    rma_f  = float(rma) if rma and rma != "?" else 0.0

    print(f"  Izquierdo → Lpwm={lpwm}  Lrpm={lrpm}  LmA={lma}A")
    print(f"  Derecho   → Rpwm={rpwm}  Rrpm={rrpm}  RmA={rma}A\n")

    if rpwm_i == 0:
        print(f"  {R}✗ Rpwm=0 → El firmware NO envía PWM al motor derecho.{X}")
        print(f"  {Y}  → Problema de SOFTWARE. Revisar lógica PID/cogging.{X}")
    elif rpwm_i > 0 and rrpm_i == 0 and rma_f < 0.1:
        print(f"  {R}✗ Rpwm={rpwm_i} pero Rrpm=0 y RmA≈0A{X}")
        print(f"  {R}  → PWM llega al firmware pero el motor NO gira.{X}")
        print(f"  {Y}  → Problema de HARDWARE:{X}")
        print(f"  {Y}    • Revisar cableado PWM pin44→driver derecho{X}")
        print(f"  {Y}    • Revisar cableado STOP pin26 (¿tirado a GND?){X}")
        print(f"  {Y}    • Revisar driver ZS-X11H canal derecho{X}")
        print(f"  {Y}    • Medir voltaje en salida del driver con multímetro{X}")
    elif rpwm_i > 0 and (rrpm_i > 0 or rma_f > 0.1):
        print(f"  {G}✓ Motor derecho FUNCIONANDO con PWM directo!{X}")
        print(f"  {Y}  → El problema era el umbral mínimo de PWM (cogging).{X}")
        print(f"  {Y}  → El fix MIN_PWM_RIGHT=22 debería haber resuelto esto.{X}")
    else:
        print(f"  {Y}⚠ Resultado ambiguo — datos incompletos.{X}")
else:
    print(f"  {Y}⚠ No se recibió telemetría T durante el test.{X}")
    print(f"  {Y}  Verifica que el firmware esté cargado correctamente.{X}")

# ─── TEST 3: Motor derecho solo (R directo) ────────────────────────
sep("TEST 3 — Motor derecho solo (comando R 50)")
print(f"  {Y}Observa si el motor DERECHO gira con este comando directo.{X}")
cmd(ser, "hb off", wait=0.5)        # balance OFF — imprescindible para test directo
cmd(ser, "HABILITAR", wait=0.5)
cmd(ser, "R 50", wait=0.3)
time.sleep(2.5)
cmd(ser, "R 0",  wait=0.3, show=False)
cmd(ser, "INHABILITAR", wait=0.3, show=False)

print(f"\n  Si el motor derecho giró en TEST 3 → problema era velocidad mínima.")
print(f"  Si NO giró en TEST 3 → problema de hardware (driver/cableado/motor).\n")

# ─── TEST 4: cmd_vel v 0.3 — el caso problemático ──────────────────
sep("TEST 4 — v 0.3 0.0 con balance ON (caso real Stadia)")
print(f"  {Y}Este es el escenario exacto que falló antes: v=0.3 con PID.{X}")
print(f"  {Y}Con MIN_PWM_RIGHT=60 el motor derecho debería mantener RPM.{X}")
cmd(ser, "hb on", wait=0.5)
cmd(ser, "v 0.3 0.0", wait=0.2)
print(f"  {Y}⏳ Observando 3 segundos...{X}")
lines_v = flush(ser, 3.0)
max_rrpm = 0
rma_max  = 0.0
for l in lines_v:
    if l.startswith("B "):
        parts = {}
        for tok in l[2:].split():
            if "=" in tok:
                k,_,v = tok.partition("=")
                parts[k] = v
        try:
            rr = int(float(parts.get("Rrpm","0")))
            rm = abs(float(parts.get("RmA","0")))
            if rr > max_rrpm: max_rrpm = rr
            if rm > rma_max:  rma_max  = rm
        except Exception:
            pass
cmd(ser, "v 0.0 0.0", wait=0.3, show=False)
cmd(ser, "INHABILITAR", wait=0.3, show=False)

sep("RESULTADO TEST 4")
print(f"  Motor derecho RPM máx durante 3s: {max_rrpm}")
print(f"  Motor derecho Corriente máx:       {rma_max:.2f}A\n")
if max_rrpm > 10 and rma_max > 0.05:
    print(f"  {G}✓ MOTOR DERECHO FUNCIONANDO a v=0.3 con MIN_PWM=60  ✓{X}")
    print(f"  {G}  El fix de stiction funciona correctamente.{X}")
elif max_rrpm > 0:
    print(f"  {Y}⚠ Motor derecho gira parcialmente (Rrpm={max_rrpm}) — puede mejorar{X}")
else:
    print(f"  {R}✗ Motor derecho sigue sin girar a v=0.3{X}")
    print(f"  {Y}  Posibles causas:{X}")
    print(f"  {Y}  1. MIN_PWM_RIGHT=60 no es suficiente — subir a 70{X}")
    print(f"  {Y}  2. El PID calcula PWM<60 y setRightMotor lo eleva a 60{X}")
    print(f"  {Y}     pero 60 sigue por debajo de la stiction real{X}")
    print(f"  {Y}  3. Problema mecánico — rodamiento, engranaje atascado{X}")

ser.close()
print(f"{G}Diagnóstico completado.{X}\n")
