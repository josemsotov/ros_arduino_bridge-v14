"""
stadia_controller.py
Smart Golf Trolley — Control Stadia → Serial → Arduino Mega

DEPENDENCIAS:
    pip install pyserial pywinusb

FASE ACTUAL:  Stadia BT --> PC --> USB Serial --> Arduino
FASE FUTURA:  Stadia BT --> Pi5 --> USB Serial --> Arduino

PROTOCOLO ARDUINO (ROS2_Bridge.h):
    v <linear m/s> <angular rad/s>   -> mover
    v 0.0 0.0                        -> parar
    e                                -> leer encoders
    s                                -> estado

MAPEO STADIA (HID raw, layout confirmado por captura):
    Report ID 0x03, 11 bytes:
      [2] botones grupo 1 (L2/R2/bumpers)
      [3] botones grupo 2 (cara + d-pad)
      [4] Left Stick X   (0=izq, 128=centro, 255=der)
      [5] Left Stick Y   (0=arriba, 128=centro, 255=abajo)
      [6] Right Stick X
      [7] Right Stick Y
      [8] Left Trigger analógico (0-255)
      [9] Right Trigger analógico (0-255)

    Left Stick Y  -> velocidad lineal  adelante/atras
    Left Stick X  -> velocidad angular giro izq/der
    BTN_A  (0x08 en byte[3]) -> PARADA DE EMERGENCIA
    BTN_MENU (0x20 en byte[3]) -> solicitar estado (s)
    BTN_CAP  (0x10 en byte[3]) -> leer encoders (e)

USO:
    python stadia_controller.py           # auto-detecta puerto Arduino
    python stadia_controller.py COM5      # puerto especifico
    python stadia_controller.py --scan    # listar puertos y gamepads disponibles
    python stadia_controller.py --test    # modo test: muestra ejes y botones en tiempo real

Ctrl+C para salir (envia v 0.0 0.0 antes de cerrar).
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports
import pywinusb.hid as hid

# ===========================================================================
# STADIA HID -- layout confirmado por captura raw (VID=0x18D1 PID=0x9400)
# ===========================================================================

STADIA_VID = 0x18D1
STADIA_PID = 0x9400

# Índices dentro del paquete HID (0-based, [0]=Report ID=0x03)
IDX_BTN1  = 2   # botones grupo 1 (bumpers/triggers digitales)
IDX_BTN2  = 3   # botones grupo 2 (cara + d-pad bits)
IDX_LSX   = 4   # Left Stick X
IDX_LSY   = 5   # Left Stick Y
IDX_RSX   = 6   # Right Stick X
IDX_RSY   = 7   # Right Stick Y
IDX_LT    = 8   # Left Trigger analógico
IDX_RT    = 9   # Right Trigger analógico

AXIS_CENTER = 128
AXIS_RANGE  = 127.0

# Máscaras en byte[3] — confirmadas por test hardware
# Ejecuta --test y pulsa cada botón para verificar
BTN_A    = 0x08   # Botón A (cara inferior)   -> PARADA EMERGENCIA
BTN_B    = 0x04   # Botón B (cara derecha)
BTN_X    = 0x10   # Botón X (cara izquierda)
BTN_Y    = 0x20   # Botón Y (cara superior)   -> TOGGLE AUTO-BALANCEO
BTN_MENU = 0x40   # Botón Menu (tres líneas)  -> solicitar estado
BTN_CAP  = 0x02   # Botón Capture (círculo)   -> encoders

# Estado compartido entre el hilo HID y el hilo serial
_lock   = threading.Lock()
_state  = {
    'lsx': 128, 'lsy': 128,
    'rsx': 128, 'rsy': 128,
    'lt': 0,    'rt': 0,
    'btn1': 0,  'btn2': 0,
    'connected': False,
}

def _hid_handler(data):
    """Callback llamado por pywinusb con cada paquete HID."""
    if len(data) < 10:
        return
    if data[0] != 0x03:
        return
    with _lock:
        _state['btn1'] = data[IDX_BTN1]
        _state['btn2'] = data[IDX_BTN2]
        _state['lsx']  = data[IDX_LSX]
        _state['lsy']  = data[IDX_LSY]
        _state['rsx']  = data[IDX_RSX]
        _state['rsy']  = data[IDX_RSY]
        _state['lt']   = data[IDX_LT]
        _state['rt']   = data[IDX_RT]
        _state['connected'] = True

def stadia_open():
    """Abre el Stadia HID y registra el handler. Devuelve el dispositivo o None."""
    devices = [d for d in hid.find_all_hid_devices()
               if d.vendor_id == STADIA_VID and d.product_id == STADIA_PID]
    if not devices:
        return None
    dev = devices[0]
    dev.open()
    dev.set_raw_data_handler(_hid_handler)
    return dev

def stadia_get():
    """Devuelve una copia del estado actual del mando."""
    with _lock:
        return dict(_state)

def axis_normalize(raw: int) -> float:
    """Convierte valor 0-255 a -1.0 ... +1.0 con centro en 128."""
    return (raw - AXIS_CENTER) / AXIS_RANGE

# ===========================================================================
# CONFIGURACION
# ===========================================================================

COM_PORT     = None      # None = auto-detectar. Forzar: "COM5"
BAUD_RATE    = 115200
SEND_RATE_HZ = 10

MAX_LINEAR   = 0.5       # m/s  maximo
MAX_ANGULAR  = 1.5       # rad/s maximo
DEADZONE     = 0.12

# ===========================================================================
# UTILIDADES
# ===========================================================================

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[SERIAL] No se encontraron puertos COM.")
    else:
        print("[SERIAL] Puertos disponibles:")
        for p in ports:
            print(f"  {p.device:10s}  {p.description}")

def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

def find_arduino_port() -> str:
    for p in serial.tools.list_ports.comports():
        desc = p.description.lower()
        if "arduino" in desc or "mega" in desc or "ch340" in desc or "cp210" in desc:
            return p.device
    return None

# ===========================================================================
# MODO TEST -- muestra ejes y botones en tiempo real para calibrar el mapeo
# ===========================================================================

def run_test():
    dev = stadia_open()
    if dev is None:
        print("[ERROR] Stadia no encontrado (VID=0x18D1 PID=0x9400). Conectalo por BT.")
        sys.exit(1)

    print("[TEST] Stadia conectado via HID.")
    print("[TEST] Mueve los ejes y pulsa botones. Ctrl+C para salir.\n")
    print("       Columnas: LSX  LSY  RSX  RSY   LT   RT  BTN1 BTN2")

    prev_btn1, prev_btn2 = 0, 0
    try:
        while True:
            s = stadia_get()
            lx = axis_normalize(s['lsx'])
            ly = axis_normalize(s['lsy'])
            rx = axis_normalize(s['rsx'])
            ry = axis_normalize(s['rsy'])

            # Detectar nuevos botones pulsados
            new1 = s['btn1'] & ~prev_btn1
            new2 = s['btn2'] & ~prev_btn2
            if new1:
                for bit in range(8):
                    if new1 & (1 << bit):
                        print(f"\n  [BTN] byte[2] bit {bit}  mascara=0x{1<<bit:02X}")
            if new2:
                for bit in range(8):
                    if new2 & (1 << bit):
                        print(f"\n  [BTN] byte[3] bit {bit}  mascara=0x{1<<bit:02X}")
            prev_btn1, prev_btn2 = s['btn1'], s['btn2']

            print(
                f"  LSX={lx:+.2f} LSY={ly:+.2f}  RSX={rx:+.2f} RSY={ry:+.2f}"
                f"  LT={s['lt']:>3} RT={s['rt']:>3}"
                f"  B1=0x{s['btn1']:02X} B2=0x{s['btn2']:02X}   ",
                end="\r"
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[TEST] Fin.")
    finally:
        dev.close()

# ===========================================================================
# MAIN
# ===========================================================================

def main():
    # --- Modo test ---
    if "--test" in sys.argv:
        run_test()
        return

    # --- Modo scan ---
    if "--scan" in sys.argv:
        list_ports()
        print()
        devices = [d for d in hid.find_all_hid_devices()
                   if d.vendor_id == STADIA_VID and d.product_id == STADIA_PID]
        if devices:
            print(f"[GAMEPAD] Stadia HID encontrado ({len(devices)} interfaz(es))")
        else:
            print("[GAMEPAD] Stadia no encontrado (VID=0x18D1 PID=0x9400)")
        sys.exit(0)

    # --- Puerto serial ---
    port = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else COM_PORT
    if port is None:
        port = find_arduino_port()
    if port is None:
        list_ports()
        port = input("Introduce el puerto COM del Arduino (ej: COM5): ").strip()

    print(f"\n[SERIAL] Conectando a {port} @ {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] No se pudo abrir {port}: {e}")
        list_ports()
        sys.exit(1)

    time.sleep(2)
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(f"  [BOOT] {line}")

    # --- Abrir Stadia HID ---
    dev = stadia_open()
    if dev is None:
        print("[ERROR] Stadia no encontrado (VID=0x18D1 PID=0x9400).")
        print("        Conecta el mando por Bluetooth y vuelve a intentarlo.")
        ser.close()
        sys.exit(1)

    print("[GAMEPAD] Stadia conectado via HID.")
    print("[INFO] Ejecuta '--test' para verificar el mapeo de botones.\n")

    print("========================================")
    print("  Smart Golf Trolley -- Control Stadia")
    print("========================================")
    print(f"  Left Stick arriba/abajo  -> Lineal   (max +-{MAX_LINEAR} m/s)")
    print(f"  Left Stick izq/der       -> Angular  (max +-{MAX_ANGULAR} rad/s)")
    print(f"  Boton A  (B2=0x{BTN_A:02X}) -> PARADA DE EMERGENCIA")
    print(f"  Boton Y  (B2=0x{BTN_Y:02X}) -> TOGGLE AUTO-BALANCEO (on/off)")
    print(f"  Boton Menu (B2=0x{BTN_MENU:02X}) -> Estado del sistema (s)")
    print(f"  Ctrl+C                   -> Salir (para motores)")
    print("========================================\n")

    interval      = 1.0 / SEND_RATE_HZ
    last_send     = 0.0
    last_linear   = None
    last_angular  = None
    prev_btn2     = 0
    balance_active = False  # estado local del balance

    def _parse_T(line):
        """Parsea líneas de telemetría T o B del Arduino."""
        d = {}
        for token in line.split():
            if '=' in token:
                k, _, v = token.partition('=')
                d[k] = v
        return d

    def read_arduino():
        """Hilo que lee y muestra respuestas del Arduino."""
        nonlocal balance_active
        while True:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("T "):
                        d = _parse_T(line[2:])
                        lin  = float(d.get('lin',  0))
                        ang  = float(d.get('ang',  0))
                        lrpm = int(float(d.get('Lrpm', 0)))
                        rrpm = int(float(d.get('Rrpm', 0)))
                        lma  = float(d.get('LmA', 0.0))
                        rma  = float(d.get('RmA', 0.0))
                        bal  = "[BAL]" if balance_active else "     "
                        print(
                            f"\r  {bal} v={lin:+.3f} w={ang:+.3f} | "
                            f"Lrpm={lrpm:>5} Rrpm={rrpm:>5} | "
                            f"I L={lma:+.2f}A R={rma:+.2f}A   ",
                            end='', flush=True
                        )
                    elif line.startswith("B "):
                        # Suprimir líneas B del balance para no contaminar pantalla
                        pass
                    elif "ACTIVADO" in line:
                        balance_active = True
                        print(f"\n  [BALANCE] *** ACTIVO ***", flush=True)
                    elif "DESACTIVADO" in line:
                        balance_active = False
                        print(f"\n  [BALANCE] --- inactivo ---", flush=True)
                    elif "CAIDA" in line:
                        balance_active = False
                        print(f"\n  [BALANCE] !!! CAIDA DETECTADA — balance OFF !!!", flush=True)
                    else:
                        print(f"\n  [ARD] {line}", flush=True)
            except Exception:
                break
            time.sleep(0.02)

    t = threading.Thread(target=read_arduino, daemon=True)
    t.start()

    try:
        while True:
            now = time.monotonic()
            s   = stadia_get()

            btn2    = s['btn2']
            pressed = btn2 & ~prev_btn2
            prev_btn2 = btn2

            # Parada de emergencia — botón A
            if btn2 & BTN_A:
                ser.write(b"v 0.0 0.0\n")
                print("[STOP] Parada de emergencia (boton A)          ")
                last_linear  = 0.0
                last_angular = 0.0
                time.sleep(0.1)
                continue

            # Botón Y → toggle auto-balanceo
            if pressed & BTN_Y:
                if balance_active:
                    ser.write(b"hb off\n")
                    print("\n  [Y] Auto-balanceo DESACTIVADO                   ", flush=True)
                else:
                    ser.write(b"hb on\n")
                    print("\n  [Y] Auto-balanceo ACTIVADO                      ", flush=True)

            # Botón Menu → estado
            if pressed & BTN_MENU:
                ser.write(b"s\n")
                print("\n  [MENU] Solicitando estado...                    ", flush=True)

            # Calcular velocidades con zona muerta
            # LSY: 0=arriba → positivo; 255=abajo → negativo
            raw_ly =  axis_normalize(s['lsy'])   # up=negative raw → invertir
            raw_lx = -axis_normalize(s['lsx'])   # left=negative raw → CCW=positivo

            linear  = apply_deadzone(-raw_ly, DEADZONE) * MAX_LINEAR
            angular = apply_deadzone( raw_lx, DEADZONE) * MAX_ANGULAR

            # Enviar a la frecuencia configurada (siempre, para mantener vivo el timeout Arduino)
            if now - last_send >= interval:
                cmd = f"v {linear:.3f} {angular:.3f}\n"
                ser.write(cmd.encode())

                if linear != 0.0 or angular != 0.0:
                    print(f"  v  lin={linear:+.3f} m/s   ang={angular:+.3f} rad/s   ", end="\r")
                else:
                    print(f"  [en reposo]                                    ", end="\r")

                last_linear  = linear
                last_angular = angular
                last_send = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[EXIT] Parando motores...")
        ser.write(b"v 0.0 0.0\n")
        time.sleep(0.3)
    finally:
        dev.close()
        ser.close()
        print("[EXIT] Conexiones cerradas.")

if __name__ == "__main__":
    main()
