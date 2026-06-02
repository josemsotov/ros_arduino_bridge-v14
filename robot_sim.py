"""
robot_sim.py
Smart Golf Trolley - Simulador Visual con Control Stadia

Vista cenital del robot diferencial con:
- Robot animado (cuerpo + ruedas + flecha de direccion)
- Trail de trayectoria con degradado
- Visualizacion del joystick
- Barras PWM rueda izquierda / derecha
- Posicion y orientacion en tiempo real
- Conexion opcional al Arduino por Serial

Stadia HID report (ID=0x03, 11 bytes):
  [0]  Report ID = 0x03
  [1]  D-pad (nibble bajo) + botones
  [2]  Botones  (bit0=A, bit1=B, bit2=X, bit3=Y, bit4=LB, bit5=RB)
  [3]  Botones  (bit0=Capture, bit1=Menu)
  [4]  Left  Stick X   0x00=izq  0x80=centro  0xFF=der
  [5]  Left  Stick Y   0x00=arr  0x80=centro  0xFF=abj
  [6]  Right Stick X
  [7]  Right Stick Y
  [8]  L2 trigger
  [9]  R2 trigger
  [10] extra

Dependencias: pip install pyserial pywinusb
"""

import tkinter as tk
import threading
import time
import math
import sys

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

import pywinusb.hid as hid

# ---------------------------------------------------------------------------
# Constantes de configuracion
# ---------------------------------------------------------------------------

STADIA_VID   = 0x18D1
STADIA_PID   = 0x9400

MAX_LINEAR   = 0.5     # m/s
MAX_ANGULAR  = 1.5     # rad/s
DEADZONE     = 0.12
WHEEL_BASE   = 0.35    # m (distancia entre ruedas)

# Que stick controla el robot
# "left"  -> bytes [4],[5]  (Left Stick)
# "right" -> bytes [6],[7]  (Right Stick)
DRIVE_STICK  = "right"   # cambiar a "left" si el left stick responde

# ---------------------------------------------------------------------------
# Estado compartido (hilo HID <-> hilo GUI)
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.lock          = threading.Lock()
        self.raw           = [0] * 11        # ultimo paquete HID
        self.linear        = 0.0
        self.angular       = 0.0
        self.x             = 0.0
        self.y             = 0.0
        self.theta         = math.pi / 2     # apuntando arriba al inicio
        self.stadia_ok     = False
        self.last_hid_time = 0.0
        self.ser           = None

state = State()

# ---------------------------------------------------------------------------
# Utilidades de eje
# ---------------------------------------------------------------------------

def norm_axis(raw_byte):
    """Convierte 0-255 a -1.0...+1.0"""
    v = (int(raw_byte) - 128) / 127.0
    return max(-1.0, min(1.0, v))

def deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - dz) / (1.0 - dz)

# ---------------------------------------------------------------------------
# Callback HID (ejecutado en hilo de pywinusb)
# ---------------------------------------------------------------------------

def hid_handler(data):
    if len(data) < 10:
        return
    with state.lock:
        state.raw      = list(data)
        state.last_hid_time = time.monotonic()

        if DRIVE_STICK == "left":
            raw_fwd = -norm_axis(data[5])   # Y invertido: arriba=adelante
            raw_ang = -norm_axis(data[4])   # X: izq=CCW positivo
        else:
            raw_fwd = -norm_axis(data[7])
            raw_ang = -norm_axis(data[6])

        state.linear  = deadzone(raw_fwd, DEADZONE) * MAX_LINEAR
        state.angular = deadzone(raw_ang, DEADZONE) * MAX_ANGULAR

# ---------------------------------------------------------------------------
# Aplicacion GUI
# ---------------------------------------------------------------------------

MAP_W   = 620
MAP_H   = 620
PANEL_W = 300
SCALE   = 80    # pixeles por metro

BG_MAP    = "#0d1117"
BG_PANEL  = "#161b22"
BG_CARD   = "#21262d"
FG_TXT    = "#c9d1d9"
FG_ACC    = "#f78166"
FG_GREEN  = "#56d364"
FG_YELLOW = "#e3b341"
FG_DIM    = "#484f58"
GRID_C    = "#161b22"
AXIS_C    = "#1c2128"
TRAIL_C1  = "#1c2f4a"
TRAIL_C2  = "#388bfd"
ROBOT_C   = "#f78166"
WHEEL_C   = "#388bfd"


class App:
    def __init__(self, root):
        self.root     = root
        self.trail    = []
        self._last_t  = time.monotonic()
        root.title("Smart Golf Trolley — Simulador Visual")
        root.configure(bg=BG_PANEL)
        root.resizable(False, False)

        self._build_ui()
        self._draw_grid()
        self._start_hid()
        self._tick()

    # ------------------------------------------------------------------ UI --

    def _build_ui(self):
        # Canvas izquierdo (mapa)
        self.cv = tk.Canvas(self.root, width=MAP_W, height=MAP_H,
                            bg=BG_MAP, highlightthickness=0)
        self.cv.pack(side=tk.LEFT, padx=8, pady=8)

        # Panel derecho
        p = tk.Frame(self.root, bg=BG_PANEL, width=PANEL_W)
        p.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8), pady=8)
        p.pack_propagate(False)

        def lbl(parent, text, size=9, bold=False, color=FG_TXT):
            w = tk.Label(parent, text=text, bg=BG_PANEL, fg=color,
                         font=("Consolas", size, "bold" if bold else "normal"))
            return w

        def sep():
            tk.Frame(p, bg=FG_DIM, height=1).pack(fill=tk.X, padx=10, pady=6)

        # Titulo
        lbl(p, "SMART GOLF TROLLEY", 11, True, FG_ACC).pack(pady=(14, 0))
        lbl(p, "Simulador Visual  v14", 9, color=FG_DIM).pack(pady=(2, 10))
        sep()

        # --- Estado conexion ---
        f = tk.Frame(p, bg=BG_PANEL); f.pack(fill=tk.X, padx=14)
        lbl(f, "Stadia:").grid(row=0, column=0, sticky='w', pady=2)
        self.lbl_stadia = tk.Label(f, text="● Buscando...", bg=BG_PANEL,
                                   fg=FG_YELLOW, font=("Consolas", 9))
        self.lbl_stadia.grid(row=0, column=1, sticky='w', padx=8)

        lbl(f, "Arduino:").grid(row=1, column=0, sticky='w', pady=2)
        self.lbl_ard = tk.Label(f, text="● Sin conectar", bg=BG_PANEL,
                                fg=FG_DIM, font=("Consolas", 9))
        self.lbl_ard.grid(row=1, column=1, sticky='w', padx=8)
        sep()

        # --- Joystick visual ---
        lbl(p, "JOYSTICK  (stick activo)", 9, True).pack()
        self.joy_cv = tk.Canvas(p, width=130, height=130,
                                bg=BG_CARD, highlightthickness=1,
                                highlightbackground=FG_DIM)
        self.joy_cv.pack(pady=6)
        R = 50
        cx = cy = 65
        self.joy_cv.create_oval(cx-R, cy-R, cx+R, cy+R,
                                outline=FG_DIM, width=2)
        dz_r = int(R * DEADZONE)
        self.joy_cv.create_oval(cx-dz_r, cy-dz_r, cx+dz_r, cy+dz_r,
                                outline="#2d333b", dash=(3, 3))
        self.joy_cv.create_line(cx-R, cy, cx+R, cy, fill="#2d333b")
        self.joy_cv.create_line(cx, cy-R, cx, cy+R, fill="#2d333b")
        self._joy_dot = self.joy_cv.create_oval(cx-7, cy-7, cx+7, cy+7,
                                                fill=FG_ACC, outline="")
        self._joy_cx = cx; self._joy_cy = cy; self._joy_R = R
        sep()

        # --- Velocidades ---
        f = tk.Frame(p, bg=BG_PANEL); f.pack(fill=tk.X, padx=14)
        lbl(f, "VELOCIDADES", 9, True).grid(row=0, column=0, columnspan=2,
                                             sticky='w', pady=(0, 4))
        lbl(f, "Lineal:").grid(row=1, column=0, sticky='w')
        self.lbl_v = tk.Label(f, text="+0.000 m/s", bg=BG_PANEL,
                              fg=FG_ACC, font=("Consolas", 11, "bold"))
        self.lbl_v.grid(row=1, column=1, sticky='w', padx=8)

        lbl(f, "Angular:").grid(row=2, column=0, sticky='w', pady=2)
        self.lbl_w = tk.Label(f, text="+0.000 rad/s", bg=BG_PANEL,
                              fg=FG_ACC, font=("Consolas", 11, "bold"))
        self.lbl_w.grid(row=2, column=1, sticky='w', padx=8)

        # Barras PWM ruedas
        lbl(f, "Rueda Izq:", color=FG_DIM).grid(row=3, column=0, sticky='w', pady=(8, 0))
        self._bar_l_var = tk.DoubleVar()
        self._bar_l_bg = tk.Frame(f, bg="#2d333b", width=140, height=10)
        self._bar_l_bg.grid(row=3, column=1, sticky='w', padx=8, pady=(8, 0))
        self._bar_l_fill = tk.Frame(self._bar_l_bg, bg=FG_GREEN, height=10)
        self._bar_l_fill.place(x=0, y=0, width=0, height=10)

        lbl(f, "Rueda Der:", color=FG_DIM).grid(row=4, column=0, sticky='w', pady=2)
        self._bar_r_bg = tk.Frame(f, bg="#2d333b", width=140, height=10)
        self._bar_r_bg.grid(row=4, column=1, sticky='w', padx=8, pady=2)
        self._bar_r_fill = tk.Frame(self._bar_r_bg, bg=FG_GREEN, height=10)
        self._bar_r_fill.place(x=0, y=0, width=0, height=10)
        sep()

        # --- Posicion ---
        f = tk.Frame(p, bg=BG_PANEL); f.pack(fill=tk.X, padx=14)
        lbl(f, "POSICION (simulacion)", 9, True).grid(row=0, column=0,
                                                        columnspan=2, sticky='w')
        for i, (name, attr) in enumerate([("X:", "lbl_x"),
                                           ("Y:", "lbl_y"),
                                           ("Angulo:", "lbl_ang")], 1):
            lbl(f, name).grid(row=i, column=0, sticky='w', pady=1)
            w = tk.Label(f, text="0.00", bg=BG_PANEL,
                         fg=FG_TXT, font=("Consolas", 9))
            w.grid(row=i, column=1, sticky='w', padx=8)
            setattr(self, attr, w)
        sep()

        # --- Botones ---
        bf = tk.Frame(p, bg=BG_PANEL); bf.pack(fill=tk.X, padx=14)
        for txt, cmd in [("Reset posicion", self._reset),
                         ("Borrar trail",   self._clear_trail)]:
            tk.Button(bf, text=txt, bg=BG_CARD, fg=FG_TXT,
                      font=("Consolas", 9), relief='flat',
                      padx=6, pady=4, cursor="hand2",
                      activebackground="#2d333b",
                      command=cmd).pack(side=tk.LEFT, padx=(0, 4))

        # --- Conectar Arduino ---
        sep()
        lbl(p, "ARDUINO  (opcional)", 9, True).pack(anchor='w', padx=14)
        af = tk.Frame(p, bg=BG_PANEL); af.pack(fill=tk.X, padx=14, pady=4)
        self._port_var = tk.StringVar(value="Auto")
        ports = ["Auto"] + ([p.device for p in serial.tools.list_ports.comports()]
                            if HAS_SERIAL else [])
        om = tk.OptionMenu(af, self._port_var, *ports)
        om.config(bg=BG_CARD, fg=FG_TXT, font=("Consolas", 9),
                  relief='flat', highlightthickness=0, width=10)
        om.pack(side=tk.LEFT)
        tk.Button(af, text="Conectar", bg=BG_CARD, fg=FG_TXT,
                  font=("Consolas", 9), relief='flat', padx=6, pady=4,
                  cursor="hand2", command=self._connect_arduino).pack(
                      side=tk.LEFT, padx=6)

    # --------------------------------------------------------------- Grid ---

    def _draw_grid(self):
        step = SCALE
        for xi in range(0, MAP_W + step, step):
            self.cv.create_line(xi, 0, xi, MAP_H, fill=GRID_C, tags="bg")
        for yi in range(0, MAP_H + step, step):
            self.cv.create_line(0, yi, MAP_W, yi, fill=GRID_C, tags="bg")
        cx, cy = MAP_W // 2, MAP_H // 2
        self.cv.create_line(0, cy, MAP_W, cy, fill=AXIS_C, width=2, tags="bg")
        self.cv.create_line(cx, 0, cx, MAP_H, fill=AXIS_C, width=2, tags="bg")
        # Etiquetas de metros
        for m in range(-4, 5):
            if m == 0:
                continue
            px = cx + m * SCALE
            py = cy - m * SCALE
            self.cv.create_text(px, cy + 12, text=f"{m}m",
                                fill="#2d333b", font=("Consolas", 7), tags="bg")
            self.cv.create_text(cx - 16, py, text=f"{m}m",
                                fill="#2d333b", font=("Consolas", 7), tags="bg")

    # -------------------------------------------------------------- Robot ---

    def _w2c(self, x, y):
        """Coordenadas mundo (m) -> canvas (px)"""
        return (MAP_W // 2 + x * SCALE,
                MAP_H // 2 - y * SCALE)

    def _draw_robot(self, x, y, theta):
        self.cv.delete("robot")
        cx, cy = self._w2c(x, y)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        def rot(lx, ly):
            # local (fwd=+y, right=+x) -> screen
            sx = lx * cos_t - ly * sin_t
            sy = lx * sin_t + ly * cos_t
            return cx + sx, cy - sy

        BW, BH = 28, 42    # cuerpo (px)
        WW, WH = 8, 20     # rueda

        # Cuerpo
        body = [rot(-BW/2, -BH/2), rot(BW/2, -BH/2),
                rot(BW/2,   BH/2), rot(-BW/2,  BH/2)]
        self.cv.create_polygon(body, fill="#8b1a1a", outline=ROBOT_C,
                               width=2, tags="robot")

        # Flecha de avance
        tx, ty = rot(0, BH/2 + 10)
        self.cv.create_line(cx, cy, tx, ty, fill="white", width=3,
                            arrow=tk.LAST, arrowshape=(12, 14, 5), tags="robot")

        # Ruedas
        for side in (-1, 1):
            wx = side * (BW/2 + WW/2)
            corners = [rot(wx - WW/2, -WH/2), rot(wx + WW/2, -WH/2),
                       rot(wx + WW/2,  WH/2), rot(wx - WW/2,  WH/2)]
            self.cv.create_polygon(corners, fill=WHEEL_C,
                                   outline="#1f6feb", width=1, tags="robot")

        # Punto central
        self.cv.create_oval(cx-4, cy-4, cx+4, cy+4,
                            fill="white", outline="", tags="robot")

    # ----------------------------------------------------------- HID loop ---

    def _start_hid(self):
        t = threading.Thread(target=self._hid_loop, daemon=True)
        t.start()

    def _hid_loop(self):
        dev = None
        while True:
            devs = [d for d in hid.find_all_hid_devices()
                    if d.vendor_id == STADIA_VID and d.product_id == STADIA_PID]
            if devs:
                try:
                    dev = devs[0]
                    dev.open()
                    dev.set_raw_data_handler(hid_handler)
                    with state.lock:
                        state.stadia_ok = True
                    # Mantener mientras lleguen datos
                    while True:
                        time.sleep(0.5)
                        with state.lock:
                            age = time.monotonic() - state.last_hid_time
                        if age > 5.0:
                            break
                    try:
                        dev.close()
                    except Exception:
                        pass
                except Exception:
                    pass
            with state.lock:
                state.stadia_ok = False
                state.linear    = 0.0
                state.angular   = 0.0
            time.sleep(2)

    # ---------------------------------------------------- Arduino serial ---

    def _connect_arduino(self):
        if not HAS_SERIAL:
            return
        port_sel = self._port_var.get()
        if port_sel == "Auto":
            port_sel = None
            for p in serial.tools.list_ports.comports():
                desc = p.description.lower()
                if any(k in desc for k in ("arduino", "mega", "ch340", "cp210")):
                    port_sel = p.device
                    break
        if port_sel is None:
            self.lbl_ard.config(text="● No encontrado", fg=FG_YELLOW)
            return
        try:
            ser = serial.Serial(port_sel, 115200, timeout=1)
            with state.lock:
                state.ser = ser
            self.lbl_ard.config(text=f"● {port_sel}", fg=FG_GREEN)
        except Exception as e:
            self.lbl_ard.config(text=f"● Error: {e}", fg=FG_ACC)

    # ------------------------------------------------------------- Tick ---

    def _tick(self):
        now  = time.monotonic()
        dt   = min(now - self._last_t, 0.1)
        self._last_t = now

        with state.lock:
            linear  = state.linear
            angular = state.angular
            ok      = state.stadia_ok
            raw     = list(state.raw)
            # Integrar odometria
            state.theta += angular * dt
            state.x     += linear * math.cos(state.theta) * dt
            state.y     += linear * math.sin(state.theta) * dt
            x, y, theta = state.x, state.y, state.theta
            # Enviar serial si conectado
            ser = state.ser

        if ser is not None:
            try:
                cmd = f"v {linear:.3f} {angular:.3f}\n"
                ser.write(cmd.encode())
            except Exception:
                with state.lock:
                    state.ser = None
                self.lbl_ard.config(text="● Desconectado", fg=FG_DIM)

        # --- Trail ---
        cx, cy = self._w2c(x, y)
        if not self.trail or abs(cx - self.trail[-1][0]) > 1.5 or abs(cy - self.trail[-1][1]) > 1.5:
            self.trail.append((cx, cy))
            if len(self.trail) > 800:
                self.trail.pop(0)

        # --- Redibujar trail + robot ---
        self.cv.delete("trail")
        self.cv.delete("robot")
        n = len(self.trail)
        if n > 1:
            for i in range(1, n):
                t_alpha = i / n
                r = int(0x1c + t_alpha * (0x38 - 0x1c))
                g = int(0x2f + t_alpha * (0x8b - 0x2f))
                b = int(0x4a + t_alpha * (0xfd - 0x4a))
                color = f"#{r:02x}{g:02x}{b:02x}"
                x0, y0 = self.trail[i - 1]
                x1, y1 = self.trail[i]
                self.cv.create_line(x0, y0, x1, y1, fill=color,
                                    width=2, tags="trail")

        self._draw_robot(x, y, theta)

        # --- Joystick dot ---
        if raw and len(raw) >= 8:
            if DRIVE_STICK == "left":
                jx, jy = raw[4], raw[5]
            else:
                jx, jy = raw[6], raw[7]
            nx =  norm_axis(jx)
            ny = -norm_axis(jy)
            dot_x = self._joy_cx + nx * self._joy_R * 0.88
            dot_y = self._joy_cy - ny * self._joy_R * 0.88
            self.joy_cv.coords(self._joy_dot,
                               dot_x - 7, dot_y - 7, dot_x + 7, dot_y + 7)

        # --- Labels velocidad ---
        self.lbl_v.config(text=f"{linear:+.3f} m/s")
        self.lbl_w.config(text=f"{angular:+.3f} rad/s")

        # Barras PWM (cinematica diferencial)
        pwm_l = (linear - angular * WHEEL_BASE / 2) / MAX_LINEAR
        pwm_r = (linear + angular * WHEEL_BASE / 2) / MAX_LINEAR
        bar_w = 140
        self._bar_l_fill.place(width=int(abs(pwm_l) * bar_w))
        self._bar_r_fill.place(width=int(abs(pwm_r) * bar_w))
        lc = FG_GREEN if pwm_l >= 0 else FG_ACC
        rc = FG_GREEN if pwm_r >= 0 else FG_ACC
        self._bar_l_fill.config(bg=lc)
        self._bar_r_fill.config(bg=rc)

        # --- Pose ---
        self.lbl_x.config(text=f"{x:+.2f} m")
        self.lbl_y.config(text=f"{y:+.2f} m")
        deg = math.degrees(theta) % 360
        self.lbl_ang.config(text=f"{deg:.1f} deg")

        # --- Status Stadia ---
        if ok:
            self.lbl_stadia.config(text="● Conectado", fg=FG_GREEN)
        else:
            self.lbl_stadia.config(text="● Sin senal", fg=FG_ACC)

        self.root.after(30, self._tick)   # ~33 fps

    # --- Callbacks botones ---

    def _reset(self):
        with state.lock:
            state.x, state.y, state.theta = 0.0, 0.0, math.pi / 2
        self.trail.clear()
        self.cv.delete("trail")

    def _clear_trail(self):
        self.trail.clear()
        self.cv.delete("trail")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    root = tk.Tk()
    app  = App(root)
    root.mainloop()
