#!/usr/bin/env python3
"""
motor_monitor.py  —  Corriente + Velocidad en tiempo real
Smart Golf Trolley · MOTOR-INTERFACE-V14

Controla el robot con el Stadia Y muestra simultáneamente:
  ┌──────────────────────────┬──────────────────────────┐
  │ MOTOR IZQ — RPM + I(A)  │ MOTOR DER — RPM + I(A)   │
  │  eje izq=RPM, der=Amps  │  eje izq=RPM, der=Amps   │
  ├──────────────────────────┼──────────────────────────┤
  │ IZQ — curva de carga     │ DER — curva de carga     │
  │  I(A) vs RPM (scatter)   │  I(A) vs RPM (scatter)   │
  └──────────────────────────┴──────────────────────────┘

Stadia:  L-Stick=mover   BTN_Y=balance on/off   BTN_A=STOP

Uso:
    python motor_monitor.py [PUERTO]
    python motor_monitor.py COM4
    python motor_monitor.py /dev/ttyACM0
"""

import sys
import time
import threading
from collections import deque

import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

import serial
import serial.tools.list_ports

try:
    import pywinusb.hid as hid
    _HID_OK = True
except ImportError:
    _HID_OK = False

# ── Configuración ─────────────────────────────────────────────────────────────
WINDOW_SECONDS = 20       # segundos de historial en los gráficos de tiempo
UPDATE_MS      = 80       # intervalo de animación (ms)
SCATTER_N      = 200      # puntos en el scatter I vs RPM
MAX_RPM_PLOT   = 250      # escala Y del eje RPM
MAX_CURRENT    = 12.0     # escala Y del eje corriente (A)
BAUD           = 115200
SEND_RATE_HZ   = 10
MAX_LINEAR     = 0.5      # m/s
MAX_ANGULAR    = 1.5      # rad/s
DEADZONE       = 0.12

# Stadia
STADIA_VID   = 0x18D1
STADIA_PID   = 0x9400
IDX_BTN2     = 3
IDX_LSX, IDX_LSY = 4, 5
AXIS_CENTER  = 128
AXIS_RANGE   = 127.0
BTN_A        = 0x08
BTN_Y        = 0x20
BTN_MENU     = 0x40

# ── Colores ───────────────────────────────────────────────────────────────────
BG       = "#0d0d0d"
PANEL_BG = "#141414"
PLOT_BG  = "#0a0a0a"
TEXT     = "#e0e0e0"
C_GREEN  = "#39d353"
C_RED    = "#f85149"
C_YELLOW = "#e3b341"
C_CYAN   = "#58a6ff"
C_BLUE   = "#388bfd"
C_ORANGE = "#ff8c42"
C_DIM    = "#444444"
GRID_C   = "#1e1e1e"

# ── Stadia HID ────────────────────────────────────────────────────────────────
_hid_lock  = threading.Lock()
_hid_state = {
    "lsx": 128, "lsy": 128,
    "btn2": 0,  "connected": False
}

def _hid_callback(data):
    if len(data) < 10 or data[0] != 0x03:
        return
    with _hid_lock:
        _hid_state["btn2"] = data[IDX_BTN2]
        _hid_state["lsx"]  = data[IDX_LSX]
        _hid_state["lsy"]  = data[IDX_LSY]
        _hid_state["connected"] = True

def _stadia_open():
    if not _HID_OK:
        return None
    devs = [d for d in hid.find_all_hid_devices()
            if d.vendor_id == STADIA_VID and d.product_id == STADIA_PID]
    if not devs:
        return None
    dev = devs[0]
    dev.open()
    dev.set_raw_data_handler(_hid_callback)
    return dev

def _stadia_get():
    with _hid_lock:
        return dict(_hid_state)

def _axis(raw):
    return (raw - AXIS_CENTER) / AXIS_RANGE

def _dz(v, dz=DEADZONE):
    if abs(v) < dz:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - dz) / (1.0 - dz)


# ── Monitor principal ─────────────────────────────────────────────────────────
class MotorMonitor:
    def __init__(self, root: tk.Tk, port: str):
        self.root = root
        self.port = port
        self.root.title(f"Motor Monitor  —  Corriente vs Velocidad  —  {port}")
        self.root.configure(bg=BG)
        self.root.geometry("1180x700")
        self.root.minsize(960, 580)

        maxlen = int(WINDOW_SECONDS * 25)

        # Buffers de serie temporal
        self.t_buf    = deque(maxlen=maxlen)
        self.lrpm_buf = deque(maxlen=maxlen)
        self.rrpm_buf = deque(maxlen=maxlen)
        self.lma_buf  = deque(maxlen=maxlen)
        self.rma_buf  = deque(maxlen=maxlen)

        # Buffers del scatter (I vs RPM)
        self.ls_x = deque(maxlen=SCATTER_N)   # RPM izq (abs)
        self.ls_y = deque(maxlen=SCATTER_N)   # I izq (abs)
        self.rs_x = deque(maxlen=SCATTER_N)   # RPM der (abs)
        self.rs_y = deque(maxlen=SCATTER_N)   # I der (abs)

        self.t0             = time.time()
        self.balance_active = False
        self._stop          = threading.Event()

        # Serial
        self.ser = None
        try:
            self.ser = serial.Serial(port, BAUD, timeout=1)
            time.sleep(2)
            while self.ser.in_waiting:
                self.ser.readline()          # descartar boot msgs
        except serial.SerialException as e:
            print(f"[ERR] Serial: {e}")

        # Stadia HID
        self.stadia_dev = _stadia_open()
        self.stadia_ok  = self.stadia_dev is not None

        # Construir GUI
        self._build_gui()

        # Hilos de fondo
        threading.Thread(target=self._rx_loop, daemon=True).start()
        if self.stadia_ok:
            threading.Thread(target=self._stadia_loop, daemon=True).start()

    # ── GUI ───────────────────────────────────────────────────────────────────
    def _build_gui(self):
        plt.style.use("dark_background")

        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=4, pady=4)
        body.grid_columnconfigure(0, weight=1)
        body.grid_columnconfigure(1, weight=0, minsize=230)
        body.grid_rowconfigure(0, weight=1)

        # ── Plots ──────────────────────────────────────────────────────────
        pf = tk.Frame(body, bg=BG)
        pf.grid(row=0, column=0, sticky="nsew")

        self.fig = plt.figure(figsize=(10.5, 6.2), facecolor=BG)
        gs = gridspec.GridSpec(2, 2, figure=self.fig,
                               hspace=0.46, wspace=0.44,
                               left=0.07, right=0.96,
                               top=0.94,  bottom=0.09)

        # ── Helper: crear eje con twin Y ──────────────────────────────────
        def _make_ax(idx, title, cl_rpm, cl_curr):
            ax = self.fig.add_subplot(idx)
            ax.set_facecolor(PLOT_BG)
            ax.set_title(title, color=TEXT, fontsize=8.5, pad=3)
            ax.set_xlim(-WINDOW_SECONDS, 0.5)
            ax.set_ylim(-MAX_RPM_PLOT * 0.05, MAX_RPM_PLOT)
            ax.set_ylabel("RPM", color=cl_rpm, fontsize=7)
            ax.tick_params(axis="y", colors=cl_rpm, labelsize=6)
            ax.tick_params(axis="x", colors=C_DIM,  labelsize=6)
            ax.axhline(0, color=GRID_C, lw=0.5)
            ax.grid(True, color=GRID_C, lw=0.4, alpha=0.6)
            for sp in ax.spines.values():
                sp.set_edgecolor(GRID_C)

            ax2 = ax.twinx()
            ax2.set_ylim(-MAX_CURRENT * 0.05, MAX_CURRENT)
            ax2.set_ylabel("I (A)", color=cl_curr, fontsize=7)
            ax2.tick_params(axis="y", colors=cl_curr, labelsize=6)
            for sp in ax2.spines.values():
                sp.set_edgecolor(GRID_C)

            ln_rpm, = ax.plot([], [], color=cl_rpm,  lw=1.8, label="RPM")
            ln_ma,  = ax2.plot([], [], color=cl_curr, lw=1.5,
                                label="I(A)", alpha=0.9, ls="--")
            h1, l1 = ax.get_legend_handles_labels()
            h2, l2 = ax2.get_legend_handles_labels()
            ax2.legend(h1+h2, l1+l2, fontsize=6, loc="upper right",
                       framealpha=0.3, facecolor=PLOT_BG, labelcolor=TEXT)
            return ax, ax2, ln_rpm, ln_ma

        self.ax_l,  self.ax_lI, self.ln_lrpm, self.ln_lma = _make_ax(
            gs[0, 0], "MOTOR IZQ  —  RPM (──)  y  Corriente (- -)", C_BLUE, C_ORANGE)
        self.ax_r,  self.ax_rI, self.ln_rrpm, self.ln_rma = _make_ax(
            gs[0, 1], "MOTOR DER  —  RPM (──)  y  Corriente (- -)", C_YELLOW, C_RED)

        # ── Scatter: I vs |RPM| ───────────────────────────────────────────
        def _make_scatter(idx, title, cl):
            ax = self.fig.add_subplot(idx)
            ax.set_facecolor(PLOT_BG)
            ax.set_title(title, color=TEXT, fontsize=8.5, pad=3)
            ax.set_xlabel("|RPM|", color=C_DIM,  fontsize=7)
            ax.set_ylabel("|I| (A)", color=cl, fontsize=7)
            ax.set_xlim(0, MAX_RPM_PLOT)
            ax.set_ylim(0, MAX_CURRENT)
            ax.tick_params(colors=C_DIM, labelsize=6)
            ax.grid(True, color=GRID_C, lw=0.4, alpha=0.6)
            for sp in ax.spines.values():
                sp.set_edgecolor(GRID_C)
            # trail: línea fina + punto brillante al frente
            trail, = ax.plot([], [], color=cl, lw=1.0, alpha=0.5,
                             marker="o", ms=1.5)
            dot,   = ax.plot([], [], "o", color=cl, ms=8,
                             markeredgecolor="white", markeredgewidth=0.5,
                             zorder=6)
            # línea de referencia de sobrecarga
            ax.axhline(8.0, color=C_RED, lw=0.7, ls=":", alpha=0.6)
            ax.text(MAX_RPM_PLOT * 0.02, 8.3, "sobrecarga 8A",
                    color=C_RED, fontsize=5.5, alpha=0.7)
            return ax, trail, dot

        self.ax_ls, self.sc_l_trail, self.sc_l_dot = _make_scatter(
            gs[1, 0], "IZQ — Curva de carga  |I| vs |RPM|", C_ORANGE)
        self.ax_rs, self.sc_r_trail, self.sc_r_dot = _make_scatter(
            gs[1, 1], "DER — Curva de carga  |I| vs |RPM|", C_RED)

        self.canvas = FigureCanvasTkAgg(self.fig, master=pf)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self._anim = FuncAnimation(
            self.fig, self._animate,
            interval=UPDATE_MS, blit=False, cache_frame_data=False
        )

        # ── Panel derecho ──────────────────────────────────────────────────
        self._build_panel(body)

    def _build_panel(self, parent):
        pf = tk.Frame(parent, bg=PANEL_BG, width=230)
        pf.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        pf.grid_propagate(False)

        def _sec(txt):
            tk.Label(pf, text=txt, bg=PANEL_BG, fg=C_CYAN,
                     font=("Consolas", 8, "bold")).pack(
                anchor="w", padx=10, pady=(10, 2))

        def _num_row(label, attr_name, color):
            r = tk.Frame(pf, bg=PANEL_BG)
            r.pack(fill="x", padx=10, pady=1)
            tk.Label(r, text=label, bg=PANEL_BG, fg=C_DIM,
                     font=("Consolas", 8), width=6, anchor="w").pack(side="left")
            lbl = tk.Label(r, text="----", bg=PANEL_BG, fg=color,
                           font=("Consolas", 16, "bold"), anchor="w")
            lbl.pack(side="left")
            setattr(self, attr_name, lbl)

        # Estado
        _sec("ESTADO")
        conn_txt = f"● {self.port} OK" if (self.ser and self.ser.is_open) \
                   else f"● {self.port} ERR"
        conn_clr = C_GREEN if (self.ser and self.ser.is_open) else C_RED
        self.lbl_conn = tk.Label(pf, text=conn_txt, bg=PANEL_BG,
                                 fg=conn_clr, font=("Consolas", 8))
        self.lbl_conn.pack(anchor="w", padx=10)

        stadia_txt = "⚡ Stadia conectado" if self.stadia_ok else "⚠  Sin Stadia (solo monitor)"
        stadia_clr = C_GREEN if self.stadia_ok else C_YELLOW
        tk.Label(pf, text=stadia_txt, bg=PANEL_BG,
                 fg=stadia_clr, font=("Consolas", 8)).pack(anchor="w", padx=10, pady=(1, 0))

        self.lbl_bal = tk.Label(pf, text="Balance: INACTIVO",
                                bg=PANEL_BG, fg=C_DIM, font=("Consolas", 8))
        self.lbl_bal.pack(anchor="w", padx=10, pady=(1, 0))

        ttk.Separator(pf, orient="horizontal").pack(fill="x", padx=6, pady=6)

        # Lecturas instantáneas
        _sec("MOTOR IZQUIERDO")
        _num_row("RPM", "lbl_lrpm_n", C_BLUE)
        _num_row("I",   "lbl_lma_n",  C_ORANGE)

        ttk.Separator(pf, orient="horizontal").pack(fill="x", padx=6, pady=4)
        _sec("MOTOR DERECHO")
        _num_row("RPM", "lbl_rrpm_n", C_YELLOW)
        _num_row("I",   "lbl_rma_n",  C_RED)

        ttk.Separator(pf, orient="horizontal").pack(fill="x", padx=6, pady=6)

        # Balance toggle
        _sec("BALANCE")
        self.btn_bal = tk.Button(
            pf, text="⚡ ACTIVAR BALANCE",
            command=self._toggle_balance,
            bg="#1a3a1a", fg=C_GREEN,
            activebackground="#1a4a1a", activeforeground=C_GREEN,
            font=("Consolas", 9, "bold"),
            relief="flat", bd=0, pady=9, cursor="hand2"
        )
        self.btn_bal.pack(fill="x", padx=8, pady=2)

        ttk.Separator(pf, orient="horizontal").pack(fill="x", padx=6, pady=6)

        # Stadia hints
        _sec("MANDO STADIA")
        hints = [
            ("L-Stick ↑↓",  "Lineal"),
            ("L-Stick ←→",  "Giro"),
            ("BTN Y  ▲",    "Balance ON/OFF"),
            ("BTN A  ●",    "PARADA"),
            ("BTN MENU ☰",  "Estado"),
        ]
        for key, val in hints:
            r = tk.Frame(pf, bg=PANEL_BG)
            r.pack(fill="x", padx=8, pady=0)
            tk.Label(r, text=key, bg=PANEL_BG, fg=C_CYAN,
                     font=("Consolas", 7, "bold"), width=12, anchor="w"
                     ).pack(side="left")
            tk.Label(r, text=val, bg=PANEL_BG, fg=C_DIM,
                     font=("Consolas", 7), anchor="w").pack(side="left")

        # Spacer + cerrar
        tk.Frame(pf, bg=PANEL_BG).pack(fill="both", expand=True)
        tk.Button(pf, text="✕  Cerrar",
                  command=self.on_close,
                  bg="#3a1a1a", fg=C_RED,
                  activebackground="#4a1a1a", activeforeground=C_RED,
                  font=("Consolas", 9, "bold"),
                  relief="flat", bd=0, pady=7, cursor="hand2"
                  ).pack(fill="x", padx=8, pady=(0, 10))

    # ── Animación ─────────────────────────────────────────────────────────────
    def _animate(self, _frame):
        if len(self.t_buf) < 2:
            return

        now  = time.time() - self.t0
        t_rel   = [v - now for v in self.t_buf]
        lrpm    = list(self.lrpm_buf)
        rrpm    = list(self.rrpm_buf)
        lma     = list(self.lma_buf)
        rma     = list(self.rma_buf)
        t_min   = -WINDOW_SECONDS

        # Series temporales IZQ
        self.ln_lrpm.set_data(t_rel, lrpm)
        self.ln_lma.set_data( t_rel, lma)
        self.ax_l.set_xlim(t_min, 0.5)

        # Series temporales DER
        self.ln_rrpm.set_data(t_rel, rrpm)
        self.ln_rma.set_data( t_rel, rma)
        self.ax_r.set_xlim(t_min, 0.5)

        # Scatter IZQ
        lsx = list(self.ls_x)
        lsy = list(self.ls_y)
        if lsx:
            self.sc_l_trail.set_data(lsx, lsy)
            self.sc_l_dot.set_data([lsx[-1]], [lsy[-1]])

        # Scatter DER
        rsx = list(self.rs_x)
        rsy = list(self.rs_y)
        if rsx:
            self.sc_r_trail.set_data(rsx, rsy)
            self.sc_r_dot.set_data([rsx[-1]], [rsy[-1]])

        # Números en panel
        if lrpm:
            self._refresh_nums(
                lrpm[-1], rrpm[-1],
                lma[-1],  rma[-1]
            )

    def _refresh_nums(self, lr, rr, la, ra):
        def _cc(a):
            aa = abs(a)
            return C_RED if aa > 8.0 else (C_YELLOW if aa > 4.0 else C_GREEN)

        self.lbl_lrpm_n.config(text=f"{int(lr):4d}")
        self.lbl_rrpm_n.config(text=f"{int(rr):4d}")
        self.lbl_lma_n.config( text=f"{la:+.2f}A", fg=_cc(la))
        self.lbl_rma_n.config( text=f"{ra:+.2f}A", fg=_cc(ra))

    # ── RX serial ─────────────────────────────────────────────────────────────
    def _rx_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        raw = line.decode("utf-8", errors="replace").strip()
                        if raw:
                            self._process_line(raw)
                else:
                    time.sleep(0.02)
            except (serial.SerialException, OSError):
                break

    def _process_line(self, raw: str):
        # Parsear T y B por igual (ambas contienen Lrpm, Rrpm, LmA, RmA)
        if raw.startswith("T ") or raw.startswith("B "):
            d = {}
            for tok in raw[2:].split():
                if "=" in tok:
                    k, _, v = tok.partition("=")
                    try:
                        d[k] = float(v)
                    except ValueError:
                        pass
            lr = d.get("Lrpm")
            rr = d.get("Rrpm")
            la = d.get("LmA")
            ra = d.get("RmA")
            if lr is not None:
                now = time.time() - self.t0
                self.t_buf.append(now)
                self.lrpm_buf.append(lr)
                self.rrpm_buf.append(rr if rr is not None else 0.0)
                self.lma_buf.append( la if la is not None else 0.0)
                self.rma_buf.append( ra if ra is not None else 0.0)
                # Scatter con valores absolutos (curva de carga)
                if la is not None:
                    self.ls_x.append(abs(lr))
                    self.ls_y.append(abs(la))
                if rr is not None and ra is not None:
                    self.rs_x.append(abs(rr))
                    self.rs_y.append(abs(ra))
            return

        # Estado del balance
        if "ACTIVADO" in raw:
            self.balance_active = True
            self.root.after(0, lambda: self.lbl_bal.config(
                text="Balance: ACTIVO ✓", fg=C_GREEN))
            self.root.after(0, lambda: self.btn_bal.config(
                text="⏹  DESACTIVAR BALANCE",
                bg="#3a1a1a", fg=C_RED,
                activebackground="#4a1a1a", activeforeground=C_RED))
        elif "DESACTIVADO" in raw or "CAIDA" in raw:
            self.balance_active = False
            self.root.after(0, lambda: self.lbl_bal.config(
                text="Balance: INACTIVO", fg=C_DIM))
            self.root.after(0, lambda: self.btn_bal.config(
                text="⚡ ACTIVAR BALANCE",
                bg="#1a3a1a", fg=C_GREEN,
                activebackground="#1a4a1a", activeforeground=C_GREEN))

    # ── Stadia loop ───────────────────────────────────────────────────────────
    def _stadia_loop(self):
        interval   = 1.0 / SEND_RATE_HZ
        last_send  = 0.0
        prev_btn2  = 0

        while not self._stop.is_set():
            now = time.monotonic()
            s   = _stadia_get()
            btn2    = s["btn2"]
            pressed = btn2 & ~prev_btn2
            prev_btn2 = btn2

            # Parada de emergencia
            if btn2 & BTN_A:
                self._tx(b"v 0.0 0.0\n")
                time.sleep(0.1)
                continue

            # BTN_Y → toggle balance
            if pressed & BTN_Y:
                self._tx(b"hb off\n" if self.balance_active else b"hb on\n")

            # BTN_MENU → estado
            if pressed & BTN_MENU:
                self._tx(b"s\n")

            # Velocidad al ritmo configurado
            if now - last_send >= interval:
                lx = _axis(s["lsx"])
                ly = _axis(s["lsy"])
                lin  = _dz(-ly) * MAX_LINEAR
                ang  = _dz(-lx) * MAX_ANGULAR
                self._tx(f"v {lin:.3f} {ang:.3f}\n".encode())
                last_send = now

            time.sleep(0.005)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _tx(self, cmd: bytes):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd)
            except serial.SerialException:
                pass

    def _toggle_balance(self):
        self._tx(b"hb off\n" if self.balance_active else b"hb on\n")

    def on_close(self):
        self._stop.set()
        self._tx(b"v 0.0 0.0\n")
        time.sleep(0.3)
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.stadia_dev:
            try:
                self.stadia_dev.close()
            except Exception:
                pass
        plt.close("all")
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    root = tk.Tk()
    app  = MotorMonitor(root, port)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
