#!/usr/bin/env python3
"""
robot_interface.py  —  Interfaz principal del robot (Pi5)
Smart Golf Trolley · MOTOR-INTERFACE-V14

Comunicación: serial USB directo al Arduino Mega (115200 baud)
Plataforma:   Raspberry Pi 5  /  cualquier Linux o Windows con Python 3

Uso:
    python3 robot_interface.py [PUERTO]
    python3 robot_interface.py /dev/ttyACM0
    python3 robot_interface.py COM4          # Windows
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import re
import sys
import time

# ── Tema visual ─────────────────────────────────────────────────────────────────
BG       = "#0d0d0d"
PANEL_BG = "#141414"
PLOT_BG  = "#0a0a0a"
TEXT     = "#e0e0e0"
C_GREEN  = "#39d353"
C_RED    = "#f85149"
C_YELLOW = "#e3b341"
C_CYAN   = "#58a6ff"
C_BLUE   = "#388bfd"
C_DIM    = "#555555"

FONT_MONO = ("Consolas", 10)
FONT_MED  = ("Consolas", 12, "bold")
FONT_HUGE = ("Consolas", 22, "bold")

BAUD                 = 115200
DEFAULT_PORT_LINUX   = "/dev/ttyACM0"
DEFAULT_PORT_WINDOWS = "COM4"


# ── Helper ──────────────────────────────────────────────────────────────────────
def _btn(parent, text, cmd, bg, fg, **kw):
    return tk.Button(
        parent, text=text, command=cmd,
        bg=bg, fg=fg, activebackground=bg, activeforeground=fg,
        font=("Consolas", 10, "bold"), relief="flat", bd=0,
        cursor="hand2", **kw
    )


# ── Ventana principal ───────────────────────────────────────────────────────────
class RobotInterface:
    def __init__(self, root: tk.Tk, port_hint: str = ""):
        self.root = root
        self.root.title("Robot Interface  —  Smart Golf Trolley")
        self.root.configure(bg=BG)
        self.root.geometry("900x620")
        self.root.minsize(760, 540)

        # Serial
        self.ser        = None
        self.connected  = False
        self.rx_thread  = None
        self._stop_rx   = threading.Event()

        # Estado del robot
        self.balance_active = False
        self.front_limit    = 35.0
        self.rear_limit     = 35.0

        self._build_ui()

        # Puerto sugerido por argumento
        if port_hint:
            self.port_var.set(port_hint)

    # ── Construcción UI ─────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Barra superior ──────────────────────────────────────────────────────
        top = tk.Frame(self.root, bg=PANEL_BG, pady=6)
        top.pack(fill="x")
        tk.Label(top, text="⚙  SMART GOLF TROLLEY", bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 13, "bold")).pack(side="left", padx=12)
        self.lbl_status = tk.Label(top, text="● DESCONECTADO",
                                   bg=PANEL_BG, fg=C_RED, font=FONT_MONO)
        self.lbl_status.pack(side="right", padx=12)

        # ── Cuerpo ──────────────────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=8, pady=(4, 8))
        body.grid_columnconfigure(0, weight=1, minsize=290)
        body.grid_columnconfigure(1, weight=2)
        body.grid_rowconfigure(0, weight=1)

        self._build_left(body)
        self._build_right(body)

    # ── Panel izquierdo: conexión + balance ─────────────────────────────────────
    def _build_left(self, parent):
        lf = tk.Frame(parent, bg=PANEL_BG)
        lf.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        PAD = {"padx": 10}

        # ── Sección: Conexión serial ────────────────────────────────────────────
        self._section(lf, "CONEXIÓN SERIAL")

        pf = tk.Frame(lf, bg=PANEL_BG)
        pf.pack(fill="x", **PAD, pady=(0, 4))
        tk.Label(pf, text="Puerto:", bg=PANEL_BG, fg=C_DIM,
                 font=FONT_MONO).pack(side="left")
        self.port_var   = tk.StringVar()
        self.port_combo = ttk.Combobox(pf, textvariable=self.port_var,
                                       width=15, font=FONT_MONO)
        self.port_combo.pack(side="left", padx=4)
        tk.Button(pf, text="↻", font=FONT_MONO, bg=PANEL_BG, fg=C_CYAN,
                  bd=0, cursor="hand2",
                  command=self._refresh_ports).pack(side="left")
        self._refresh_ports()

        self.btn_connect = _btn(lf, "▶  CONECTAR", self._toggle_connection,
                                "#1a3a1a", C_GREEN, pady=7)
        self.btn_connect.pack(fill="x", **PAD, pady=(0, 8))

        ttk.Separator(lf, orient="horizontal").pack(fill="x", padx=8, pady=4)

        # ── Sección: Auto-balanceo ──────────────────────────────────────────────
        self._section(lf, "AUTO-BALANCEO")

        # Estado grande
        self.lbl_bal_state = tk.Label(lf, text="INACTIVO",
                                      font=FONT_HUGE, bg=PANEL_BG, fg=C_DIM)
        self.lbl_bal_state.pack(pady=(8, 6))

        # Botón ON/OFF prominente
        self.btn_balance = _btn(lf, "⚡  ACTIVAR BALANCEO",
                                self._toggle_balance,
                                "#1a3a1a", C_GREEN, pady=13)
        self.btn_balance.pack(fill="x", **PAD, pady=(0, 10))

        # Telemetría rápida: pitch + límites
        tf = tk.Frame(lf, bg=PANEL_BG)
        tf.pack(fill="x", **PAD, pady=(0, 4))

        def _row(parent, label, var_name, color):
            r = tk.Frame(parent, bg=PANEL_BG)
            r.pack(fill="x", pady=1)
            tk.Label(r, text=label, bg=PANEL_BG, fg=C_DIM,
                     font=FONT_MONO, width=8, anchor="w").pack(side="left")
            lbl = tk.Label(r, text="---", bg=PANEL_BG,
                           fg=color, font=FONT_MED, anchor="w")
            lbl.pack(side="left")
            setattr(self, var_name, lbl)

        _row(tf, "Pitch:",  "lbl_pitch", TEXT)
        _row(tf, "Front:",  "lbl_fl",    C_RED)
        _row(tf, "Rear:",   "lbl_rl",    C_YELLOW)

        ttk.Separator(lf, orient="horizontal").pack(fill="x", padx=8, pady=6)

        # ── Sección: Calibración ────────────────────────────────────────────────
        self._section(lf, "CALIBRACIÓN")

        for txt, cmd, fg in [
            ("⬤  Calibrar neutro",    "hb cal",       C_GREEN),
            ("▲  Calibrar front",     "hb cal front", C_RED),
            ("▼  Calibrar rear",      "hb cal rear",  C_YELLOW),
        ]:
            _btn(lf, txt, lambda c=cmd: self._send(c),
                 PANEL_BG, fg, pady=5, relief="groove", bd=1
                 ).pack(fill="x", **PAD, pady=2)

        # ── Spacer push-down ────────────────────────────────────────────────────
        tk.Frame(lf, bg=PANEL_BG).pack(fill="both", expand=True)

    # ── Panel derecho: telemetría + consola ─────────────────────────────────────
    def _build_right(self, parent):
        rf = tk.Frame(parent, bg=PANEL_BG)
        rf.grid(row=0, column=1, sticky="nsew")

        # ── Telemetría en vivo ──────────────────────────────────────────────────
        self._section(rf, "TELEMETRÍA EN VIVO")

        trow = tk.Frame(rf, bg=PANEL_BG)
        trow.pack(fill="x", padx=10, pady=(0, 6))

        self.lbl_corr = tk.Label(trow, text="Corrección: ---",
                                 bg=PANEL_BG, fg=TEXT, font=FONT_MONO)
        self.lbl_lrpm = tk.Label(trow, text="RPM L: ---",
                                 bg=PANEL_BG, fg=C_BLUE, font=FONT_MONO)
        self.lbl_rrpm = tk.Label(trow, text="RPM R: ---",
                                 bg=PANEL_BG, fg=C_YELLOW, font=FONT_MONO)
        for w in (self.lbl_corr, self.lbl_lrpm, self.lbl_rrpm):
            w.pack(side="left", padx=12)

        ttk.Separator(rf, orient="horizontal").pack(fill="x", padx=8, pady=4)

        # ── Consola serial ──────────────────────────────────────────────────────
        self._section(rf, "CONSOLA SERIAL")

        cf = tk.Frame(rf, bg=PANEL_BG)
        cf.pack(fill="both", expand=True, padx=10, pady=(0, 4))

        self.console = tk.Text(
            cf, bg=PLOT_BG, fg=TEXT, font=("Consolas", 9),
            insertbackground=TEXT, relief="flat", bd=0,
            wrap="char", state=tk.DISABLED
        )
        sb = tk.Scrollbar(cf, command=self.console.yview,
                          bg=PANEL_BG, troughcolor=PANEL_BG, bd=0, relief="flat")
        self.console.config(yscrollcommand=sb.set)
        self.console.tag_config("bal",  foreground=C_GREEN)
        self.console.tag_config("err",  foreground=C_RED)
        self.console.tag_config("warn", foreground=C_YELLOW)
        self.console.tag_config("dim",  foreground=C_DIM)
        self.console.tag_config("tx",   foreground=C_CYAN)
        sb.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)

        # ── Entrada manual ──────────────────────────────────────────────────────
        ef = tk.Frame(rf, bg=PANEL_BG)
        ef.pack(fill="x", padx=10, pady=(0, 8))
        self.entry_cmd = tk.Entry(
            ef, bg=PLOT_BG, fg=C_CYAN, font=FONT_MONO,
            insertbackground=C_CYAN, relief="flat", bd=2
        )
        self.entry_cmd.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.entry_cmd.bind("<Return>", lambda _e: self._send_entry())
        _btn(ef, "Enviar", self._send_entry, "#1a2a3a", C_CYAN, pady=5
             ).pack(side="left")

    # ── Helper de sección ───────────────────────────────────────────────────────
    def _section(self, parent, title: str):
        tk.Label(parent, text=title, bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 8, "bold")).pack(
            anchor="w", padx=10, pady=(8, 2))

    # ── Puertos ─────────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = [DEFAULT_PORT_LINUX, DEFAULT_PORT_WINDOWS]
        self.port_combo["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    # ── Conexión ─────────────────────────────────────────────────────────────────
    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        try:
            self.ser = serial.Serial(port, BAUD, timeout=1)
            time.sleep(2)   # esperar reset del Arduino
            self.connected = True
            self._stop_rx.clear()
            self.rx_thread = threading.Thread(
                target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            self.lbl_status.config(text=f"● {port}", fg=C_GREEN)
            self.btn_connect.config(
                text="■  DESCONECTAR",
                bg="#3a1a1a", fg=C_RED,
                activebackground="#4a1a1a", activeforeground=C_RED)
            self._log(f"[SYS] Conectado a {port} @ {BAUD}", "bal")
            self._send("hb stat")
        except serial.SerialException as ex:
            self._log(f"[ERR] {ex}", "err")

    def _disconnect(self):
        self._stop_rx.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.lbl_status.config(text="● DESCONECTADO", fg=C_RED)
        self.btn_connect.config(
            text="▶  CONECTAR",
            bg="#1a3a1a", fg=C_GREEN,
            activebackground="#1a4a1a", activeforeground=C_GREEN)
        self._log("[SYS] Desconectado", "warn")

    # ── Balance toggle ───────────────────────────────────────────────────────────
    def _toggle_balance(self):
        if not self.connected:
            self._log("[ERR] Sin conexión serial — conecta primero.", "err")
            return
        self._send("hb off" if self.balance_active else "hb on")

    def _set_balance_ui(self, active: bool):
        self.balance_active = active
        if active:
            self.lbl_bal_state.config(text="ACTIVO", fg=C_GREEN)
            self.btn_balance.config(
                text="⏹  DESACTIVAR BALANCEO",
                bg="#3a1a1a", fg=C_RED,
                activebackground="#4a1a1a", activeforeground=C_RED)
        else:
            self.lbl_bal_state.config(text="INACTIVO", fg=C_DIM)
            self.btn_balance.config(
                text="⚡  ACTIVAR BALANCEO",
                bg="#1a3a1a", fg=C_GREEN,
                activebackground="#1a4a1a", activeforeground=C_GREEN)

    # ── RX serial (hilo) ─────────────────────────────────────────────────────────
    def _rx_loop(self):
        buf = b""
        while not self._stop_rx.is_set():
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        raw = line.decode("utf-8", errors="replace").strip()
                        if raw:
                            self.root.after(0, self._process_line, raw)
                else:
                    time.sleep(0.02)
            except (serial.SerialException, OSError):
                self.root.after(0, self._disconnect)
                break

    def _process_line(self, raw: str):
        # Telemetría de balance (B line) — actualizar labels, no imprimir
        if raw.startswith("B "):
            self._parse_b_line(raw)
            return
        # Telemetría de velocidad (T line) — silenciar
        if raw.startswith("T "):
            return

        # Mensajes de balance
        if "ACTIVADO" in raw:
            self._set_balance_ui(True)
            self._log(raw, "bal")
        elif "DESACTIVADO" in raw:
            self._set_balance_ui(False)
            self._log(raw, "warn")
        elif "CAIDA" in raw:
            self._set_balance_ui(False)
            self._log(raw, "err")
        elif "ERROR" in raw:
            self._log(raw, "err")
        elif raw.startswith("[BAL]") or raw.startswith("[SYS]"):
            self._log(raw, "bal")
        else:
            self._log(raw, "dim")

    def _parse_b_line(self, raw: str):
        """Parsea la línea B del firmware:
        B err=X.XX gy=X.X cor=X.XXX base=X.XXX fin=X.XXX fl=35.0 rl=35.0 Lrpm=X Rrpm=X
        """
        def _f(key):
            m = re.search(rf"{key}=(-?[\d.]+)", raw)
            return float(m.group(1)) if m else None

        err  = _f("err")
        cor  = _f("cor")
        lrpm = _f("Lrpm")
        rrpm = _f("Rrpm")
        fl   = _f("fl")
        rl   = _f("rl")

        if err is not None:
            abs_e = abs(err)
            if (err > 0 and abs_e > self.front_limit * 0.8) or \
               (err < 0 and abs_e > self.rear_limit  * 0.8):
                c = C_RED
            elif abs_e > 1.5:
                c = C_YELLOW
            else:
                c = C_GREEN
            self.lbl_pitch.config(text=f"{err:+.2f}°", fg=c)

        if cor  is not None:
            self.lbl_corr.config(text=f"Corrección: {cor:+.3f} m/s")
        if lrpm is not None:
            self.lbl_lrpm.config(text=f"RPM L: {int(lrpm)}")
        if rrpm is not None:
            self.lbl_rrpm.config(text=f"RPM R: {int(rrpm)}")
        if fl is not None:
            self.front_limit = fl
            self.lbl_fl.config(text=f"+{fl:.1f}°")
        if rl is not None:
            self.rear_limit = rl
            self.lbl_rl.config(text=f"-{rl:.1f}°")

    # ── Enviar comando ───────────────────────────────────────────────────────────
    def _send(self, cmd: str):
        if not self.connected or not self.ser or not self.ser.is_open:
            self._log("[ERR] Sin conexión", "err")
            return
        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
            self._log(f"> {cmd}", "tx")
        except serial.SerialException as ex:
            self._log(f"[ERR] {ex}", "err")

    def _send_entry(self):
        cmd = self.entry_cmd.get().strip()
        if cmd:
            self._send(cmd)
            self.entry_cmd.delete(0, tk.END)

    # ── Consola ──────────────────────────────────────────────────────────────────
    def _log(self, msg: str, tag: str = "dim"):
        def _do():
            self.console.config(state=tk.NORMAL)
            self.console.insert(tk.END, msg + "\n", tag)
            self.console.see(tk.END)
            n = int(self.console.index("end-1c").split(".")[0])
            if n > 500:
                self.console.delete("1.0", f"{n - 400}.0")
            self.console.config(state=tk.DISABLED)
        self.root.after(0, _do)

    # ── Cierre limpio ────────────────────────────────────────────────────────────
    def on_close(self):
        self._disconnect()
        self.root.destroy()


# ── Entry point ─────────────────────────────────────────────────────────────────
def main():
    port_hint = sys.argv[1] if len(sys.argv) > 1 else ""
    root = tk.Tk()
    app = RobotInterface(root, port_hint)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
