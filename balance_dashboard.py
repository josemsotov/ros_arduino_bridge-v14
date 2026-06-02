#!/usr/bin/env python3
"""
balance_dashboard.py
====================
Dashboard gráfico en tiempo real para calibración y pruebas del
controlador anti-caída PD del Smart Golf Trolley.

USO:
    python balance_dashboard.py [PUERTO] [--baud BAUD]
    python balance_dashboard.py COM4
    python balance_dashboard.py COM4 --baud 115200

TELEMETRÍA ESPERADA DEL ARDUINO:
    B err=X.XX gy=X.X cor=X.XXX base=X.XXX fin=X.XXX Lrpm=X Rrpm=X
    T lin=X ang=X Lpwm=X Rpwm=X Lrpm=X Rrpm=X Ld=F Rd=F
    [BAL] ...  (mensajes de estado)

CONTROLES:
    • Botones ON / OFF / CAL / STAT
    • Sliders Kp y Kd con botón "Aplicar" para enviar al robot
    • Sliders de velocidad lineal y angular + botón STOP
    • Entrada manual de comandos serial
"""

import sys
import argparse
import threading
import queue
import time
import math
from collections import deque

import tkinter as tk
from tkinter import ttk, scrolledtext

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

import serial

# ─── Configuración ────────────────────────────────────────────────────────────
WINDOW_SECONDS = 15      # Ventana de tiempo visible en las gráficas (s)
UPDATE_MS      = 120     # Intervalo de refresco de la animación (ms)
MAX_PITCH_PLOT = 40.0    # Rango del eje Y en plot de error
DEAD_ZONE      = 1.5     # Zona muerta °  (para sombreado visual)
FALL_ANGLE     = 35.0    # Ángulo de caída ° (para línea de peligro)
MAX_CORRECTION = 0.40    # Corrección máxima m/s (para límite en gráfica)
MAX_RPM_PLOT   = 80      # RPM máximo para escala inicial

# ─── Paleta de colores (Catppuccin Mocha) ─────────────────────────────────────
BG_COLOR    = "#1e1e2e"
PANEL_BG    = "#2a2a3e"
PLOT_BG     = "#181825"
GRID_COLOR  = "#585b70"
TEXT_COLOR  = "#cdd6f4"
C_BLUE      = "#89b4fa"
C_RED       = "#f38ba8"
C_GREEN     = "#a6e3a1"
C_YELLOW    = "#f9e2af"
C_PURPLE    = "#cba6f7"
C_CYAN      = "#94e2d5"
C_MAUVE     = "#cba6f7"


# ─── Utilidades ───────────────────────────────────────────────────────────────
def _style_axes(ax, title):
    ax.set_facecolor(PLOT_BG)
    ax.set_title(title, color=TEXT_COLOR, fontsize=8.5, pad=4)
    ax.tick_params(colors=GRID_COLOR, labelsize=7)
    ax.set_xlabel("t (s)", color=GRID_COLOR, fontsize=7)
    for sp in ax.spines.values():
        sp.set_color(GRID_COLOR)
    ax.grid(color=GRID_COLOR, lw=0.4, alpha=0.5)


def _button(parent, text, cmd, bg, fg, padx=8, pady=5, **kwargs):
    return tk.Button(
        parent, text=text, command=cmd,
        bg=bg, fg=fg, activebackground=bg,
        font=("Consolas", 8, "bold"), relief="flat",
        cursor="hand2", bd=0, padx=padx, pady=pady, **kwargs
    )


# ─── Dashboard principal ───────────────────────────────────────────────────────
class BalanceDashboard:
    def __init__(self, port: str, baud: int):
        self.port    = port
        self.baud    = baud
        self.ser     = None
        self.connected = False
        self.running   = True

        # Colas thread-safe
        self.data_queue = queue.Queue(maxsize=2000)
        self.cmd_queue  = queue.Queue()

        # Buffers de telemetría (ventana temporal)
        maxlen = int(WINDOW_SECONDS * 25)
        self.t_buf    = deque(maxlen=maxlen)
        self.err_buf  = deque(maxlen=maxlen)
        self.gy_buf   = deque(maxlen=maxlen)
        self.cor_buf  = deque(maxlen=maxlen)
        self.base_buf = deque(maxlen=maxlen)
        self.fin_buf  = deque(maxlen=maxlen)
        self.lrpm_buf = deque(maxlen=maxlen)
        self.rrpm_buf = deque(maxlen=maxlen)

        # Límites de inclinación actuales (se actualizan desde telemetría B)
        self.front_limit = FALL_ANGLE
        self.rear_limit  = FALL_ANGLE

        # Estado
        self.balance_active = False
        self.current_kp     = 0.025
        self.current_kd     = 0.002
        self.t0             = time.time()

        # ── Construir GUI ──────────────────────────────────────────────────
        self._build_gui()

        # ── Hilo serial ────────────────────────────────────────────────────
        self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.serial_thread.start()

    # =========================================================================
    # GUI
    # =========================================================================

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title(f"Balance Anti-Caída  —  Smart Golf Trolley  —  {self.port}")
        self.root.configure(bg=BG_COLOR)
        self.root.geometry("1300x780")
        self.root.minsize(900, 600)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Layout: izquierda (gráficas) + derecha (controles)
        outer = tk.Frame(self.root, bg=BG_COLOR)
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        self.left_frame  = tk.Frame(outer, bg=BG_COLOR)
        self.right_frame = tk.Frame(outer, bg=PANEL_BG, width=295)

        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
        self.right_frame.pack_propagate(False)

        self._build_plots()
        self._build_controls()

    # ──────────────────────────────────────────────────────────────────────────
    # Gráficas matplotlib
    # ──────────────────────────────────────────────────────────────────────────

    def _build_plots(self):
        plt.style.use("dark_background")
        self.fig = Figure(figsize=(9.2, 6.2), facecolor=BG_COLOR)
        gs = gridspec.GridSpec(
            2, 2, figure=self.fig,
            hspace=0.42, wspace=0.36,
            left=0.07, right=0.97, top=0.95, bottom=0.07
        )

        # ── Plot 1: Pitch Error ───────────────────────────────────────────
        self.ax_err = self.fig.add_subplot(gs[0, 0])
        _style_axes(self.ax_err, "Pitch Error  (°)")
        # Zona muerta sombreada en verde
        self.ax_err.axhspan(-DEAD_ZONE, DEAD_ZONE, alpha=0.12, color=C_GREEN, zorder=0)
        # Líneas de límite dinámicas (se actualizan en _animate con los valores del robot)
        self.hline_front, = self.ax_err.plot([], [], color=C_RED, lw=1.5, ls="--", alpha=0.85, zorder=2)
        self.hline_rear,  = self.ax_err.plot([], [], color=C_YELLOW, lw=1.5, ls="--", alpha=0.85, zorder=2)
        self.ax_err.axhline(0, color=GRID_COLOR, lw=0.6, zorder=1)
        self.ax_err.set_ylim(-MAX_PITCH_PLOT, MAX_PITCH_PLOT)
        self.lbl_front_line = self.ax_err.text(
            0.98, 0.97, f"FRONT {FALL_ANGLE:.0f}°",
            ha="right", va="top", transform=self.ax_err.transAxes,
            fontsize=6, color=C_RED, alpha=0.85
        )
        self.lbl_rear_line = self.ax_err.text(
            0.98, 0.03, f"REAR {FALL_ANGLE:.0f}°",
            ha="right", va="bottom", transform=self.ax_err.transAxes,
            fontsize=6, color=C_YELLOW, alpha=0.85
        )
        self.line_err, = self.ax_err.plot([], [], color=C_BLUE,   lw=1.8, label="error (°)", zorder=3)
        self.line_gy,  = self.ax_err.plot([], [], color=C_PURPLE, lw=1.0, ls="--",
                                           alpha=0.65, label="gyroY/10", zorder=2)
        self.ax_err.legend(fontsize=6, loc="upper right",
                           labelcolor=TEXT_COLOR, framealpha=0.25, facecolor=PLOT_BG)

        # ── Plot 2: Corrección ────────────────────────────────────────────
        self.ax_cor = self.fig.add_subplot(gs[0, 1])
        _style_axes(self.ax_cor, "Corrección anti-caída  (m/s)")
        self.ax_cor.axhline( MAX_CORRECTION, color=C_YELLOW, lw=0.9, ls="--",
                             alpha=0.6, zorder=1, label=f"±{MAX_CORRECTION} m/s lím.")
        self.ax_cor.axhline(-MAX_CORRECTION, color=C_YELLOW, lw=0.9, ls="--",
                             alpha=0.6, zorder=1)
        self.ax_cor.axhline(0, color=GRID_COLOR, lw=0.6, zorder=1)
        self.ax_cor.set_ylim(-MAX_CORRECTION * 1.4, MAX_CORRECTION * 1.4)
        self.line_cor,  = self.ax_cor.plot([], [], color=C_RED,   lw=1.8, label="correcc.", zorder=3)
        self.line_base, = self.ax_cor.plot([], [], color=C_CYAN,  lw=1.0, ls=":",
                                            alpha=0.8, label="base (usuario)", zorder=2)
        self.line_fin,  = self.ax_cor.plot([], [], color=C_GREEN, lw=1.5,
                                            alpha=0.85, label="final", zorder=2)
        self.ax_cor.legend(fontsize=6, loc="upper right",
                           labelcolor=TEXT_COLOR, framealpha=0.25, facecolor=PLOT_BG)

        # ── Plot 3: RPM ───────────────────────────────────────────────────
        self.ax_rpm = self.fig.add_subplot(gs[1, 0])
        _style_axes(self.ax_rpm, "RPM  Motores")
        self.ax_rpm.axhline(0, color=GRID_COLOR, lw=0.6, zorder=1)
        self.ax_rpm.set_ylim(-MAX_RPM_PLOT, MAX_RPM_PLOT)
        self.line_lrpm, = self.ax_rpm.plot([], [], color=C_BLUE,   lw=1.8, label="Izquierdo", zorder=3)
        self.line_rrpm, = self.ax_rpm.plot([], [], color=C_YELLOW, lw=1.8, label="Derecho",   zorder=3)
        self.ax_rpm.legend(fontsize=6, loc="upper right",
                           labelcolor=TEXT_COLOR, framealpha=0.25, facecolor=PLOT_BG)

        # ── Plot 4: Indicador de inclinación ──────────────────────────────
        self.ax_tilt = self.fig.add_subplot(gs[1, 1])
        self.ax_tilt.set_facecolor(PLOT_BG)
        self.ax_tilt.set_title("Inclinación del chasis", color=TEXT_COLOR, fontsize=8.5, pad=4)
        self.ax_tilt.set_xlim(-2.5, 2.5)
        self.ax_tilt.set_ylim(-1.0, 4.0)
        self.ax_tilt.axis("off")
        self._build_tilt_indicator()

        # Canvas embebido en tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.left_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Animación
        self._anim = FuncAnimation(
            self.fig, self._animate,
            interval=UPDATE_MS, blit=False, cache_frame_data=False
        )

    def _build_tilt_indicator(self):
        ax = self.ax_tilt
        # Suelo
        ax.plot([-2.2, 2.2], [-0.35, -0.35], color=GRID_COLOR, lw=2.5)
        # Ruedas estáticas (referencia)
        wl = plt.Circle((-0.55, -0.05), 0.28, color="#404060", zorder=2)
        wr = plt.Circle(( 0.55, -0.05), 0.28, color="#404060", zorder=2)
        ax.add_patch(wl)
        ax.add_patch(wr)
        # Zona muerta (arco verde)
        arc_d = patches.Arc((0, -0.05), 1.8, 1.8, angle=90,
                             theta1=-DEAD_ZONE, theta2=DEAD_ZONE,
                             color=C_GREEN, lw=2.0, alpha=0.5)
        ax.add_patch(arc_d)
        # Zona de caída (arcos rojos)
        arc_f1 = patches.Arc((0, -0.05), 2.4, 2.4, angle=90,
                              theta1=-self.front_limit, theta2=-DEAD_ZONE,
                              color=C_RED, lw=1.5, alpha=0.4, ls="--")
        arc_f2 = patches.Arc((0, -0.05), 2.4, 2.4, angle=90,
                              theta1=DEAD_ZONE, theta2=self.rear_limit,
                              color=C_YELLOW, lw=1.5, alpha=0.4, ls="--")
        ax.add_patch(arc_f1)
        ax.add_patch(arc_f2)
        # Cuerpo del robot (polígono rotante)
        verts = self._get_body_verts(0.0)
        self.body_poly = patches.Polygon(
            verts, closed=True,
            facecolor="#1a3a6a", edgecolor=C_BLUE, lw=2.0, zorder=4
        )
        ax.add_patch(self.body_poly)
        # Cabeza / sensor
        verts_h = self._get_head_verts(0.0)
        self.head_poly = patches.Polygon(
            verts_h, closed=True,
            facecolor="#2a2a5a", edgecolor=C_CYAN, lw=1.5, zorder=5
        )
        ax.add_patch(self.head_poly)
        # Texto de ángulo
        self.tilt_text = ax.text(
            0, 3.6, "0.0°",
            ha="center", va="center",
            fontsize=15, color=C_GREEN, fontweight="bold", zorder=6
        )
        # Etiqueta de estado
        self.tilt_state = ax.text(
            0, -0.75, "NEUTRO",
            ha="center", va="center",
            fontsize=7, color=C_GREEN, alpha=0.9, zorder=6
        )

    def _get_body_verts(self, angle_deg):
        rad = math.radians(angle_deg)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        w, h = 0.28, 2.4
        # Esquinas del cuerpo (pivote en base, 0.0)
        local = [(-w, 0.0), (w, 0.0), (w, h), (-w, h)]
        return [(x * cos_a - y * sin_a, x * sin_a + y * cos_a) for x, y in local]

    def _get_head_verts(self, angle_deg):
        rad = math.radians(angle_deg)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        w, h_start, h_end = 0.22, 2.4, 2.9
        local = [(-w, h_start), (w, h_start), (w, h_end), (-w, h_end)]
        return [(x * cos_a - y * sin_a, x * sin_a + y * cos_a) for x, y in local]

    def _update_tilt(self, angle_deg):
        self.body_poly.set_xy(self._get_body_verts(angle_deg))
        self.head_poly.set_xy(self._get_head_verts(angle_deg))
        abs_a = abs(angle_deg)
        if abs_a > self.front_limit * 0.8 or abs_a > self.rear_limit * 0.8:
            ec, fc, tc, state = C_RED,    "#4a1a1a", C_RED,    "⚠ PELIGRO"
        elif abs_a > DEAD_ZONE:
            ec, fc, tc, state = C_YELLOW, "#3a3a1a", C_YELLOW, "CORRECCIÓN"
        else:
            ec, fc, tc, state = C_BLUE,   "#1a3a6a", C_GREEN,  "NEUTRO"
        self.body_poly.set_edgecolor(ec)
        self.body_poly.set_facecolor(fc)
        self.tilt_text.set_text(f"{angle_deg:+.1f}°")
        self.tilt_text.set_color(tc)
        self.tilt_state.set_text(state)
        self.tilt_state.set_color(tc)

    # ──────────────────────────────────────────────────────────────────────────
    # Panel de controles
    # ──────────────────────────────────────────────────────────────────────────

    def _build_controls(self):
        f   = self.right_frame
        PAD = {"padx": 9, "pady": 3}

        # ── Estado ────────────────────────────────────────────────────────
        tk.Label(f, text="ESTADO", bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 8, "bold")).pack(padx=9, anchor="w", pady=(10, 1))

        self.lbl_conn = tk.Label(f, text="● Desconectado", bg=PANEL_BG,
                                 fg=C_RED, font=("Consolas", 9))
        self.lbl_conn.pack(**PAD, anchor="w")

        self.lbl_bal = tk.Label(f, text="Balance: —", bg=PANEL_BG,
                                fg=TEXT_COLOR, font=("Consolas", 8))
        self.lbl_bal.pack(**PAD, anchor="w")

        self.lbl_pitch = tk.Label(f, text="Pitch:  0.00°", bg=PANEL_BG,
                                  fg=C_BLUE, font=("Consolas", 12, "bold"))
        self.lbl_pitch.pack(**PAD, anchor="w")

        frm_nums = tk.Frame(f, bg=PANEL_BG)
        frm_nums.pack(**PAD, fill="x")
        self.lbl_corr = tk.Label(frm_nums, text="Corr: 0.000 m/s", bg=PANEL_BG,
                                 fg=C_YELLOW, font=("Consolas", 8))
        self.lbl_corr.pack(side=tk.LEFT)
        self.lbl_rpm = tk.Label(frm_nums, text="  L:0 R:0 RPM", bg=PANEL_BG,
                                fg=C_CYAN, font=("Consolas", 8))
        self.lbl_rpm.pack(side=tk.LEFT)

        ttk.Separator(f, orient="horizontal").pack(fill="x", padx=8, pady=5)

        # ── Balance ON / OFF / CAL / STAT ─────────────────────────────────
        tk.Label(f, text="CONTROL BALANCE", bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 8, "bold")).pack(**PAD, anchor="w")

        bf = tk.Frame(f, bg=PANEL_BG)
        bf.pack(**PAD, fill="x")
        _button(bf, "▶ ON",   lambda: self._send_cmd("hb on"),   "#1a4a1a", C_GREEN
                ).grid(row=0, column=0, padx=2, sticky="ew")
        _button(bf, "■ OFF",  lambda: self._send_cmd("hb off"),  "#4a1a1a", C_RED
                ).grid(row=0, column=1, padx=2, sticky="ew")
        _button(bf, "⊙ CAL",  lambda: self._send_cmd("hb cal"),  "#1a2a4a", C_BLUE,
                pady=4).grid(row=1, column=0, padx=2, pady=(4, 0), sticky="ew")
        _button(bf, "? STAT", lambda: self._send_cmd("hb stat"), "#3a3a1a", C_YELLOW,
                pady=4).grid(row=1, column=1, padx=2, pady=(4, 0), sticky="ew")
        bf.grid_columnconfigure(0, weight=1)
        bf.grid_columnconfigure(1, weight=1)

        ttk.Separator(f, orient="horizontal").pack(fill="x", padx=8, pady=5)

        # ── Ganancias PD ──────────────────────────────────────────────────
        tk.Label(f, text="GANANCIAS PD", bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 8, "bold")).pack(**PAD, anchor="w")

        # Kp
        kf = tk.Frame(f, bg=PANEL_BG)
        kf.pack(**PAD, fill="x")
        self.lbl_kp = tk.Label(kf, text="Kp=0.0250", bg=PANEL_BG,
                               fg=TEXT_COLOR, font=("Consolas", 8), width=10, anchor="w")
        self.lbl_kp.pack(side=tk.LEFT)
        self.var_kp = tk.DoubleVar(value=0.025)
        tk.Scale(kf, from_=0.005, to=0.100, resolution=0.005,
                 orient=tk.HORIZONTAL, variable=self.var_kp,
                 command=self._on_kp_change,
                 bg=PANEL_BG, fg=TEXT_COLOR, highlightthickness=0,
                 troughcolor=PLOT_BG, activebackground=C_BLUE,
                 length=150, showvalue=False).pack(side=tk.LEFT)

        # Kd
        kdf = tk.Frame(f, bg=PANEL_BG)
        kdf.pack(**PAD, fill="x")
        self.lbl_kd = tk.Label(kdf, text="Kd=0.0020", bg=PANEL_BG,
                               fg=TEXT_COLOR, font=("Consolas", 8), width=10, anchor="w")
        self.lbl_kd.pack(side=tk.LEFT)
        self.var_kd = tk.DoubleVar(value=0.002)
        tk.Scale(kdf, from_=0.001, to=0.015, resolution=0.001,
                 orient=tk.HORIZONTAL, variable=self.var_kd,
                 command=self._on_kd_change,
                 bg=PANEL_BG, fg=TEXT_COLOR, highlightthickness=0,
                 troughcolor=PLOT_BG, activebackground=C_PURPLE,
                 length=150, showvalue=False).pack(side=tk.LEFT)

        _button(f, "⚡ Aplicar Kp/Kd al robot →", self._apply_gains,
                "#2d2d1a", C_YELLOW, pady=5).pack(**PAD, fill="x")

        ttk.Separator(f, orient="horizontal").pack(fill="x", padx=8, pady=5)

        # ── Límites de inclinación frontal / trasero ──────────────────────
        tk.Label(f, text="LÍMITES DE INCLINACiÓN", bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 8, "bold")).pack(**PAD, anchor="w")

        # Labels que se actualizarán con telemetría
        lf1 = tk.Frame(f, bg=PANEL_BG)
        lf1.pack(**PAD, fill="x")
        self.lbl_front_val = tk.Label(lf1, text="FRONT: +35.0° (default)",
                                      bg=PANEL_BG, fg=C_RED, font=("Consolas", 7))
        self.lbl_front_val.pack(side=tk.LEFT)

        lf2 = tk.Frame(f, bg=PANEL_BG)
        lf2.pack(**PAD, fill="x")
        self.lbl_rear_val = tk.Label(lf2, text="REAR:  -35.0° (default)",
                                     bg=PANEL_BG, fg=C_YELLOW, font=("Consolas", 7))
        self.lbl_rear_val.pack(side=tk.LEFT)

        # Botones de calibración asimétrica
        cf = tk.Frame(f, bg=PANEL_BG)
        cf.pack(**PAD, fill="x")
        _button(cf, "▲ CAL FRONT",
                lambda: self._send_cmd("hb cal front"),
                "#3a1a1a", C_RED, pady=4
                ).grid(row=0, column=0, padx=2, sticky="ew")
        _button(cf, "▼ CAL REAR",
                lambda: self._send_cmd("hb cal rear"),
                "#3a3a00", C_YELLOW, pady=4
                ).grid(row=0, column=1, padx=2, sticky="ew")
        cf.grid_columnconfigure(0, weight=1)
        cf.grid_columnconfigure(1, weight=1)

        # Sliders manuales para fijar límites
        flf = tk.Frame(f, bg=PANEL_BG)
        flf.pack(**PAD, fill="x")
        self.lbl_front_s = tk.Label(flf, text="F=35.0°", bg=PANEL_BG,
                                    fg=C_RED, font=("Consolas", 7), width=8, anchor="w")
        self.lbl_front_s.pack(side=tk.LEFT)
        self.var_front = tk.DoubleVar(value=35.0)
        tk.Scale(flf, from_=5, to=60, resolution=0.5,
                 orient=tk.HORIZONTAL, variable=self.var_front,
                 command=self._on_front_change,
                 bg=PANEL_BG, fg=TEXT_COLOR, highlightthickness=0,
                 troughcolor=PLOT_BG, activebackground=C_RED,
                 length=140, showvalue=False).pack(side=tk.LEFT)

        rlf = tk.Frame(f, bg=PANEL_BG)
        rlf.pack(**PAD, fill="x")
        self.lbl_rear_s = tk.Label(rlf, text="R=35.0°", bg=PANEL_BG,
                                   fg=C_YELLOW, font=("Consolas", 7), width=8, anchor="w")
        self.lbl_rear_s.pack(side=tk.LEFT)
        self.var_rear = tk.DoubleVar(value=35.0)
        tk.Scale(rlf, from_=5, to=60, resolution=0.5,
                 orient=tk.HORIZONTAL, variable=self.var_rear,
                 command=self._on_rear_change,
                 bg=PANEL_BG, fg=TEXT_COLOR, highlightthickness=0,
                 troughcolor=PLOT_BG, activebackground=C_YELLOW,
                 length=140, showvalue=False).pack(side=tk.LEFT)

        _button(f, "✔ Enviar límites al robot", self._apply_limits,
                "#1a2a3a", C_CYAN, pady=5).pack(**PAD, fill="x")

        ttk.Separator(f, orient="horizontal").pack(fill="x", padx=8, pady=5)

        # ── Velocidad manual ──────────────────────────────────────────────
        tk.Label(f, text="VELOCIDAD MANUAL", bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 8, "bold")).pack(**PAD, anchor="w")

        # Lineal
        vlf = tk.Frame(f, bg=PANEL_BG)
        vlf.pack(**PAD, fill="x")
        self.lbl_vlin = tk.Label(vlf, text="Lin= 0.00m/s", bg=PANEL_BG,
                                 fg=TEXT_COLOR, font=("Consolas", 8), width=13, anchor="w")
        self.lbl_vlin.pack(side=tk.LEFT)
        self.var_vlin = tk.DoubleVar(value=0.0)
        tk.Scale(vlf, from_=-0.5, to=0.5, resolution=0.05,
                 orient=tk.HORIZONTAL, variable=self.var_vlin,
                 command=self._on_vlin_change,
                 bg=PANEL_BG, fg=TEXT_COLOR, highlightthickness=0,
                 troughcolor=PLOT_BG, activebackground=C_CYAN,
                 length=140, showvalue=False).pack(side=tk.LEFT)

        # Angular
        waf = tk.Frame(f, bg=PANEL_BG)
        waf.pack(**PAD, fill="x")
        self.lbl_wang = tk.Label(waf, text="Ang= 0.00r/s", bg=PANEL_BG,
                                 fg=TEXT_COLOR, font=("Consolas", 8), width=13, anchor="w")
        self.lbl_wang.pack(side=tk.LEFT)
        self.var_wang = tk.DoubleVar(value=0.0)
        tk.Scale(waf, from_=-1.0, to=1.0, resolution=0.1,
                 orient=tk.HORIZONTAL, variable=self.var_wang,
                 command=self._on_wang_change,
                 bg=PANEL_BG, fg=TEXT_COLOR, highlightthickness=0,
                 troughcolor=PLOT_BG, activebackground=C_CYAN,
                 length=140, showvalue=False).pack(side=tk.LEFT)

        vbf = tk.Frame(f, bg=PANEL_BG)
        vbf.pack(**PAD, fill="x")
        _button(vbf, "▶ Enviar",      self._send_velocity, "#1a3a1a", C_GREEN,
                pady=5).pack(side=tk.LEFT, padx=(0, 4))
        _button(vbf, "■ STOP",        self._send_stop,     "#4a1a1a", C_RED,
                pady=5).pack(side=tk.LEFT)

        ttk.Separator(f, orient="horizontal").pack(fill="x", padx=8, pady=5)

        # ── Consola serial ────────────────────────────────────────────────
        tk.Label(f, text="CONSOLA SERIAL", bg=PANEL_BG, fg=C_CYAN,
                 font=("Consolas", 8, "bold")).pack(**PAD, anchor="w")

        self.console = scrolledtext.ScrolledText(
            f, bg=PLOT_BG, fg=TEXT_COLOR, font=("Consolas", 7),
            height=9, wrap=tk.WORD, state=tk.DISABLED,
            insertbackground=TEXT_COLOR, relief="flat",
            selectbackground="#45475a"
        )
        self.console.pack(padx=9, fill=tk.BOTH, expand=True, pady=(0, 2))

        # Tags de color en consola
        self.console.tag_config("ok",   foreground=C_GREEN)
        self.console.tag_config("warn", foreground=C_YELLOW)
        self.console.tag_config("err",  foreground=C_RED)
        self.console.tag_config("tx",   foreground=C_CYAN)
        self.console.tag_config("dim",  foreground=GRID_COLOR)

        # Entrada manual
        ef = tk.Frame(f, bg=PANEL_BG)
        ef.pack(padx=9, fill="x", pady=(0, 8))
        self.cmd_entry = tk.Entry(
            ef, bg=PLOT_BG, fg=TEXT_COLOR, font=("Consolas", 8),
            insertbackground=TEXT_COLOR, relief="flat"
        )
        self.cmd_entry.pack(side=tk.LEFT, fill="x", expand=True, padx=(0, 4))
        self.cmd_entry.bind("<Return>", lambda _: self._send_manual_cmd())
        _button(ef, "↵", self._send_manual_cmd, PANEL_BG, C_BLUE,
                pady=2).pack(side=tk.LEFT)

    # =========================================================================
    # Animación matplotlib  (hilo principal, llamada periódica)
    # =========================================================================

    def _animate(self, _frame):
        # Drenar la cola de datos seriales
        processed = 0
        while not self.data_queue.empty() and processed < 80:
            try:
                line = self.data_queue.get_nowait()
                self._handle_data(line)
                processed += 1
            except queue.Empty:
                break

        if not self.t_buf:
            return

        t_arr   = list(self.t_buf)
        t_rel   = [x - t_arr[-1] for x in t_arr]   # relativo al último punto
        t_min   = -WINDOW_SECONDS

        err_arr  = list(self.err_buf)
        gy_arr   = [g / 10.0 for g in self.gy_buf]  # escalar para coincidir con eje °
        cor_arr  = list(self.cor_buf)
        base_arr = list(self.base_buf)
        fin_arr  = list(self.fin_buf)
        lrpm_arr = list(self.lrpm_buf)
        rrpm_arr = list(self.rrpm_buf)

        # ── Plot 1: Pitch error ───────────────────────────────────────────
        self.line_err.set_data(t_rel, err_arr)
        self.line_gy.set_data(t_rel, gy_arr)
        self.ax_err.set_xlim(t_min, 0.5)        # Actualizar líneas dinámicas de límite frontal/trasero
        fl = self.front_limit
        rl = self.rear_limit
        self.hline_front.set_data([t_min, 0.5], [ fl,  fl])
        self.hline_rear.set_data( [t_min, 0.5], [-rl, -rl])
        self.lbl_front_line.set_text(f"FRONT +{fl:.1f}°")
        self.lbl_rear_line.set_text(f"REAR −{rl:.1f}°")
        # ── Plot 2: Corrección ────────────────────────────────────────────
        self.line_cor.set_data(t_rel, cor_arr)
        self.line_base.set_data(t_rel, base_arr)
        self.line_fin.set_data(t_rel, fin_arr)
        self.ax_cor.set_xlim(t_min, 0.5)
        all_vals = [abs(v) for v in cor_arr + fin_arr + base_arr if v]
        if all_vals:
            lim = max(max(all_vals) * 1.35, MAX_CORRECTION * 0.6)
            self.ax_cor.set_ylim(-lim, lim)

        # ── Plot 3: RPM ───────────────────────────────────────────────────
        self.line_lrpm.set_data(t_rel, lrpm_arr)
        self.line_rrpm.set_data(t_rel, rrpm_arr)
        self.ax_rpm.set_xlim(t_min, 0.5)
        all_rpm = [abs(v) for v in lrpm_arr + rrpm_arr if v]
        if all_rpm:
            mx = max(max(all_rpm) * 1.3, 5)
            self.ax_rpm.set_ylim(-mx, mx)

        # ── Plot 4: Tilt indicator ────────────────────────────────────────
        last_err = err_arr[-1] if err_arr else 0.0
        self._update_tilt(last_err)

        # ── Labels numéricos en panel ─────────────────────────────────────
        last_cor  = cor_arr[-1]  if cor_arr  else 0.0
        last_lrpm = lrpm_arr[-1] if lrpm_arr else 0
        last_rrpm = rrpm_arr[-1] if rrpm_arr else 0
        self._refresh_labels(last_err, last_cor, last_lrpm, last_rrpm)

    def _refresh_labels(self, err, cor, lrpm, rrpm):
        abs_e = abs(err)
        fl = self.front_limit
        rl = self.rear_limit
        if (err > 0 and abs_e > fl * 0.8) or (err < 0 and abs_e > rl * 0.8):
            pitch_color = C_RED
        elif abs_e > DEAD_ZONE:
            pitch_color = C_YELLOW
        else:
            pitch_color = C_GREEN
        self.lbl_pitch.config(text=f"Pitch: {err:+6.2f}°", fg=pitch_color)
        self.lbl_corr.config(text=f"Corr: {cor:+.3f} m/s")
        self.lbl_rpm.config(text=f"  L:{int(lrpm):3d} R:{int(rrpm):3d} rpm")

    # =========================================================================
    # Comunicación serial  (hilo de fondo)
    # =========================================================================

    def _serial_loop(self):
        while self.running:
            if not self.connected:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
                    self.connected = True
                    self._log(f"✓ Conectado a {self.port} @ {self.baud}", "ok")
                    self.root.after(0, lambda: self.lbl_conn.config(
                        text=f"● {self.port} {self.baud}", fg=C_GREEN))
                except serial.SerialException as e:
                    self._log(f"✗ {e}", "err")
                    time.sleep(2.0)
                    continue

            try:
                # Enviar comandos pendientes
                while not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get_nowait()
                    self.ser.write((cmd + "\n").encode("utf-8"))
                    self._log(f"→ {cmd}", "tx")

                # Leer línea si hay datos
                if self.ser.in_waiting:
                    raw = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if raw:
                        self.data_queue.put(raw)
                else:
                    time.sleep(0.002)

            except (serial.SerialException, OSError) as e:
                self._log(f"✗ Serial perdido: {e}", "err")
                self.connected = False
                self.root.after(0, lambda: self.lbl_conn.config(
                    text="● Desconectado", fg=C_RED))
                try:
                    self.ser.close()
                except Exception:
                    pass

    def _handle_data(self, raw: str):
        """Parsea una línea y actualiza buffers o consola."""
        now = time.time() - self.t0

        # ── Línea B (balance telemetría 10 Hz) ────────────────────────────
        # B err=X.XX gy=X.X cor=X.XXX base=X.XXX fin=X.XXX Lrpm=X Rrpm=X
        if raw.startswith("B "):
            try:
                parts = {}
                for tok in raw[2:].split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        parts[k] = float(v)
                self.t_buf.append(now)
                self.err_buf.append(parts.get("err",  0.0))
                self.gy_buf.append(parts.get("gy",    0.0))
                self.cor_buf.append(parts.get("cor",  0.0))
                self.base_buf.append(parts.get("base", 0.0))
                self.fin_buf.append(parts.get("fin",  0.0))
                self.lrpm_buf.append(parts.get("Lrpm", 0.0))
                self.rrpm_buf.append(parts.get("Rrpm", 0.0))
                # Actualizar límites si el robot los envía
                if "fl" in parts:
                    self.front_limit = parts["fl"]
                    self.root.after(0, lambda fl=parts["fl"]: self.lbl_front_val.config(
                        text=f"FRONT: +{fl:.1f}°"))
                if "rl" in parts:
                    self.rear_limit = parts["rl"]
                    self.root.after(0, lambda rl=parts["rl"]: self.lbl_rear_val.config(
                        text=f"REAR:  -{rl:.1f}°"))
            except (ValueError, KeyError):
                pass
            return

        # ── Línea T (motor telemetría, usar solo RPM si no hay línea B) ───
        if raw.startswith("T "):
            # No inundar consola; solo usamos los RPM si faltan datos B
            if not self.t_buf or (now - list(self.t_buf)[-1]) > 0.3:
                try:
                    parts = {}
                    for tok in raw[2:].split():
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            try:
                                parts[k] = float(v)
                            except ValueError:
                                pass
                    self.t_buf.append(now)
                    self.err_buf.append(0.0)
                    self.gy_buf.append(0.0)
                    self.cor_buf.append(0.0)
                    self.base_buf.append(parts.get("lin", 0.0))
                    self.fin_buf.append(parts.get("lin", 0.0))
                    self.lrpm_buf.append(parts.get("Lrpm", 0.0))
                    self.rrpm_buf.append(parts.get("Rrpm", 0.0))
                except (ValueError, KeyError):
                    pass
            return

        # ── Mensajes de estado → consola ──────────────────────────────────
        if any(raw.startswith(p) for p in ("[BAL]", "[MPU]", "[HB]", "===")):
            tag = "ok"
            if "ERROR" in raw or "CAIDA" in raw or "✗" in raw or "PELIGRO" in raw:
                tag = "err"
            elif "DESACTIVADO" in raw or "WARNING" in raw:
                tag = "warn"
            self._log(raw, tag)

            # Actualizar indicador de balance
            if "ACTIVADO" in raw:
                self.balance_active = True
                self.root.after(0, lambda: self.lbl_bal.config(
                    text="Balance: ACTIVO ✓", fg=C_GREEN))
            elif "DESACTIVADO" in raw or "CAIDA" in raw:
                self.balance_active = False
                self.root.after(0, lambda: self.lbl_bal.config(
                    text="Balance: INACTIVO ✗", fg=C_RED))            # Actualizar labels de límites cuando llegan mensajes de calibración
            if "Limite FRONTAL calibrado" in raw or "Limite FRONTAL:" in raw:
                try:
                    v = float(raw.split("+")[1].split()[0])
                    self.front_limit = v
                    tag_cal = " (CAL)" if "calibrado" in raw else ""
                    self.root.after(0, lambda v=v, t=tag_cal:
                        (self.lbl_front_val.config(text=f"FRONT: +{v:.1f}\u00b0{t}"),
                         self.var_front.set(v),
                         self.lbl_front_s.config(text=f"F={v:.1f}\u00b0")))
                except (IndexError, ValueError):
                    pass
            if "Limite TRASERO calibrado" in raw or "Limite TRASERO:" in raw:
                try:
                    v = float(raw.split("-")[1].split()[0])
                    self.rear_limit = v
                    tag_cal = " (CAL)" if "calibrado" in raw else ""
                    self.root.after(0, lambda v=v, t=tag_cal:
                        (self.lbl_rear_val.config(text=f"REAR:  -{v:.1f}°{t}"),
                         self.var_rear.set(v),
                         self.lbl_rear_s.config(text=f"R={v:.1f}°")))
                except (IndexError, ValueError):
                    pass
            return

        # Cualquier otra línea no T/B: mostrar en consola
        self._log(raw, "dim")

    # =========================================================================
    # Utilidades GUI
    # =========================================================================

    def _log(self, msg: str, tag: str = "dim"):
        """Añade mensaje a la consola de forma thread-safe."""
        def _do():
            self.console.config(state=tk.NORMAL)
            self.console.insert(tk.END, msg + "\n", tag)
            self.console.see(tk.END)
            # Mantener máximo 400 líneas
            n = int(self.console.index("end-1c").split(".")[0])
            if n > 400:
                self.console.delete("1.0", f"{n - 300}.0")
            self.console.config(state=tk.DISABLED)
        self.root.after(0, _do)

    def _send_cmd(self, cmd: str):
        self.cmd_queue.put(cmd)

    # ── Callbacks de controles ────────────────────────────────────────────

    def _on_kp_change(self, val):
        v = float(val)
        self.current_kp = v
        self.lbl_kp.config(text=f"Kp={v:.4f}")

    def _on_kd_change(self, val):
        v = float(val)
        self.current_kd = v
        self.lbl_kd.config(text=f"Kd={v:.4f}")

    def _apply_gains(self):
        self._send_cmd(f"hb kp {self.current_kp:.4f}")
        self._send_cmd(f"hb kd {self.current_kd:.4f}")

    def _on_front_change(self, val):
        v = float(val)
        self.lbl_front_s.config(text=f"F={v:.1f}°")

    def _on_rear_change(self, val):
        v = float(val)
        self.lbl_rear_s.config(text=f"R={v:.1f}°")

    def _apply_limits(self):
        fl = self.var_front.get()
        rl = self.var_rear.get()
        self._send_cmd(f"hb front {fl:.1f}")
        self._send_cmd(f"hb rear {rl:.1f}")

    def _on_vlin_change(self, val):
        self.lbl_vlin.config(text=f"Lin={float(val):+.2f}m/s")

    def _on_wang_change(self, val):
        self.lbl_wang.config(text=f"Ang={float(val):+.2f}r/s")

    def _send_velocity(self):
        lin = self.var_vlin.get()
        ang = self.var_wang.get()
        self._send_cmd(f"v {lin:.3f} {ang:.3f}")

    def _send_stop(self):
        self.var_vlin.set(0.0)
        self.var_wang.set(0.0)
        self.lbl_vlin.config(text="Lin= 0.00m/s")
        self.lbl_wang.config(text="Ang= 0.00r/s")
        self._send_cmd("v 0.000 0.000")

    def _send_manual_cmd(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self._send_cmd(cmd)
            self.cmd_entry.delete(0, tk.END)

    def _on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        plt.close("all")
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Dashboard gráfico — Balance Anti-Caída Smart Golf Trolley"
    )
    parser.add_argument("port", nargs="?", default="COM4",
                        help="Puerto serial (default: COM4)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baudrate (default: 115200)")
    args = parser.parse_args()

    print(f"Balance Dashboard  —  {args.port} @ {args.baud} baud")
    print("Cierra la ventana para salir.\n")

    app = BalanceDashboard(args.port, args.baud)
    app.run()


if __name__ == "__main__":
    main()
