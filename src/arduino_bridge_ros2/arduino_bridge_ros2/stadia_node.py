"""
stadia_node.py — Nodo ROS2 para mando Stadia (evdev → /cmd_vel)

Parámetros configurables en follower_params.yaml bajo stadia_node:
  max_linear_vel, max_angular_vel, deadzone_lin, deadzone_ang,
  angular_expo, smoothing_alpha, send_rate_hz
"""
import json
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class StadiaNode(Node):
    def __init__(self):
        super().__init__('stadia_node')
        # ── Parámetros ──────────────────────────────────────────────────────────
        self.declare_parameter('max_linear_vel',  0.40)   # m/s
        self.declare_parameter('max_angular_vel', 0.70)   # rad/s
        self.declare_parameter('deadzone_lin',    0.12)
        self.declare_parameter('deadzone_ang',    0.15)
        self.declare_parameter('angular_expo',    2.0)    # 1.0=lineal 2.0=cuadrático
        self.declare_parameter('smoothing_alpha', 0.35)   # 0=sin cambio 1=instantáneo
        self.declare_parameter('send_rate_hz',    20.0)
        self.declare_parameter('rotation_dominance', 2.0)
        self.declare_parameter('stadia_mac',      'C1:B8:D3:D6:E9:D5')

        # ── Publicadores ────────────────────────────────────────────────────────
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.pub_raw    = self.create_publisher(String, '/arduino/raw_command', 10)
        self.pub_enable = self.create_publisher(Bool,   '/follower/enable', 5)
        self.pub_status = self.create_publisher(String, '/stadia/state', 5)

        # ── Estado ──────────────────────────────────────────────────────────────
        self.smooth_lin = 0.0
        self.smooth_ang = 0.0
        self.axis = {}   # evdev raw axis values
        self.balance_on = False
        self._dev = None
        self._running = True

        # ── Timer de publicación ─────────────────────────────────────────────────
        hz = self.get_parameter('send_rate_hz').value
        self.create_timer(1.0 / hz, self._publish_cb)

        # ── Hilo de lectura evdev ────────────────────────────────────────────────
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info('StadiaNode iniciado')

    # ── Helpers matemáticos ──────────────────────────────────────────────────────
    @staticmethod
    def _normalize(raw, mid=128.0, rng=127.0):
        return (raw - mid) / rng

    @staticmethod
    def _deadzone(v, dz):
        if abs(v) < dz:
            return 0.0
        s = 1.0 if v > 0 else -1.0
        return s * (abs(v) - dz) / (1.0 - dz)

    @staticmethod
    def _expo(v, e):
        return (abs(v) ** e) * (1.0 if v >= 0 else -1.0)

    # ── Buscar Stadia ─────────────────────────────────────────────────────────────
    def _find_stadia(self):
        try:
            import evdev
            for p in evdev.list_devices():
                d = evdev.InputDevice(p)
                if 'stadia' in d.name.lower() or \
                   (hasattr(d, 'info') and d.info.vendor == 0x18d1):
                    return d
                d.close()
        except Exception as e:
            self.get_logger().warn(f'evdev error: {e}')
        return None

    # ── Bucle de lectura ─────────────────────────────────────────────────────────
    def _read_loop(self):
        import time
        try:
            from evdev import ecodes
            BTN_A = ecodes.BTN_A; BTN_B = ecodes.BTN_B
            BTN_X = ecodes.BTN_X; BTN_Y = ecodes.BTN_Y
            BTN_MENU = ecodes.BTN_START
            EV_ABS = ecodes.EV_ABS; EV_KEY = ecodes.EV_KEY
            ABS_LX = ecodes.ABS_X; ABS_LY = ecodes.ABS_Y
        except ImportError:
            self.get_logger().error('evdev no disponible')
            return

        while self._running:
            if self._dev is None:
                self._dev = self._find_stadia()
                if self._dev is None:
                    time.sleep(3.0)
                    continue
                try:
                    self._dev.grab()
                    self.get_logger().info(f'Stadia conectado: {self._dev.name}')
                    self._publish_status('connected')
                except Exception:
                    pass

            try:
                for ev in self._dev.read_loop():
                    if ev.type == EV_ABS:
                        self.axis[ev.code] = ev.value
                    elif ev.type == EV_KEY and ev.value == 1:
                        self._handle_button(ev.code,
                                            BTN_A, BTN_B, BTN_X, BTN_Y, BTN_MENU)
            except Exception as e:
                self.get_logger().warn(f'Stadia desconectado: {e}')
                try:
                    self._dev.ungrab(); self._dev.close()
                except Exception:
                    pass
                self._dev = None
                self.smooth_lin = 0.0
                self.smooth_ang = 0.0
                self._publish_status('disconnected')

    # ── Gestión de botones ────────────────────────────────────────────────────────
    def _handle_button(self, code, BTN_A, BTN_B, BTN_X, BTN_Y, BTN_MENU):
        if code == BTN_A:                          # STOP de emergencia
            self._send_stop()
        elif code == BTN_Y:                        # Toggle balance
            cmd = 'hb off' if self.balance_on else 'hb on'
            self.pub_raw.publish(String(data=cmd))
            self.balance_on = not self.balance_on
        elif code == BTN_X:
            self.pub_raw.publish(String(data='z'))
        elif code == BTN_MENU:
            self.pub_raw.publish(String(data='s'))

    def _send_stop(self):
        self.smooth_lin = 0.0; self.smooth_ang = 0.0
        t = Twist(); self.pub_cmd.publish(t)
        self.get_logger().info('[A] PARADA EMERGENCIA')

    # ── Timer de publicación (send_rate_hz) ───────────────────────────────────────
    def _publish_cb(self):
        if self._dev is None:
            return

        p = {
            'max_lin':  self.get_parameter('max_linear_vel').value,
            'max_ang':  self.get_parameter('max_angular_vel').value,
            'dz_lin':   self.get_parameter('deadzone_lin').value,
            'dz_ang':   self.get_parameter('deadzone_ang').value,
            'expo':     self.get_parameter('angular_expo').value,
            'alpha':    self.get_parameter('smoothing_alpha').value,
            'rot_dom':  self.get_parameter('rotation_dominance').value,
        }

        from evdev import ecodes
        lx_n = self._normalize(self.axis.get(ecodes.ABS_X, 128))
        ly_n = self._normalize(self.axis.get(ecodes.ABS_Y, 128))

        target_lin = self._deadzone(-ly_n, p['dz_lin']) * p['max_lin']
        ax_raw     = self._deadzone(lx_n,  p['dz_ang'])
        target_ang = self._expo(ax_raw, p['expo']) * p['max_ang']

        # Rotación pura si angular domina
        if target_ang != 0 and (
            target_lin == 0 or abs(target_ang) > p['rot_dom'] * abs(target_lin)
        ):
            target_lin = 0.0

        # Suavizado (smoothing_alpha: 0=lento 1=directo)
        a = p['alpha']
        self.smooth_lin += a * (target_lin - self.smooth_lin)
        self.smooth_ang += a * (target_ang - self.smooth_ang)

        t = Twist()
        t.linear.x  = float(self.smooth_lin)
        t.angular.z = float(self.smooth_ang)
        self.pub_cmd.publish(t)

    def _publish_status(self, state: str):
        self.pub_status.publish(String(data=json.dumps({'stadia': state})))

    def destroy_node(self):
        self._running = False
        if self._dev:
            try:
                self._dev.ungrab(); self._dev.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StadiaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
