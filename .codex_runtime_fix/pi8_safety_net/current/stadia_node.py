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
        self.declare_parameter('rotation_speed_scale', 0.5)
        self.declare_parameter('stadia_mac',      'C1:B8:D3:D6:E9:D5')
        self.declare_parameter('takeover_on_connect', True)
        self.declare_parameter('takeover_axis_threshold', 0.25)
        self.declare_parameter('require_neutral_to_arm', True)

        # ── Publicadores ────────────────────────────────────────────────────────
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.pub_raw    = self.create_publisher(String, '/arduino/raw_command', 10)
        self.pub_enable = self.create_publisher(Bool,   '/follower/enable', 5)
        self.pub_authorized_enable = self.create_publisher(
            Bool, '/follower/authorized_enable', 5
        )
        self.pub_status = self.create_publisher(String, '/stadia/state', 5)
        self.create_subscription(String, '/stadia/control', self._control_cb, 5)
        self.create_subscription(
            Bool,
            '/operator/follower_request',
            self._operator_follower_request_cb,
            5,
        )
        self.create_subscription(Bool, '/follower/enable', self._follower_enable_cb, 5)

        # ── Estado ──────────────────────────────────────────────────────────────
        self.smooth_lin = 0.0
        self.smooth_ang = 0.0
        self.axis = {}   # evdev raw axis values
        self.balance_on = False
        # Fail closed at startup. A StadiaNode process is not evidence that
        # the physical controller is connected.
        self.control_mode = 'off'
        self._dev = None
        self._running = True
        self._neutral_armed = False

        # ── Timer de publicación ─────────────────────────────────────────────────
        hz = self.get_parameter('send_rate_hz').value
        self.create_timer(1.0 / hz, self._publish_cb)
        self.create_timer(0.5, self._connection_safety_tick)

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
                    if self.get_parameter('takeover_on_connect').value:
                        self._set_stadia_mode()
                        self.get_logger().warn(
                            'Stadia conectado: takeover manual automatico'
                        )
                    else:
                        self._publish_status('connected')
                except Exception:
                    pass

            try:
                for ev in self._dev.read_loop():
                    if ev.type == EV_ABS:
                        self.axis[ev.code] = ev.value
                        if (
                            self.control_mode == 'follower'
                            and ev.code in (ABS_LX, ABS_LY)
                            and self._manual_axis_takeover_requested(
                                ABS_LX, ABS_LY
                            )
                        ):
                            self._takeover_from_stick()
                    elif ev.type == EV_KEY and ev.value == 1:
                        self._handle_button(ev.code,
                                            BTN_A, BTN_B, BTN_X, BTN_Y, BTN_MENU)
            except Exception as e:
                self.get_logger().warn(f'Stadia desconectado: {e}')
                try:
                    self._dev.ungrab(); self._dev.close()
                except Exception:
                    pass
                # A lost controller must actively overwrite the last non-zero command.
                self._send_stop()
                self.pub_authorized_enable.publish(Bool(data=False))
                self.pub_enable.publish(Bool(data=False))
                self.axis.clear()
                self._neutral_armed = False
                self.control_mode = 'off'
                self._dev = None
                self._publish_status('disconnected')

    def _manual_axis_takeover_requested(self, abs_lx, abs_ly):
        if abs_lx not in self.axis or abs_ly not in self.axis:
            return False
        threshold = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('takeover_axis_threshold').value),
            ),
        )
        lx_n = self._normalize(self.axis[abs_lx])
        ly_n = self._normalize(self.axis[abs_ly])
        return abs(lx_n) >= threshold or abs(ly_n) >= threshold

    def _takeover_from_stick(self):
        # A deliberate stick movement is an emergency/manual override. Revoke
        # autonomous motion first; the regular timer publishes manual velocity
        # only after this zero command.
        self.control_mode = 'stadia'
        self.pub_authorized_enable.publish(Bool(data=False))
        self.pub_enable.publish(Bool(data=False))
        self._send_stop()
        self._neutral_armed = True
        self._publish_status('connected')
        self.get_logger().warn(
            'Manual Stadia takeover: FOLLOWER cancelled by stick movement'
        )

    # ── Gestión de botones ────────────────────────────────────────────────────────
    def _handle_button(self, code, BTN_A, BTN_B, BTN_X, BTN_Y, BTN_MENU):
        if code == BTN_A:                          # Volver a control Stadia
            self._set_stadia_mode()
        elif code == BTN_B:
            self._send_stop()
            self.get_logger().warn(
                'Boton B ignored: FOLLOWER must be selected from the interface'
            )
        elif code == BTN_Y:                        # Toggle balance
            cmd = 'hb off' if self.balance_on else 'hb on'
            self.pub_raw.publish(String(data=cmd))
            self.balance_on = not self.balance_on
        elif code == BTN_X:
            self.pub_raw.publish(String(data='z'))
        elif code == BTN_MENU:
            self.pub_enable.publish(Bool(data=False))
            self.pub_raw.publish(String(data='s'))
            self._send_stop()

    def _follower_enable_cb(self, msg: Bool):
        # A gesture or another direct /follower/enable publisher must never
        # displace an explicitly selected manual/off mode. Intentional follower
        # selection goes through _set_follower_mode(), which changes the mode
        # before publishing True.
        if bool(msg.data) and self.control_mode in ('stadia', 'off'):
            self._send_stop()
            self.pub_enable.publish(Bool(data=False))
            self.get_logger().warn(
                'Follower activation rejected; Stadia/off mode has priority'
            )

    def _control_cb(self, msg: String):
        command = msg.data.strip().upper()
        if command in ('ON', 'STADIA_ON', 'STADIA'):
            self._set_stadia_mode()
        elif command in ('FOLLOWER', 'FOLLOW', 'FOLLOWER_ON'):
            self._send_stop()
            self.get_logger().warn(
                'FOLLOWER control command rejected: use operator interface request'
            )
        elif command in ('OFF', 'STADIA_OFF'):
            self._set_off_mode()

    def _operator_follower_request_cb(self, msg: Bool):
        if bool(msg.data):
            self._set_follower_mode()
            return
        # Leaving FOLLOWER from the interface must revoke motion immediately,
        # regardless of whether the safety controller remains connected.
        self.pub_authorized_enable.publish(Bool(data=False))
        self.pub_enable.publish(Bool(data=False))
        self._send_stop()
        if self._dev is None:
            self.control_mode = 'off'

    def _send_stop(self, log=True):
        self.smooth_lin = 0.0; self.smooth_ang = 0.0
        t = Twist(); self.pub_cmd.publish(t)
        if log:
            self.get_logger().info('Stadia STOP')

    def _set_stadia_mode(self):
        self.control_mode = 'stadia'
        self.pub_authorized_enable.publish(Bool(data=False))
        self.pub_enable.publish(Bool(data=False))
        self._neutral_armed = not bool(
            self.get_parameter('require_neutral_to_arm').value
        )
        # Refresh the physical stick position so an old cached axis value
        # cannot restart motion when returning from FOLLOWER or IDLE.
        if self._dev is not None:
            try:
                from evdev import ecodes
                for code in (ecodes.ABS_X, ecodes.ABS_Y):
                    info = self._dev.absinfo(code)
                    # The Stadia BLE driver reports value=0 before the first
                    # real axis event even though the advertised minimum is 1.
                    if info.min <= info.value <= info.max:
                        self.axis[code] = info.value
                    else:
                        self.axis.pop(code, None)
            except Exception as exc:
                self.get_logger().warn(f'No se pudieron refrescar ejes Stadia: {exc}')
                self.axis.clear()
        self._send_stop()
        self._publish_status('connected')
        self.get_logger().info('[A] Control Stadia activo')

    def _set_follower_mode(self):
        if self._dev is None:
            self.control_mode = 'off'
            self._neutral_armed = False
            self.pub_authorized_enable.publish(Bool(data=False))
            self.pub_enable.publish(Bool(data=False))
            self._send_stop()
            self._publish_status('disconnected')
            self.get_logger().warn(
                'FOLLOWER rejected: physical Stadia controller is not connected'
            )
            return
        self.control_mode = 'follower'
        self._neutral_armed = False
        self._send_stop()
        self._publish_status('connected')
        self.pub_authorized_enable.publish(Bool(data=True))
        self.get_logger().info(
            'Follower activo por interfaz; joystick Stadia listo para takeover'
        )

    def _set_off_mode(self):
        self.control_mode = 'off'
        self._neutral_armed = False
        self.pub_authorized_enable.publish(Bool(data=False))
        self.pub_enable.publish(Bool(data=False))
        self._send_stop()
        self._publish_status('connected')
        self.get_logger().info('Stadia desactivado por comando')

    # ── Timer de publicación (send_rate_hz) ───────────────────────────────────────
    def _publish_cb(self):
        if self._dev is None:
            return
        if self.control_mode != 'stadia':
            return

        p = {
            'max_lin':  self.get_parameter('max_linear_vel').value,
            'max_ang':  self.get_parameter('max_angular_vel').value,
            'dz_lin':   self.get_parameter('deadzone_lin').value,
            'dz_ang':   self.get_parameter('deadzone_ang').value,
            'expo':     self.get_parameter('angular_expo').value,
            'alpha':    self.get_parameter('smoothing_alpha').value,
            'rot_dom':  self.get_parameter('rotation_dominance').value,
            'rot_scale': self.get_parameter('rotation_speed_scale').value,
        }

        from evdev import ecodes
        if (
            not self._neutral_armed
            and (
                ecodes.ABS_X not in self.axis
                or ecodes.ABS_Y not in self.axis
            )
        ):
            self._send_stop(log=False)
            return

        lx_n = self._normalize(self.axis.get(ecodes.ABS_X, 128))
        ly_n = self._normalize(self.axis.get(ecodes.ABS_Y, 128))

        target_lin = self._deadzone(-ly_n, p['dz_lin']) * p['max_lin']
        ax_raw     = self._deadzone(lx_n,  p['dz_ang'])
        target_ang = self._expo(ax_raw, p['expo']) * p['max_ang']

        # A controller that connects with a displaced stick must never move
        # the robot. It is armed only after both sticks are observed neutral.
        if not self._neutral_armed:
            if target_lin == 0.0 and target_ang == 0.0:
                self._neutral_armed = True
                self.get_logger().info('Stadia armado: palancas centradas')
            self._send_stop(log=False)
            return

        # Rotación pura si angular domina
        if target_ang != 0 and (
            target_lin == 0 or abs(target_ang) > p['rot_dom'] * abs(target_lin)
        ):
            target_lin = 0.0
            target_ang *= max(0.0, min(1.0, float(p['rot_scale'])))

        # Suavizado (smoothing_alpha: 0=lento 1=directo)
        a = p['alpha']
        self.smooth_lin += a * (target_lin - self.smooth_lin)
        self.smooth_ang += a * (target_ang - self.smooth_ang)

        t = Twist()
        t.linear.x  = float(self.smooth_lin)
        t.angular.z = float(self.smooth_ang)
        self.pub_cmd.publish(t)

    def _connection_safety_tick(self):
        if self._dev is None:
            self.control_mode = 'off'
            self._neutral_armed = False
            self.pub_authorized_enable.publish(Bool(data=False))
            self.pub_enable.publish(Bool(data=False))
            self._send_stop(log=False)
            self._publish_status('disconnected')
            return
        self._publish_status('connected')

    def _publish_status(self, state: str):
        self.pub_status.publish(String(data=json.dumps({
            'stadia': state,
            'mode': self.control_mode,
            'neutral_armed': self._neutral_armed,
            'takeover_on_connect': self.get_parameter('takeover_on_connect').value,
            'takeover_axis_threshold': self.get_parameter(
                'takeover_axis_threshold'
            ).value,
            'rotation_speed_scale': self.get_parameter('rotation_speed_scale').value,
            'buttons': {'A': 'stadia_takeover', 'B': 'ignored', 'menu': 'stop'},
        })))

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
