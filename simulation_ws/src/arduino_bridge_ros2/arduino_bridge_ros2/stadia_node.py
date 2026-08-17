"""
stadia_node.py — Nodo ROS2 para mando Stadia (evdev → /cmd_vel)

Parámetros configurables en follower_params.yaml bajo stadia_node:
  max_linear_vel, max_angular_vel, deadzone_lin, deadzone_ang,
  angular_expo, smoothing_alpha, send_rate_hz
"""
import json
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32


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
        self.declare_parameter('kinect_tilt_deadzone', 0.20)
        self.declare_parameter('kinect_tilt_speed_deg_s', 10.0)
        self.declare_parameter('kinect_tilt_command_interval', 0.15)

        # ── Publicadores ────────────────────────────────────────────────────────
        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.pub_raw    = self.create_publisher(String, '/arduino/raw_command', 10)
        self.pub_enable = self.create_publisher(Bool,   '/follower/enable', 5)
        self.pub_authorized_enable = self.create_publisher(
            Bool, '/follower/authorized_enable', 5
        )
        self.pub_status = self.create_publisher(String, '/stadia/state', 5)
        self.pub_kinect_tilt = self.create_publisher(Int32, '/kinect/tilt/set', 5)
        self.create_subscription(String, '/stadia/control', self._control_cb, 5)
        self.create_subscription(
            String, '/stadia/speed_limits', self._speed_limits_cb, 5
        )
        self.create_subscription(
            Bool,
            '/operator/follower_request',
            self._operator_follower_request_cb,
            5,
        )
        self.create_subscription(Bool, '/follower/enable', self._follower_enable_cb, 5)
        self.create_subscription(
            Int32, '/kinect/tilt/state', self._kinect_tilt_state_cb, 5
        )

        # ── Estado ──────────────────────────────────────────────────────────────
        self.smooth_lin = 0.0
        self.smooth_ang = 0.0
        self.axis = {}   # evdev raw axis values
        self.balance_on = False
        self._last_balance_off = 0.0
        # Repeat during startup so the command is received even if the Arduino
        # subscriber becomes ready after this node.
        self._startup_balance_off_remaining = 6
        # Fail closed at startup. A StadiaNode process is not evidence that
        # the physical controller is connected.
        self.control_mode = 'off'
        self._dev = None
        self._running = True
        self._neutral_armed = False
        self.kinect_tilt_mode = False
        self.kinect_tilt_degrees = 8
        self._tilt_neutral_armed = False
        self._last_tilt_command = 0.0
        self._last_drive_debug = 0.0
        self.max_linear_override = float(
            self.get_parameter('max_linear_vel').value
        )
        self.max_angular_override = float(
            self.get_parameter('max_angular_vel').value
        )

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
                            self.control_mode != 'stadia'
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
                self.kinect_tilt_mode = False
                self._tilt_neutral_armed = False
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
            self._set_stadia_mode()
            self.get_logger().warn(
                'Boton B: FOLLOWER cancelado; control Stadia activo en STOP'
            )
        elif code == BTN_Y:
            # Never let a controller button enable the firmware balance loop:
            # it can drive the wheels independently of /cmd_vel.
            self._set_stadia_mode()
            self._disable_balance(force=True)
            self._send_stop(disable_balance=False)
            self.get_logger().warn(
                'Boton Y: balance bloqueado por seguridad; STOP aplicado'
            )
        elif code == BTN_X:
            self._toggle_kinect_tilt_mode()
        elif code == BTN_MENU:
            self._set_off_mode()
            self.pub_raw.publish(String(data='s'))

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

    def _speed_limits_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            linear = max(0.10, min(0.40, float(payload['linear'])))
            angular = max(0.25, min(0.70, float(payload['angular'])))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Limites Stadia invalidos: {exc}')
            return
        self.max_linear_override = linear
        self.max_angular_override = angular
        self.get_logger().info(
            f'Limites Stadia actualizados: linear={linear:.2f} angular={angular:.2f}'
        )
        self._publish_status('connected' if self._dev is not None else 'disconnected')

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

    def _disable_balance(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_balance_off < 1.0:
            return
        self.pub_raw.publish(String(data='hb off'))
        self._last_balance_off = now
        self.balance_on = False

    def _kinect_tilt_state_cb(self, msg: Int32):
        self.kinect_tilt_degrees = max(-30, min(30, int(msg.data)))

    def _toggle_kinect_tilt_mode(self):
        self.kinect_tilt_mode = not self.kinect_tilt_mode
        self._tilt_neutral_armed = False
        if self.kinect_tilt_mode and self._dev is not None:
            try:
                from evdev import ecodes
                info = self._dev.absinfo(ecodes.ABS_RZ)
                if info.min <= info.value <= info.max:
                    self.axis[ecodes.ABS_RZ] = info.value
            except Exception as exc:
                self.get_logger().warn(
                    f'No se pudo refrescar palanca derecha: {exc}'
                )
        state = 'ACTIVO' if self.kinect_tilt_mode else 'INACTIVO'
        self.get_logger().info(
            f'[X] Control tilt Kinect {state}; angulo={self.kinect_tilt_degrees} deg'
        )
        self._publish_status('connected' if self._dev is not None else 'disconnected')

    def _update_kinect_tilt(self):
        if not self.kinect_tilt_mode or self._dev is None:
            return
        from evdev import ecodes
        raw = self.axis.get(ecodes.ABS_RZ, 128)
        value = self._normalize(raw)
        deadzone = max(
            0.0,
            min(0.9, float(self.get_parameter('kinect_tilt_deadzone').value)),
        )
        value = self._deadzone(value, deadzone)
        if not self._tilt_neutral_armed:
            if value == 0.0:
                self._tilt_neutral_armed = True
                self.get_logger().info('Tilt Kinect armado: palanca derecha centrada')
            return
        if value == 0.0:
            return
        now = time.monotonic()
        interval = max(
            0.10,
            float(self.get_parameter('kinect_tilt_command_interval').value),
        )
        if now - self._last_tilt_command < interval:
            return
        speed = max(
            1.0,
            float(self.get_parameter('kinect_tilt_speed_deg_s').value),
        )
        step = max(1, int(round(abs(value) * speed * interval)))
        # Up on the right stick reports a value below centre. Positive Kinect
        # angles point the camera upward, hence the inverted sign.
        requested = self.kinect_tilt_degrees + (-step if value > 0 else step)
        requested = max(-30, min(30, requested))
        if requested != self.kinect_tilt_degrees:
            self.pub_kinect_tilt.publish(Int32(data=requested))
            self._last_tilt_command = now

    def _send_stop(self, log=True, disable_balance=True):
        self.smooth_lin = 0.0; self.smooth_ang = 0.0
        t = Twist(); self.pub_cmd.publish(t)
        if disable_balance:
            self._disable_balance()
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
        self._update_kinect_tilt()
        if self._dev is None:
            return
        if self.control_mode != 'stadia':
            return

        p = {
            'max_lin':  self.max_linear_override,
            'max_ang':  self.max_angular_override,
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

        now_s = self.get_clock().now().nanoseconds * 1e-9
        if (
            (abs(target_lin) > 0.001 or abs(target_ang) > 0.001)
            and now_s - self._last_drive_debug >= 1.0
        ):
            self._last_drive_debug = now_s
            self.get_logger().info(
                'Stadia drive: raw_x=%s raw_y=%s norm_x=%.3f norm_y=%.3f '
                'cmd_lin=%.3f cmd_ang=%.3f'
                % (
                    self.axis.get(ecodes.ABS_X),
                    self.axis.get(ecodes.ABS_Y),
                    lx_n, ly_n, target_lin, target_ang,
                )
            )

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
        # Avoid an asymptotic stream of denormal values after the sticks
        # return to neutral. A neutral command must settle at an exact STOP.
        if target_lin == 0.0 and abs(self.smooth_lin) < 0.001:
            self.smooth_lin = 0.0
        if target_ang == 0.0 and abs(self.smooth_ang) < 0.001:
            self.smooth_ang = 0.0

        t = Twist()
        t.linear.x  = float(self.smooth_lin)
        t.angular.z = float(self.smooth_ang)
        self.pub_cmd.publish(t)

    def _connection_safety_tick(self):
        if self._startup_balance_off_remaining > 0:
            self._disable_balance(force=True)
            self._startup_balance_off_remaining -= 1
        else:
            # Authorized safety policy: keep firmware balance locked out even
            # after a late Arduino reconnect or an unexpected serial command.
            self._disable_balance()
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
            'max_linear_vel': self.max_linear_override,
            'max_angular_vel': self.max_angular_override,
            'kinect_tilt_mode': self.kinect_tilt_mode,
            'kinect_tilt_degrees': self.kinect_tilt_degrees,
            'kinect_tilt_neutral_armed': self._tilt_neutral_armed,
            'buttons': {
                'A': 'stadia_takeover',
                'B': 'stop',
                'X': 'toggle_kinect_tilt',
                'Y': 'balance_off_stop',
                'menu': 'stop',
            },
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
