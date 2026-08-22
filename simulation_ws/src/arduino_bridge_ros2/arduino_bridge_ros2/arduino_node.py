#!/usr/bin/env python3
"""
arduino_node.py — ROS2 Jazzy bridge para MOTOR-INTERFACE-V14
Protocolo serial (115200 baud):
  PC → Arduino : v <linear m/s> <angular rad/s>
                 r  (reset encoders)
                 e  (request encoders)
                 s  (status)
  Arduino → PC : T lin=.. ang=.. Lpwm=.. Rpwm=.. Lrpm=.. Rrpm=.. Ld=F Rd=F LmA=.. RmA=..
                 e <L_total> <R_total>
                 s <0|1|2>

Topics publicados:
  /odom           nav_msgs/Odometry
  /motor_status   std_msgs/String   (telemetría T)
  /encoder_counts std_msgs/String   (e L R)

Topics suscritos:
  /cmd_vel        geometry_msgs/Twist
  /arduino/raw_command std_msgs/String
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, Temperature
from std_msgs.msg import String
import tf2_ros
import serial
import threading
import math
import time

from .encoder_math import wheel_encoder_signs
from .encoder_fusion import WheelEncoderFusion


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ── Parámetros ──────────────────────────────────────────────────
        self.declare_parameter('port',      '/dev/ttyACM0')
        self.declare_parameter('baud',      115200)
        self.declare_parameter('wheel_base', 0.82)    # metros entre ruedas
        self.declare_parameter('wheel_dia',  0.27)    # physical wheel diameter (m)
        self.declare_parameter('ppr',        60)      # encoder pulses per wheel revolution
        self.declare_parameter('cmd_timeout', 0.50)  # seconds without /cmd_vel

        self.port      = self.get_parameter('port').value
        self.baud      = self.get_parameter('baud').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_dia  = self.get_parameter('wheel_dia').value
        self.ppr        = self.get_parameter('ppr').value
        self.cmd_timeout = max(0.10, float(self.get_parameter('cmd_timeout').value))
        self._last_cmd_debug = 0.0
        self._last_motor_status_debug = 0.0

        # ── Odometría ────────────────────────────────────────────────────
        self.x = self.y = self.theta = 0.0
        self.raw_left_prev = None
        self.raw_right_prev = None
        self.hall_left_prev = None
        self.hall_right_prev = None
        self.left_fusion = WheelEncoderFusion()
        self.right_fusion = WheelEncoderFusion()
        self.enc_left_filtered = 0
        self.enc_right_filtered = 0
        self.enc_left_prev = 0
        self.enc_right_prev = 0
        self.last_encoder_time = None
        self.dist_per_pulse = (math.pi * self.wheel_dia) / self.ppr
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_cmd_time = time.monotonic()
        self.cmd_watchdog_stopped = True
        self.last_left_pwm = 0
        self.last_right_pwm = 0
        self.left_encoder_sign = 1
        self.right_encoder_sign = 1
        self.last_noise_warn = 0.0

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_odom   = self.create_publisher(Odometry, '/odom',           10)
        self.pub_status = self.create_publisher(String,   '/motor_status',   10)
        self.pub_enc    = self.create_publisher(String,   '/encoder_counts', 10)
        self.pub_encoder_fusion = self.create_publisher(
            String, '/encoder_fusion/status', 10)
        self.pub_raw_rx = self.create_publisher(String,   '/arduino/raw_rx', 10)
        self.pub_fix    = self.create_publisher(NavSatFix, '/fix',           10)
        self.pub_gps    = self.create_publisher(String,   '/gps/status',     10)
        self.pub_imu    = self.create_publisher(Imu,      '/imu/data_raw',   20)
        self.pub_imu_temp = self.create_publisher(
            Temperature, '/imu/temperature', 10)
        self.pub_imu_status = self.create_publisher(
            String, '/imu/status', 10)

        # ── TF broadcaster ───────────────────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscriber cmd_vel ───────────────────────────────────────────
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.sub_raw = self.create_subscription(String, '/arduino/raw_command',
                                                self.raw_command_cb, 10)

        # ── Timer para solicitar encoders cada 50ms ──────────────────────
        self.create_timer(0.05, self.request_encoders)
        self.create_timer(0.10, self.cmd_watchdog_cb)

        # ── Serial ───────────────────────────────────────────────────────
        self.ser = None
        self._open_serial()

        # ── Hilo lector ──────────────────────────────────────────────────
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f'Arduino bridge iniciado — {self.port} @ {self.baud}')

    # ── Serial ──────────────────────────────────────────────────────────
    def _open_serial(self):
        import os
        if not os.path.exists(self.port):
            now = time.monotonic()
            if not hasattr(self, '_last_port_warn') or now - self._last_port_warn > 10.0:
                self.get_logger().warn(f'Puerto serial {self.port} no existe. Esperando dispositivo...')
                self._last_port_warn = now
            self.ser = None
            return

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # esperar reset Arduino
            while self.ser.in_waiting:
                self.ser.readline()
            self.get_logger().info('Serial conectado')
        except Exception as e:
            self.get_logger().error(f'Error serial al conectar: {e}')
            self.ser = None

    def _handle_serial_disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _send(self, text: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text.strip() + '\n').encode())
            except Exception as e:
                self.get_logger().warn(f'Error write: {e}. Desconectando serial para reconexión.')
                self._handle_serial_disconnect()

    # ── cmd_vel → Arduino ────────────────────────────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        self.left_encoder_sign, self.right_encoder_sign = wheel_encoder_signs(
            v, w, self.wheel_base,
            self.left_encoder_sign, self.right_encoder_sign)
        self.last_cmd_linear = v
        self.last_cmd_angular = w
        self.last_cmd_time = time.monotonic()
        self.cmd_watchdog_stopped = abs(v) < 1e-6 and abs(w) < 1e-6
        self._send(f'v {v:.4f} {w:.4f}')
        now = time.monotonic()
        if (
            (abs(v) > 1e-4 or abs(w) > 1e-4)
            and now - self._last_cmd_debug >= 1.0
        ):
            self._last_cmd_debug = now
            self.get_logger().info(f'Arduino TX cmd_vel: v={v:.3f} w={w:.3f}')

    def cmd_watchdog_cb(self):
        age = time.monotonic() - self.last_cmd_time
        if age <= self.cmd_timeout or self.cmd_watchdog_stopped:
            return
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.cmd_watchdog_stopped = True
        self._send('v 0.0 0.0')
        self.get_logger().warn(
            f'cmd_vel timeout after {age:.2f}s; motors stopped'
        )

    def raw_command_cb(self, msg: String):
        cmd = msg.data.strip()
        if not cmd or '\n' in cmd or '\r' in cmd or len(cmd) > 96:
            self.get_logger().warn('Comando raw ignorado: formato invalido')
            return
        self._send(cmd)

    # ── Timer: pedir encoders ────────────────────────────────────────────
    def request_encoders(self):
        self._send('e')

    # ── Hilo RX ──────────────────────────────────────────────────────────
    def _rx_loop(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(2.0)
                if self.running and (not self.ser or not self.ser.is_open):
                    self._open_serial()
                continue
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                self.pub_raw_rx.publish(String(data=line))
                if line.startswith('T '):
                    self._parse_motor_status(line)
                    self.pub_status.publish(String(data=line))
                    now = time.monotonic()
                    if now - self._last_motor_status_debug >= 1.0:
                        self._last_motor_status_debug = now
                        self.get_logger().info(f'Arduino RX motor status: {line}')
                elif line.startswith('e '):
                    self._process_encoders(line)
                elif line.startswith('G '):
                    self._process_gps(line)
                elif line.startswith('I '):
                    self._process_imu(line)
            except Exception as e:
                self.get_logger().warn(f'Error read: {e}. Desconectando serial para reconexión.')
                self._handle_serial_disconnect()
                time.sleep(1.0)

    def _parse_motor_status(self, line: str):
        parts = {}
        for token in line.split()[1:]:
            if '=' not in token:
                continue
            key, value = token.split('=', 1)
            parts[key] = value
        try:
            self.last_left_pwm = int(float(parts.get('Lpwm', self.last_left_pwm)))
            self.last_right_pwm = int(float(parts.get('Rpwm', self.last_right_pwm)))
        except ValueError:
            return

    # ── Odometría desde encoders ─────────────────────────────────────────
    def _process_gps(self, line: str):
        parts = {}
        for token in line.split()[1:]:
            if '=' not in token:
                continue
            key, value = token.split('=', 1)
            parts[key] = value

        self.pub_gps.publish(String(data=line))
        fix_ok = parts.get('fix') in ('1', 'true', 'True')
        if not fix_ok or 'lat' not in parts or 'lon' not in parts:
            return

        try:
            lat = float(parts['lat'])
            lon = float(parts['lon'])
            hdop = float(parts.get('hdop', '0') or 0)
        except ValueError:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = float('nan')

        variance = max(1.0, hdop * hdop) if hdop > 0 else 25.0
        msg.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance * 4.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_fix.publish(msg)

    def _process_imu(self, line: str):
        parts = {}
        for token in line.split()[1:]:
            if '=' in token:
                key, value = token.split('=', 1)
                parts[key] = value
        ready = parts.get('ready') in ('1', 'true', 'True')
        self.pub_imu_status.publish(String(data=line))
        if not ready:
            return
        try:
            ax, ay, az = (float(parts[key]) for key in ('ax', 'ay', 'az'))
            gx, gy, gz = (float(parts[key]) for key in ('gx', 'gy', 'gz'))
            yaw = float(parts['yaw'])
            temperature = float(parts.get('temp', 'nan'))
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(f'Invalid IMU telemetry: {line}')
            return

        stamp = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'
        deg_to_rad = math.pi / 180.0
        orientation = _yaw_to_quat(yaw * deg_to_rad)
        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]
        msg.orientation_covariance = [
            1.0e6, 0.0, 0.0,
            0.0, 1.0e6, 0.0,
            0.0, 0.0, 0.02,
        ]
        msg.angular_velocity.x = gx * deg_to_rad
        msg.angular_velocity.y = gy * deg_to_rad
        msg.angular_velocity.z = gz * deg_to_rad
        gravity = 9.80665
        msg.linear_acceleration.x = ax * gravity
        msg.linear_acceleration.y = ay * gravity
        msg.linear_acceleration.z = az * gravity
        msg.angular_velocity_covariance = [
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0012,
        ]
        msg.linear_acceleration_covariance = [
            0.09, 0.0, 0.0,
            0.0, 0.09, 0.0,
            0.0, 0.0, 0.16,
        ]
        self.pub_imu.publish(msg)

        temp = Temperature()
        temp.header.stamp = stamp
        temp.header.frame_id = 'imu_link'
        temp.temperature = temperature
        temp.variance = 1.0
        self.pub_imu_temp.publish(temp)

    def _process_encoders(self, line: str):
        # Extended format: e <optoL> <optoR> <hallL> <hallR>.
        # Legacy two-counter frames remain accepted without fusion.
        try:
            parts = line.split()
            l_opto = int(parts[1])
            r_opto = int(parts[2])
            dual_frame = len(parts) >= 5
            l_hall = int(parts[3]) if dual_frame else None
            r_hall = int(parts[4]) if dual_frame else None
        except (ValueError, IndexError):
            return

        first = self.raw_left_prev is None or self.raw_right_prev is None
        if dual_frame:
            first = first or self.hall_left_prev is None or self.hall_right_prev is None
        if first:
            self.raw_left_prev, self.raw_right_prev = l_opto, r_opto
            self.hall_left_prev, self.hall_right_prev = l_hall, r_hall
            self.last_encoder_time = time.monotonic()
            self.pub_enc.publish(String(data='L=0.000 R=0.000 initializing=1'))
            return

        raw_dl = l_opto - self.raw_left_prev
        raw_dr = r_opto - self.raw_right_prev
        self.raw_left_prev, self.raw_right_prev = l_opto, r_opto
        if dual_frame:
            hall_dl = l_hall - self.hall_left_prev
            hall_dr = r_hall - self.hall_right_prev
            self.hall_left_prev, self.hall_right_prev = l_hall, r_hall
        else:
            hall_dl = hall_dr = 0

        if raw_dl < 0 or raw_dr < 0 or hall_dl < 0 or hall_dr < 0:
            self.left_fusion.reset()
            self.right_fusion.reset()
            return

        left_moving = self.last_left_pwm > 0
        right_moving = self.last_right_pwm > 0
        if dual_frame:
            left = self.left_fusion.update(raw_dl, hall_dl, left_moving)
            right = self.right_fusion.update(raw_dr, hall_dr, right_moving)
        else:
            left = {'delta': raw_dl if left_moving else 0.0, 'source': 'OPTO_LEGACY',
                    'confidence': 0.5, 'error': -1.0,
                    'opto_window': raw_dl, 'hall_window': 0.0}
            right = {'delta': raw_dr if right_moving else 0.0, 'source': 'OPTO_LEGACY',
                     'confidence': 0.5, 'error': -1.0,
                     'opto_window': raw_dr, 'hall_window': 0.0}

        use_dl = left['delta'] * self.left_encoder_sign
        use_dr = right['delta'] * self.right_encoder_sign
        if not left_moving and raw_dl != 0:
            self._warn_encoder_noise('left', raw_dl, l_opto)
        if not right_moving and raw_dr != 0:
            self._warn_encoder_noise('right', raw_dr, r_opto)

        self.enc_left_filtered += use_dl
        self.enc_right_filtered += use_dr
        dl = use_dl * self.dist_per_pulse
        dr = use_dr * self.dist_per_pulse
        d_center = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.wheel_base
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        encoder_now = time.monotonic()
        dt = 0.05 if self.last_encoder_time is None else max(
            0.01, min(0.20, encoder_now - self.last_encoder_time))
        self.last_encoder_time = encoder_now
        now = self.get_clock().now().to_msg()
        q = _yaw_to_quat(self.theta)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
        tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = q[0], q[1]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q[2], q[3]
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt
        confidence = max(0.1, min(left['confidence'], right['confidence']))
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        for index, value in {0: 0.02, 7: 0.02, 14: 1.0e6,
                             21: 1.0e6, 28: 1.0e6, 35: 0.04}.items():
            odom.pose.covariance[index] = value / confidence if value < 1.0e6 else value
        for index, value in {0: 0.04, 7: 1.0e6, 14: 1.0e6,
                             21: 1.0e6, 28: 1.0e6, 35: 0.08}.items():
            odom.twist.covariance[index] = value / confidence if value < 1.0e6 else value
        self.pub_odom.publish(odom)

        status = (
            f"Lsrc={left['source']} Lconf={left['confidence']:.2f} "
            f"Lerr={left['error']:.3f} Rsrc={right['source']} "
            f"Rconf={right['confidence']:.2f} Rerr={right['error']:.3f}"
        )
        self.pub_encoder_fusion.publish(String(data=status))
        self.pub_enc.publish(String(data=(
            f'L={self.enc_left_filtered:.3f} R={self.enc_right_filtered:.3f} '
            f'OL={l_opto} OR={r_opto} HL={l_hall} HR={r_hall}'
        )))
    def _warn_encoder_noise(self, side: str, delta: int, raw_count: int):
        now = time.monotonic()
        if now - self.last_noise_warn < 2.0:
            return
        self.last_noise_warn = now
        self.get_logger().warn(
            f'Ignoring {side} encoder delta while PWM=0: delta={delta} raw={raw_count}'
        )

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self._send('v 0.0 0.0')
            self.ser.close()
        super().destroy_node()


def _yaw_to_quat(yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
