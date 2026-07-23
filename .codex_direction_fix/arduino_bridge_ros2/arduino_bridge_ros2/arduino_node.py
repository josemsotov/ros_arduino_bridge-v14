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
from std_msgs.msg import String
import tf2_ros
import serial
import threading
import math
import time


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ── Parámetros ──────────────────────────────────────────────────
        self.declare_parameter('port',      '/dev/ttyACM0')
        self.declare_parameter('baud',      115200)
        self.declare_parameter('wheel_base', 0.82)    # metros entre ruedas
        self.declare_parameter('wheel_dia',  0.20)    # metros diámetro rueda
        self.declare_parameter('ppr',        45)      # pulsos por revolución

        self.port      = self.get_parameter('port').value
        self.baud      = self.get_parameter('baud').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_dia  = self.get_parameter('wheel_dia').value
        self.ppr        = self.get_parameter('ppr').value

        # ── Odometría ────────────────────────────────────────────────────
        self.x = self.y = self.theta = 0.0
        self.enc_left_prev  = 0
        self.enc_right_prev = 0
        self.dist_per_pulse = (math.pi * self.wheel_dia) / self.ppr

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_odom   = self.create_publisher(Odometry, '/odom',           10)
        self.pub_status = self.create_publisher(String,   '/motor_status',   10)
        self.pub_enc    = self.create_publisher(String,   '/encoder_counts', 10)
        self.pub_raw_rx = self.create_publisher(String,   '/arduino/raw_rx', 10)

        # ── TF broadcaster ───────────────────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscriber cmd_vel ───────────────────────────────────────────
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.sub_raw = self.create_subscription(String, '/arduino/raw_command',
                                                self.raw_command_cb, 10)

        # ── Timer para solicitar encoders cada 50ms ──────────────────────
        self.create_timer(0.05, self.request_encoders)

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
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # esperar reset Arduino
            while self.ser.in_waiting:
                self.ser.readline()
            self.get_logger().info('Serial conectado')
        except Exception as e:
            self.get_logger().error(f'Error serial: {e}')
            self.ser = None

    def _send(self, text: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text.strip() + '\n').encode())
            except Exception as e:
                self.get_logger().warn(f'Error write: {e}')

    # ── cmd_vel → Arduino ────────────────────────────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        self._send(f'v {v:.4f} {w:.4f}')

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
                time.sleep(1)
                continue
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                self.pub_raw_rx.publish(String(data=line))
                if line.startswith('T '):
                    self.pub_status.publish(String(data=line))
                elif line.startswith('e '):
                    self._process_encoders(line)
            except Exception:
                pass

    # ── Odometría desde encoders ─────────────────────────────────────────
    def _process_encoders(self, line: str):
        # formato: e <L_total> <R_total>
        try:
            parts = line.split()
            l_enc = int(parts[1])
            r_enc = int(parts[2])
        except Exception:
            return

        dl = (l_enc - self.enc_left_prev)  * self.dist_per_pulse
        dr = (r_enc - self.enc_right_prev) * self.dist_per_pulse
        self.enc_left_prev  = l_enc
        self.enc_right_prev = r_enc

        d_center = (dl + dr) / 2.0
        d_theta  = (dr - dl) / self.wheel_base

        self.theta += d_theta
        self.x     += d_center * math.cos(self.theta)
        self.y     += d_center * math.sin(self.theta)

        now = self.get_clock().now().to_msg()

        # TF odom → base_link
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        q = _yaw_to_quat(self.theta)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

        # Odometry msg
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        v_lin = d_center / 0.05  # dt ≈ 50ms
        v_ang = d_theta  / 0.05
        odom.twist.twist.linear.x  = v_lin
        odom.twist.twist.angular.z = v_ang
        self.pub_odom.publish(odom)

        self.pub_enc.publish(String(data=f'L={l_enc} R={r_enc}'))

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
