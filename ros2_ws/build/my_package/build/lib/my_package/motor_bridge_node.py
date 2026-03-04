#!/usr/bin/env python3
"""
motor_bridge_node.py

ROS2 node running on the Jetson that:
  1. Subscribes to /cmd_vel (geometry_msgs/Twist)
  2. Converts to wheel velocities (skid-steer kinematics)
  3. Runs PID using encoder feedback from ESP32
  4. Sends PWM commands to ESP32 over serial
  5. Publishes /odom (nav_msgs/Odometry) from encoder data

Serial protocol with ESP32:
  Jetson → ESP32:  "M <left_pwm> <right_pwm>\n"
  ESP32 → Jetson:  "E <fl> <fr> <bl> <br>\n"  (cumulative ticks)

Usage:
  ros2 run pioneer_base motor_bridge_node
  # or standalone:
  python3 motor_bridge_node.py
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial


class PIDController:
    """Simple PID with integral clamping and reset."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = 255.0, integral_limit: float = 150.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, setpoint: float, measured: float, dt: float) -> float:
        error = setpoint - measured

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative
        d_term = self.kd * ((error - self._prev_error) / dt) if dt > 0 else 0.0
        self._prev_error = error

        output = p_term + i_term + d_term
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


class MotorBridgeNode(Node):
    """ROS2 node bridging cmd_vel ↔ ESP32 serial ↔ odometry."""

    # ── Robot physical parameters (from your firmware) ────────────────────
    WHEEL_RADIUS    = 0.038       # metres
    WHEEL_BASE      = 0.24        # metres (track width)
    TICKS_PER_REV   = 440.0
    METRES_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

    # ── Control parameters ────────────────────────────────────────────────
    V_MAX           = 0.5         # m/s at PWM=255 (tune to your robot)
    STOP_THRESHOLD  = 0.01        # m/s — below this, treat as "stop"
    CMD_TIMEOUT     = 0.5         # seconds — stop if no cmd_vel

    # ── PID gains ─────────────────────────────────────────────────────────
    KP = 60.0
    KI = 30.0
    KD =  0.5

    # ── Rates ─────────────────────────────────────────────────────────────
    CONTROL_HZ = 50    # PID + serial send rate
    ODOM_HZ    = 20    # odometry publish rate

    def __init__(self):
        super().__init__('motor_bridge')

        # ── Declare parameters (overridable via launch file / CLI) ────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 460800)
        self.declare_parameter('kp', self.KP)
        self.declare_parameter('ki', self.KI)
        self.declare_parameter('kd', self.KD)
        self.declare_parameter('v_max', self.V_MAX)
        self.declare_parameter('wheel_radius', self.WHEEL_RADIUS)
        self.declare_parameter('wheel_base', self.WHEEL_BASE)
        self.declare_parameter('use_pid', True)
        self.declare_parameter('publish_tf', True)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self._use_pid = self.get_parameter('use_pid').value
        self._publish_tf = self.get_parameter('publish_tf').value

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self._v_max = self.get_parameter('v_max').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._wheel_base = self.get_parameter('wheel_base').value
        self._metres_per_tick = (2.0 * math.pi * self._wheel_radius) / self.TICKS_PER_REV

        # ── Serial connection ─────────────────────────────────────────────
        try:
            self._serial = serial.Serial(port, baud, timeout=0.001)
            self.get_logger().info(f'Opened serial: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port {port}: {e}')
            raise SystemExit(1)

        # ── PID controllers ───────────────────────────────────────────────
        self._pid_left = PIDController(kp, ki, kd)
        self._pid_right = PIDController(kp, ki, kd)

        # ── State ─────────────────────────────────────────────────────────
        self._setpoint_left  = 0.0  # m/s desired
        self._setpoint_right = 0.0
        self._measured_left  = 0.0  # m/s measured
        self._measured_right = 0.0

        self._prev_ticks_left  = 0
        self._prev_ticks_right = 0
        self._ticks_initialised = False

        # Odometry integration
        self._odom_x     = 0.0
        self._odom_y     = 0.0
        self._odom_theta = 0.0

        self._last_cmd_time = time.monotonic()
        self._lock = threading.Lock()

        # ── ROS2 pub/sub ──────────────────────────────────────────────────
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, qos)

        self._odom_pub = self.create_publisher(Odometry, '/odom', qos)

        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        # ── Timers ────────────────────────────────────────────────────────
        self._read_timer = self.create_timer(
            0.005, self._read_serial)       # 200 Hz serial read
        self._control_timer = self.create_timer(
            1.0 / self.CONTROL_HZ, self._control_loop)
        self._odom_timer = self.create_timer(
            1.0 / self.ODOM_HZ, self._publish_odometry)

        self.get_logger().info('Motor bridge node started')

    # ── cmd_vel callback ──────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive (skid-steer) kinematics
        with self._lock:
            self._setpoint_left  = v - w * (self._wheel_base / 2.0)
            self._setpoint_right = v + w * (self._wheel_base / 2.0)
            self._last_cmd_time = time.monotonic()

    # ── Main control loop (50 Hz) ─────────────────────────────────────────
    def _control_loop(self):
        now = time.monotonic()

        # Read all available encoder lines from ESP32
        #self._read_serial()

        with self._lock:
            sp_l = self._setpoint_left
            sp_r = self._setpoint_right
            last_cmd = self._last_cmd_time

        # Safety timeout
        if (now - last_cmd) > self.CMD_TIMEOUT:
            sp_l = 0.0
            sp_r = 0.0

        # Stop check
        if abs(sp_l) < self.STOP_THRESHOLD and abs(sp_r) < self.STOP_THRESHOLD:
            self._pid_left.reset()
            self._pid_right.reset()
            self._send_pwm(0, 0)
            return

        dt = 1.0 / self.CONTROL_HZ

        if self._use_pid:
            # PID: feedforward + feedback correction
            ff_left  = (sp_l / self._v_max) * 255.0
            ff_right = (sp_r / self._v_max) * 255.0
            fb_left  = self._pid_left.compute(sp_l, self._measured_left, dt)
            fb_right = self._pid_right.compute(sp_r, self._measured_right, dt)
            pwm_left  = int(max(-255, min(255, ff_left  + fb_left)))
            pwm_right = int(max(-255, min(255, ff_right + fb_right)))
        else:
            # Pure open-loop
            pwm_left  = int(max(-255, min(255, (sp_l / self._v_max) * 255.0)))
            pwm_right = int(max(-255, min(255, (sp_r / self._v_max) * 255.0)))

        self._send_pwm(pwm_left, pwm_right)

    # ── Serial I/O ────────────────────────────────────────────────────────
    def _send_pwm(self, left: int, right: int):
        """Send motor command to ESP32."""
        try:
            cmd = f"M {left} {right}\n"
            self._serial.write(cmd.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def _read_serial(self):
        """Read and parse all available encoder lines."""
        try:
            while self._serial.in_waiting:
                line = self._serial.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('E '):
                    parts = line.split()
                    if len(parts) == 5:
                        fl, fr, bl, br = (int(x) for x in parts[1:5])
                        self._process_encoder_ticks(fl, fr, bl, br)
        except (serial.SerialException, ValueError) as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def _process_encoder_ticks(self, fl: int, fr: int, bl: int, br: int):
        """Convert cumulative ticks to wheel velocities and integrate odometry."""
        # Average front+back for each side
        ticks_left  = (fl + bl) // 2
        ticks_right = (fr + br) // 2

        if not self._ticks_initialised:
            self._prev_ticks_left  = ticks_left
            self._prev_ticks_right = ticks_right
            self._ticks_initialised = True
            return

        delta_left  = ticks_left  - self._prev_ticks_left
        delta_right = ticks_right - self._prev_ticks_right
        self._prev_ticks_left  = ticks_left
        self._prev_ticks_right = ticks_right

        dt = 1.0 / self.CONTROL_HZ  # approximate — encoder stream is ~50 Hz

        dist_left  = delta_left  * self._metres_per_tick
        dist_right = delta_right * self._metres_per_tick

        self._measured_left  = dist_left  / dt if dt > 0 else 0.0
        self._measured_right = dist_right / dt if dt > 0 else 0.0

        # Differential drive odometry integration
        v_linear  = (dist_right + dist_left) / 2.0
        v_angular = (dist_right - dist_left) / self._wheel_base

        self._odom_theta += v_angular
        # Normalise to [-pi, pi]
        self._odom_theta = math.atan2(
            math.sin(self._odom_theta), math.cos(self._odom_theta))

        self._odom_x += v_linear * math.cos(self._odom_theta)
        self._odom_y += v_linear * math.sin(self._odom_theta)

    # ── Odometry publisher (20 Hz) ────────────────────────────────────────
    def _publish_odometry(self):
        now_stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = now_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        # Pose
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(self._odom_theta)

        # Twist (instantaneous)
        v_lin = (self._measured_left + self._measured_right) / 2.0
        v_ang = (self._measured_right - self._measured_left) / self._wheel_base
        odom.twist.twist.linear.x  = v_lin
        odom.twist.twist.angular.z = v_ang

        # Covariance (same as your original firmware)
        odom.pose.covariance[0]   = 0.001   # x
        odom.pose.covariance[7]   = 0.001   # y
        odom.pose.covariance[35]  = 0.01    # yaw
        odom.twist.covariance[0]  = 0.001   # vx
        odom.twist.covariance[35] = 0.01    # vyaw

        self._odom_pub.publish(odom)

        # TF broadcast: odom → base_link
        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp    = now_stamp
            t.header.frame_id = 'odom'
            t.child_frame_id  = 'base_link'
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.translation.z = 0.0
            q = self._yaw_to_quaternion(self._odom_theta)
            t.transform.rotation = q
            self._tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        """Clean shutdown: stop motors."""
        self.get_logger().info('Shutting down — stopping motors')
        self._send_pwm(0, 0)
        time.sleep(0.1)
        self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()