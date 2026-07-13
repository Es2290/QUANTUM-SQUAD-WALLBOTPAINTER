"""Hardware simulator node — stands in for the ESP32 + physical robot.

This node closes the control loop entirely in software so the full
WallBot Painter ROS 2 stack can be exercised (and viewed in RViz) without
any physical Raspberry Pi / ESP32 / motors connected.

It plays the role that ``uart_bridge.py`` + the real ESP32 firmware would
normally play:

  Subscribes:
    /motor_rpm_cmd   (std_msgs/Float32MultiArray) — [FL, FR, RL, RR] RPM
    /odom            (nav_msgs/Odometry)           — used only to broadcast TF

  Publishes:
    /encoder_ticks   (std_msgs/Int32MultiArray)    — simulated cumulative ticks
    /imu/data        (sensor_msgs/Imu)             — simulated IMU acceleration
    TF: odom -> base_link                          — so RViz can show the robot moving

A ``slip_factor`` parameter (default 1.0 = no slip) can be lowered to
artificially desynchronise encoder vs. IMU velocity, which is handy for
testing the ``imu_slip_controller`` / ``safety_monitor`` fault paths without
real hardware.
"""

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

NUM_MOTORS = 4
FL, FR, RL, RR = 0, 1, 2, 3


class HardwareSimulator(Node):
    """Simulates ESP32 encoder/IMU feedback and broadcasts base_link TF."""

    def __init__(self) -> None:
        super().__init__('hardware_simulator')

        # ---------- Parameters (mirrors robot_params.yaml where relevant) ----------
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('encoder_ticks_per_rev', 4096)
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('slip_factor', 1.0)       # 1.0 = perfect traction
        self.declare_parameter('imu_noise_std', 0.01)    # m/s^2 gaussian noise

        self._wheel_radius: float = self.get_parameter('wheel_radius').value
        self._wheel_base: float = self.get_parameter('wheel_base').value
        self._ticks_per_rev: int = self.get_parameter('encoder_ticks_per_rev').value
        rate: float = self.get_parameter('update_rate_hz').value
        self._slip_factor: float = self.get_parameter('slip_factor').value
        self._imu_noise_std: float = self.get_parameter('imu_noise_std').value

        self._dt: float = 1.0 / rate

        # ---------- State ----------
        self._rpm_cmd: list[float] = [0.0] * NUM_MOTORS
        self._cumulative_ticks: list[int] = [0] * NUM_MOTORS
        self._tick_remainder: list[float] = [0.0] * NUM_MOTORS  # carry fractional ticks
        self._v_prev: float = 0.0

        # ---------- Publishers ----------
        self._encoder_pub = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu/data', 20)

        # ---------- Subscribers ----------
        self.create_subscription(
            Float32MultiArray, 'motor_rpm_cmd', self._on_motor_cmd, 10
        )
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)

        # ---------- TF ----------
        self._tf_broadcaster = TransformBroadcaster(self)

        # ---------- Timer: simulate encoder + IMU at fixed rate ----------
        self._timer = self.create_timer(self._dt, self._simulate_step)

        self.get_logger().info(
            f'HardwareSimulator started — standing in for ESP32 at {rate:.0f} Hz '
            f'(slip_factor={self._slip_factor:.2f}).'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _on_motor_cmd(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != NUM_MOTORS:
            self.get_logger().warn(
                f'Expected {NUM_MOTORS} motor RPM values, got {len(msg.data)}'
            )
            return
        self._rpm_cmd = list(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        """Relay odometry pose as a TF transform so RViz can render base_link."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Simulation step
    # ------------------------------------------------------------------

    def _simulate_step(self) -> None:
        self._publish_encoder_ticks()
        self._publish_imu()

    def _publish_encoder_ticks(self) -> None:
        """Integrate commanded RPM into cumulative encoder ticks (with slip)."""
        for i in range(NUM_MOTORS):
            rpm = self._rpm_cmd[i] * self._slip_factor
            ticks_per_sec = (rpm / 60.0) * self._ticks_per_rev
            exact = ticks_per_sec * self._dt + self._tick_remainder[i]
            delta = int(exact)
            self._tick_remainder[i] = exact - delta
            self._cumulative_ticks[i] += delta

        msg = Int32MultiArray()
        msg.data = list(self._cumulative_ticks)
        self._encoder_pub.publish(msg)

    def _publish_imu(self) -> None:
        """Derive a plausible forward acceleration from the *commanded* RPM
        (i.e. the 'true' motion, before any slip), so imu_slip_controller can
        compare it against the encoder-derived velocity (which does include
        slip_factor) to detect a mismatch."""
        v_left = self._mps_to_rpm_inverse(
            (self._rpm_cmd[FL] + self._rpm_cmd[RL]) / 2.0
        )
        v_right = self._mps_to_rpm_inverse(
            (self._rpm_cmd[FR] + self._rpm_cmd[RR]) / 2.0
        )
        v = (v_left + v_right) / 2.0

        accel = (v - self._v_prev) / self._dt if self._dt > 0 else 0.0
        self._v_prev = v

        if self._imu_noise_std > 0.0:
            accel += random.gauss(0.0, self._imu_noise_std)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        imu.linear_acceleration.x = accel
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        imu.angular_velocity.z = (v_right - v_left) / self._wheel_base
        # Identity orientation — not used by imu_slip_controller
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = 1.0
        self._imu_pub.publish(imu)

    def _mps_to_rpm_inverse(self, rpm: float) -> float:
        """RPM -> linear wheel speed (m/s). Inverse of MotorController._mps_to_rpm."""
        return (rpm / 60.0) * (2.0 * math.pi * self._wheel_radius)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
