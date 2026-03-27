"""IMU slip compensation controller.

Fuses encoder-derived velocity with IMU acceleration measurements to detect
wheel slip on the metallic surface.  When slip is detected a corrective
velocity adjustment is published so the motor controller can compensate in
real time, keeping the robot on its planned trajectory.

Algorithm (simplified):
    slip_ratio = (v_encoder - v_imu_integrated) / max(|v_encoder|, ε)

    If |slip_ratio| > slip_threshold:
        Publish corrected velocity = cmd_vel * (1 - slip_ratio)
        Emit a slip_detected event.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


class ImuSlipController(Node):
    """Real-time slip detection and velocity correction using IMU + encoders."""

    # Minimum speed below which slip detection is disabled (avoids division by ~0)
    _MIN_SPEED_MS = 0.01

    def __init__(self) -> None:
        super().__init__('imu_slip_controller')

        self.declare_parameter('slip_threshold', 0.15)   # 15 % slip ratio triggers correction
        self.declare_parameter('imu_topic', 'imu/data')
        self.declare_parameter('correction_gain', 0.8)   # damping factor for correction

        self._slip_threshold: float = self.get_parameter('slip_threshold').value
        self._correction_gain: float = self.get_parameter('correction_gain').value
        imu_topic: str = self.get_parameter('imu_topic').value

        # IMU-integrated state
        self._v_imu: float = 0.0          # velocity estimated from IMU integration
        self._last_imu_time = None

        # Encoder-derived velocity (from odometry)
        self._v_encoder: float = 0.0

        # Latest cmd_vel for producing corrected command
        self._cmd_vel: Twist = Twist()

        # Publishers
        self._corrected_cmd_pub = self.create_publisher(Twist, 'cmd_vel_corrected', 10)
        self._slip_detected_pub = self.create_publisher(Bool, 'slip_detected', 10)
        self._slip_ratio_pub = self.create_publisher(Float32, 'slip_ratio', 10)

        # Subscribers
        self.create_subscription(Imu, imu_topic, self._imu_callback, 20)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 20)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        self.get_logger().info('ImuSlipController started.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _imu_callback(self, msg: Imu) -> None:
        """Integrate IMU linear acceleration to estimate forward velocity."""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._last_imu_time is None:
            self._last_imu_time = now_sec
            return

        dt = now_sec - self._last_imu_time
        self._last_imu_time = now_sec

        if dt <= 0.0:
            return

        # Forward acceleration is along the robot X-axis (linear.x)
        ax = msg.linear_acceleration.x
        self._v_imu += ax * dt

        # Simple first-order decay (2 % per IMU sample) to avoid unbounded
        # integration drift caused by accelerometer bias.
        self._v_imu *= 0.98

        self._evaluate_slip()

    def _odom_callback(self, msg: Odometry) -> None:
        self._v_encoder = msg.twist.twist.linear.x

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._cmd_vel = msg

    # ------------------------------------------------------------------
    # Slip evaluation
    # ------------------------------------------------------------------

    def _evaluate_slip(self) -> None:
        """Compute slip ratio and publish corrected velocity if needed."""
        v_enc = self._v_encoder
        v_imu = self._v_imu

        slip_ratio = 0.0
        if abs(v_enc) > self._MIN_SPEED_MS:
            slip_ratio = (v_enc - v_imu) / abs(v_enc)
        elif abs(v_imu) > self._MIN_SPEED_MS:
            # Robot is sliding while encoders read zero
            slip_ratio = 1.0

        # Publish slip ratio for diagnostics
        ratio_msg = Float32()
        ratio_msg.data = float(slip_ratio)
        self._slip_ratio_pub.publish(ratio_msg)

        slip_detected = abs(slip_ratio) > self._slip_threshold
        flag = Bool()
        flag.data = slip_detected
        self._slip_detected_pub.publish(flag)

        if slip_detected:
            self.get_logger().warn(
                f'Slip detected — ratio={slip_ratio:.3f}, '
                f'v_enc={v_enc:.3f} m/s, v_imu={v_imu:.3f} m/s'
            )
            corrected = Twist()
            correction = 1.0 - self._correction_gain * slip_ratio
            correction = max(0.0, min(1.5, correction))  # bound correction factor
            corrected.linear.x = self._cmd_vel.linear.x * correction
            corrected.angular.z = self._cmd_vel.angular.z
            self._corrected_cmd_pub.publish(corrected)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuSlipController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
