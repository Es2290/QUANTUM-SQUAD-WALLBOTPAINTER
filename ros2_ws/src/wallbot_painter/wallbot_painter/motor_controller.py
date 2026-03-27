"""Motor controller node — FOC ESC interface for four brushless motors.

Converts ``cmd_vel`` (Twist) set-points into individual wheel velocity
commands sent to the four FOC ESC controllers via UART/CAN.  Encoder
feedback from the ESP32 is used to close the speed loop and publish
odometry.

Motor layout (top view, robot climbing upward):
    FL (front-left)    FR (front-right)
    RL (rear-left)     RR (rear-right)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int32MultiArray


# Number of brushless motors
NUM_MOTORS = 4

# Indices into motor arrays
FL, FR, RL, RR = 0, 1, 2, 3


class MotorController(Node):
    """Differential-drive motor controller for four brushless FOC ESC units."""

    def __init__(self) -> None:
        super().__init__('motor_controller')

        # ---------- Parameters ----------
        self.declare_parameter('wheel_radius', 0.05)       # metres
        self.declare_parameter('wheel_base', 0.30)         # metres (track width)
        self.declare_parameter('max_rpm', 300.0)
        self.declare_parameter('encoder_ticks_per_rev', 4096)

        self._wheel_radius: float = self.get_parameter('wheel_radius').value
        self._wheel_base: float = self.get_parameter('wheel_base').value
        self._max_rpm: float = self.get_parameter('max_rpm').value
        self._ticks_per_rev: int = self.get_parameter('encoder_ticks_per_rev').value

        # Odometry state
        self._x: float = 0.0
        self._y: float = 0.0
        self._theta: float = 0.0
        self._last_encoder_ticks: list[int] = [0] * NUM_MOTORS
        self._last_time = self.get_clock().now()

        # ---------- Publishers ----------
        self._motor_cmd_pub = self.create_publisher(
            Float32MultiArray, 'motor_rpm_cmd', 10
        )
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # ---------- Subscribers ----------
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(
            Int32MultiArray, 'encoder_ticks', self._encoder_callback, 10
        )

        self.get_logger().info('MotorController started — 4× brushless FOC ESC ready.')

    # ------------------------------------------------------------------
    # Velocity command → individual wheel RPM
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist) -> None:
        v = msg.linear.x   # m/s — forward
        w = msg.angular.z  # rad/s — yaw rate

        # Differential-drive kinematics
        v_left = v - (w * self._wheel_base / 2.0)
        v_right = v + (w * self._wheel_base / 2.0)

        rpm_left = self._mps_to_rpm(v_left)
        rpm_right = self._mps_to_rpm(v_right)

        rpm_left = self._clamp(rpm_left, -self._max_rpm, self._max_rpm)
        rpm_right = self._clamp(rpm_right, -self._max_rpm, self._max_rpm)

        cmd = Float32MultiArray()
        cmd.data = [
            rpm_left,   # FL
            rpm_right,  # FR
            rpm_left,   # RL
            rpm_right,  # RR
        ]
        self._motor_cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # Encoder feedback → odometry
    # ------------------------------------------------------------------

    def _encoder_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != NUM_MOTORS:
            self.get_logger().warn(
                f'Expected {NUM_MOTORS} encoder values, got {len(msg.data)}'
            )
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_time = now

        # Average left and right encoder deltas
        delta_ticks = [
            msg.data[i] - self._last_encoder_ticks[i] for i in range(NUM_MOTORS)
        ]
        self._last_encoder_ticks = list(msg.data)

        delta_left = (delta_ticks[FL] + delta_ticks[RL]) / 2.0
        delta_right = (delta_ticks[FR] + delta_ticks[RR]) / 2.0

        dist_left = (delta_left / self._ticks_per_rev) * 2.0 * math.pi * self._wheel_radius
        dist_right = (delta_right / self._ticks_per_rev) * 2.0 * math.pi * self._wheel_radius

        delta_dist = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self._wheel_base

        self._theta += delta_theta
        self._x += delta_dist * math.cos(self._theta)
        self._y += delta_dist * math.sin(self._theta)

        self._publish_odometry(now, delta_dist / dt, delta_theta / dt)

    def _publish_odometry(self, stamp, v: float, w: float) -> None:
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y

        cy = math.cos(self._theta * 0.5)
        sy = math.sin(self._theta * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self._odom_pub.publish(odom)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _mps_to_rpm(self, v: float) -> float:
        """Convert linear wheel speed (m/s) to motor RPM."""
        return (v / (2.0 * math.pi * self._wheel_radius)) * 60.0

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
