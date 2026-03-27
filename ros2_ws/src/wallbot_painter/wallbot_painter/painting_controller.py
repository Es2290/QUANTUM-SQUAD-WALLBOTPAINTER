"""Painting controller node.

Controls the automatic spray diffuser.  The paint flow rate is servo-controlled
to the **actual linear speed** of the robot so that paint coverage remains
uniform regardless of speed variations.  The controller publishes a normalised
duty-cycle command [0.0 – 1.0] to the ``spray_duty`` topic which is consumed
by the ESP32 firmware to drive the solenoid valve / pump PWM.

Flow-rate law:
    duty = clamp(k * v_actual / v_nominal, 0, 1)

where:
    k          — coverage gain (configurable)
    v_actual   — actual robot speed from odometry (m/s)
    v_nominal  — nominal design speed (m/s)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


class PaintingController(Node):
    """Servo-control spray flow rate proportionally to actual robot speed."""

    def __init__(self) -> None:
        super().__init__('painting_controller')

        self.declare_parameter('nominal_speed', 0.10)      # m/s — design speed
        self.declare_parameter('coverage_gain', 1.0)       # unitless
        self.declare_parameter('min_duty', 0.05)           # keep nozzle slightly open
        self.declare_parameter('speed_deadband', 0.005)    # m/s — below this = no spray

        self._nominal_speed: float = self.get_parameter('nominal_speed').value
        self._coverage_gain: float = self.get_parameter('coverage_gain').value
        self._min_duty: float = self.get_parameter('min_duty').value
        self._speed_deadband: float = self.get_parameter('speed_deadband').value

        self._painting_active: bool = False
        self._current_speed: float = 0.0

        # Publishers
        self._spray_duty_pub = self.create_publisher(Float32, 'spray_duty', 10)

        # Subscribers
        self.create_subscription(Odometry, 'odom', self._odom_callback, 20)
        self.create_subscription(Bool, 'painting_active', self._active_callback, 10)

        # 50 Hz control loop
        self._timer = self.create_timer(0.02, self._control_loop)

        self.get_logger().info('PaintingController started.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_speed = abs(msg.twist.twist.linear.x)

    def _active_callback(self, msg: Bool) -> None:
        self._painting_active = msg.data
        if not msg.data:
            self._publish_duty(0.0)
            self.get_logger().info('Spray OFF.')
        else:
            self.get_logger().info('Spray ON.')

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if not self._painting_active:
            return

        if self._current_speed < self._speed_deadband:
            # Robot is stopped — close spray to avoid paint pooling
            self._publish_duty(0.0)
            return

        duty = self._coverage_gain * self._current_speed / self._nominal_speed
        duty = max(self._min_duty, min(1.0, duty))
        self._publish_duty(duty)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_duty(self, duty: float) -> None:
        msg = Float32()
        msg.data = float(duty)
        self._spray_duty_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PaintingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
