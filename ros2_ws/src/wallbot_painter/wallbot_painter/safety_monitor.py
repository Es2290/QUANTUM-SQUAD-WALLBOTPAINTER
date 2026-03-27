"""Safety monitor node.

Monitors robot health and triggers protective actions:

1. **Low-battery RTH**: if battery voltage drops below a configurable
   threshold, painting is stopped and a Return-To-Home request is published.
2. **Critical battery shutdown**: at a second (lower) threshold all motors
   are stopped immediately to avoid full discharge damage.
3. **Slip fault**: if the slip controller signals persistent slip (> timeout),
   the robot is halted and an alert is raised.

Topics published:
    /rth_request        (std_msgs/Bool) — RTH flag for navigation node
    /painting_active    (std_msgs/Bool) — enable/disable spray
    /estop              (std_msgs/Bool) — emergency stop

Topics subscribed:
    /battery_voltage    (std_msgs/Float32) — battery voltage in Volts
    /slip_detected      (std_msgs/Bool)    — slip fault from IMU controller
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class SafetyMonitor(Node):
    """Monitor battery and slip faults; trigger RTH and emergency stop."""

    def __init__(self) -> None:
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('rth_voltage', 14.4)       # V — trigger RTH
        self.declare_parameter('critical_voltage', 13.2)  # V — emergency stop
        self.declare_parameter('slip_fault_timeout', 3.0) # s — persistent slip threshold
        self.declare_parameter('nominal_voltage', 16.8)   # V — 4S LiPo full

        self._rth_voltage: float = self.get_parameter('rth_voltage').value
        self._critical_voltage: float = self.get_parameter('critical_voltage').value
        self._slip_timeout: float = self.get_parameter('slip_fault_timeout').value
        self._nominal_voltage: float = self.get_parameter('nominal_voltage').value

        # State
        self._battery_voltage: float = self._nominal_voltage
        self._rth_triggered: bool = False
        self._estop_triggered: bool = False
        self._painting_on: bool = True
        self._slip_duration: float = 0.0  # seconds of continuous slip

        # Publishers
        self._rth_pub = self.create_publisher(Bool, 'rth_request', 10)
        self._painting_pub = self.create_publisher(Bool, 'painting_active', 10)
        self._estop_pub = self.create_publisher(Bool, 'estop', 10)

        # Subscribers
        self.create_subscription(Float32, 'battery_voltage', self._battery_callback, 10)
        self.create_subscription(Bool, 'slip_detected', self._slip_callback, 10)

        # Monitoring loop at 5 Hz
        self._timer = self.create_timer(0.2, self._monitor_loop)
        self._slip_timer_dt: float = 0.2  # seconds per monitor tick

        # Publish initial state — painting enabled, no emergencies
        self._publish_painting(True)

        self.get_logger().info(
            f'SafetyMonitor started — RTH@{self._rth_voltage}V, '
            f'ESTOP@{self._critical_voltage}V'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _battery_callback(self, msg: Float32) -> None:
        self._battery_voltage = msg.data

    def _slip_callback(self, msg: Bool) -> None:
        if msg.data:
            self._slip_duration += self._slip_timer_dt
        else:
            self._slip_duration = 0.0

    # ------------------------------------------------------------------
    # Monitor loop
    # ------------------------------------------------------------------

    def _monitor_loop(self) -> None:
        if self._estop_triggered:
            return  # already in emergency state

        battery_pct = (
            (self._battery_voltage - self._critical_voltage)
            / (self._nominal_voltage - self._critical_voltage)
            * 100.0
        )

        # --- Critical voltage: full emergency stop ---
        if self._battery_voltage <= self._critical_voltage:
            self.get_logger().error(
                f'CRITICAL battery voltage {self._battery_voltage:.2f} V — EMERGENCY STOP!'
            )
            self._estop_triggered = True
            self._publish_estop(True)
            self._publish_painting(False)
            self._publish_rth(False)
            return

        # --- Low voltage: initiate RTH ---
        if self._battery_voltage <= self._rth_voltage and not self._rth_triggered:
            self.get_logger().warning(
                f'Low battery {self._battery_voltage:.2f} V '
                f'({battery_pct:.0f} %) — triggering RTH.'
            )
            self._rth_triggered = True
            self._publish_painting(False)
            self._publish_rth(True)

        # --- Persistent slip fault ---
        if self._slip_duration >= self._slip_timeout and not self._estop_triggered:
            self.get_logger().error(
                f'Persistent slip for {self._slip_duration:.1f} s — halting robot!'
            )
            self._estop_triggered = True
            self._publish_estop(True)
            self._publish_painting(False)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_rth(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self._rth_pub.publish(msg)

    def _publish_painting(self, value: bool) -> None:
        self._painting_on = value
        msg = Bool()
        msg.data = value
        self._painting_pub.publish(msg)

    def _publish_estop(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self._estop_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
