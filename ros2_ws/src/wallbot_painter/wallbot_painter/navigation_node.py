"""Navigation node — runs on Raspberry Pi 5.

Responsible for high-level path planning and trajectory tracking on vertical
metallic surfaces.  It publishes velocity set-points consumed by the motor
controller and subscribes to odometry and IMU data to close the navigation
loop.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


class NavigationNode(Node):
    """Plan and track painting trajectories on a vertical metallic surface."""

    # Painting pattern: horizontal stripes with configurable width
    _STRIPE_WIDTH_M = 0.25  # metres — matches spray nozzle coverage

    def __init__(self) -> None:
        super().__init__('navigation_node')

        # Parameters
        self.declare_parameter('surface_width', 2.0)   # metres
        self.declare_parameter('surface_height', 3.0)  # metres
        self.declare_parameter('max_linear_speed', 0.15)   # m/s
        self.declare_parameter('max_angular_speed', 0.5)   # rad/s

        self._surface_width: float = self.get_parameter('surface_width').value
        self._surface_height: float = self.get_parameter('surface_height').value
        self._max_linear: float = self.get_parameter('max_linear_speed').value
        self._max_angular: float = self.get_parameter('max_angular_speed').value

        # Current robot pose from odometry
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        # Stripe-based waypoint list generated at start-up
        self._waypoints: list[tuple[float, float]] = self._generate_waypoints()
        self._current_wp: int = 0

        self._painting_active: bool = False
        self._rth_active: bool = False

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._target_pub = self.create_publisher(PoseStamped, 'current_waypoint', 10)

        # Subscribers
        self.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        self.create_subscription(Bool, 'rth_request', self._rth_callback, 10)
        self.create_subscription(Bool, 'painting_active', self._painting_callback, 10)

        # Control loop at 20 Hz
        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'NavigationNode started — {len(self._waypoints)} waypoints, '
            f'surface {self._surface_width:.2f} × {self._surface_height:.2f} m'
        )

    # ------------------------------------------------------------------
    # Waypoint generation
    # ------------------------------------------------------------------

    def _generate_waypoints(self) -> list[tuple[float, float]]:
        """Generate a boustrophedon (lawnmower) set of waypoints."""
        waypoints: list[tuple[float, float]] = []
        y = self._STRIPE_WIDTH_M / 2.0
        left_to_right = True
        while y < self._surface_height:
            if left_to_right:
                waypoints.append((0.0, y))
                waypoints.append((self._surface_width, y))
            else:
                waypoints.append((self._surface_width, y))
                waypoints.append((0.0, y))
            y += self._STRIPE_WIDTH_M
            left_to_right = not left_to_right
        # Return-to-home waypoint
        waypoints.append((0.0, 0.0))
        return waypoints

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def _rth_callback(self, msg: Bool) -> None:
        if msg.data and not self._rth_active:
            self.get_logger().warning('RTH requested — navigating back to base.')
            self._rth_active = True
            # Insert home as the next waypoint
            self._waypoints.insert(self._current_wp, (0.0, 0.0))

    def _painting_callback(self, msg: Bool) -> None:
        self._painting_active = msg.data

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self._current_wp >= len(self._waypoints):
            self._stop_robot()
            return

        target_x, target_y = self._waypoints[self._current_wp]
        dx = target_x - self._x
        dy = target_y - self._y
        distance = math.hypot(dx, dy)

        if distance < 0.05:  # within 5 cm → waypoint reached
            self.get_logger().info(
                f'Waypoint {self._current_wp} reached: ({target_x:.2f}, {target_y:.2f})'
            )
            self._current_wp += 1
            if self._rth_active and self._current_wp >= len(self._waypoints):
                self.get_logger().info('Home reached — shutting down navigation.')
                self._stop_robot()
            return

        # Pure-pursuit heading control
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._normalise_angle(desired_yaw - self._yaw)

        twist = Twist()
        twist.linear.x = min(self._max_linear, distance) * math.cos(yaw_error)
        twist.angular.z = max(-self._max_angular, min(self._max_angular, 2.0 * yaw_error))
        self._cmd_vel_pub.publish(twist)

        # Publish current target for visualisation
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'odom'
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        self._target_pub.publish(target_pose)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _stop_robot(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
