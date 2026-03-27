"""Unit tests for WallBot Painter ROS 2 nodes.

These tests exercise the pure-Python logic of each node without requiring a
running ROS 2 instance.  The rclpy API is not called directly — instead, the
internal helper methods and state-machine logic are tested in isolation.
"""

import math
import sys
import types
import unittest

# ---------------------------------------------------------------------------
# Minimal rclpy stub so that the node modules can be imported without a live
# ROS 2 installation.
# ---------------------------------------------------------------------------

def _make_rclpy_stub() -> None:
    """Inject a minimal rclpy stub into sys.modules."""
    rclpy_mod = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class FakeNode:
        def __init__(self, name):
            self._name = name
            self._params: dict = {}
            self._logger = _FakeLogger(name)

        # parameter API
        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            class _P:
                def __init__(self, v):
                    self.value = v
            return _P(self._params[name])

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return _FakeClock()

        def create_publisher(self, *a, **kw):
            return _FakePublisher()

        def create_subscription(self, *a, **kw):
            pass

        def create_timer(self, *a, **kw):
            pass

        def destroy_node(self):
            pass

    class _FakeLogger:
        def __init__(self, name):
            self._name = name
        def info(self, msg): pass
        def warn(self, msg): pass
        def warning(self, msg): pass
        def error(self, msg): pass

    class _FakeClock:
        def now(self):
            return _FakeTime()

    class _FakeTime:
        nanoseconds = 0
        def to_msg(self):
            return None

    class _FakePublisher:
        def __init__(self):
            self.last_msg = None
        def publish(self, msg):
            self.last_msg = msg

    node_mod.Node = FakeNode
    rclpy_mod.node = node_mod
    rclpy_mod.init = lambda args=None: None
    rclpy_mod.spin = lambda node: None
    rclpy_mod.shutdown = lambda: None

    sys.modules['rclpy'] = rclpy_mod
    sys.modules['rclpy.node'] = node_mod

    # Stub message types
    for pkg in ('std_msgs', 'geometry_msgs', 'sensor_msgs', 'nav_msgs'):
        mod = types.ModuleType(pkg)
        msg_mod = types.ModuleType(f'{pkg}.msg')

        class _Msg:  # generic message stub
            def __init__(self, **kw):
                for k, v in kw.items():
                    setattr(self, k, v)

        class _Vec3:
            x = 0.0
            y = 0.0
            z = 0.0

        class _Quat:
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0

        class _Point:
            x = 0.0
            y = 0.0
            z = 0.0

        class _Pose:
            def __init__(self):
                self.position = _Point()
                self.orientation = _Quat()

        class _PoseWithCovariance:
            def __init__(self):
                self.pose = _Pose()

        class _TwistInner:
            def __init__(self):
                self.linear = _Vec3()
                self.angular = _Vec3()

        class _TwistWithCovariance:
            def __init__(self):
                self.twist = _TwistInner()

        class _Header:
            def __init__(self):
                self.stamp = None
                self.frame_id = ''

        class Twist:
            def __init__(self):
                self.linear = _Vec3()
                self.angular = _Vec3()

        class Odometry:
            def __init__(self):
                self.header = _Header()
                self.child_frame_id = ''
                self.pose = _PoseWithCovariance()
                self.twist = _TwistWithCovariance()

        class PoseStamped:
            def __init__(self):
                self.header = _Header()
                self.pose = _Pose()

        class Imu:
            def __init__(self):
                self.header = _Header()
                self.linear_acceleration = _Vec3()
                self.angular_velocity = _Vec3()
                self.orientation = _Quat()

        for cls_name in ('Bool', 'Float32', 'Float32MultiArray', 'Int32MultiArray'):
            setattr(msg_mod, cls_name, type(cls_name, (), {
                '__init__': lambda self: None,
            }))

        msg_mod.Twist = Twist
        msg_mod.Odometry = Odometry
        msg_mod.PoseStamped = PoseStamped
        msg_mod.Imu = Imu

        mod.msg = msg_mod
        sys.modules[pkg] = mod
        sys.modules[f'{pkg}.msg'] = msg_mod


_make_rclpy_stub()

# Now we can import the node modules
import importlib
import importlib.util
import os

_PKG = os.path.join(
    os.path.dirname(__file__),
    '..', 'wallbot_painter'
)


def _load(module_name: str):
    path = os.path.join(_PKG, f'{module_name}.py')
    spec = importlib.util.spec_from_file_location(module_name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# NavigationNode tests
# ---------------------------------------------------------------------------

class TestNavigationHelpers(unittest.TestCase):

    def setUp(self):
        self.nav_mod = _load('navigation_node')

    def test_normalise_angle_within_range(self):
        norm = self.nav_mod.NavigationNode._normalise_angle
        for angle in (0.0, math.pi - 0.001, -math.pi + 0.001):
            result = norm(angle)
            self.assertGreaterEqual(result, -math.pi)
            self.assertLessEqual(result, math.pi)

    def test_normalise_angle_wraps_positive(self):
        norm = self.nav_mod.NavigationNode._normalise_angle
        result = norm(3 * math.pi)
        self.assertAlmostEqual(result, math.pi, places=6)

    def test_normalise_angle_wraps_negative(self):
        norm = self.nav_mod.NavigationNode._normalise_angle
        result = norm(-3 * math.pi)
        self.assertAlmostEqual(result, -math.pi, places=6)

    def test_generate_waypoints_covers_surface(self):
        node = self.nav_mod.NavigationNode.__new__(self.nav_mod.NavigationNode)
        # Manually set required attributes
        node._params = {
            'surface_width': 2.0,
            'surface_height': 1.0,
            'max_linear_speed': 0.15,
            'max_angular_speed': 0.5,
        }
        node._surface_width = 2.0
        node._surface_height = 1.0
        waypoints = node._generate_waypoints()
        # Should produce at least two waypoints per stripe, plus home
        self.assertGreaterEqual(len(waypoints), 3)
        # Last waypoint must be home (0, 0)
        self.assertEqual(waypoints[-1], (0.0, 0.0))

    def test_generate_waypoints_alternates_sides(self):
        node = self.nav_mod.NavigationNode.__new__(self.nav_mod.NavigationNode)
        node._surface_width = 2.0
        node._surface_height = 0.75
        wps = node._generate_waypoints()
        # First stripe: left→right (x starts at 0)
        self.assertAlmostEqual(wps[0][0], 0.0)
        # Second point is right end of stripe
        self.assertAlmostEqual(wps[1][0], 2.0)


# ---------------------------------------------------------------------------
# MotorController tests
# ---------------------------------------------------------------------------

class TestMotorController(unittest.TestCase):

    def setUp(self):
        self.mc_mod = _load('motor_controller')

    def test_clamp_within_bounds(self):
        clamp = self.mc_mod.MotorController._clamp
        self.assertEqual(clamp(5.0, 0.0, 10.0), 5.0)

    def test_clamp_above_max(self):
        clamp = self.mc_mod.MotorController._clamp
        self.assertEqual(clamp(15.0, 0.0, 10.0), 10.0)

    def test_clamp_below_min(self):
        clamp = self.mc_mod.MotorController._clamp
        self.assertEqual(clamp(-5.0, 0.0, 10.0), 0.0)

    def test_mps_to_rpm_zero(self):
        mc = self.mc_mod.MotorController.__new__(self.mc_mod.MotorController)
        mc._params = {
            'wheel_radius': 0.05,
            'wheel_base': 0.30,
            'max_rpm': 300.0,
            'encoder_ticks_per_rev': 4096,
        }
        mc._wheel_radius = 0.05
        self.assertAlmostEqual(mc._mps_to_rpm(0.0), 0.0)

    def test_mps_to_rpm_positive(self):
        mc = self.mc_mod.MotorController.__new__(self.mc_mod.MotorController)
        mc._wheel_radius = 0.05
        rpm = mc._mps_to_rpm(0.05 * 2.0 * math.pi)  # one rev per second
        self.assertAlmostEqual(rpm, 60.0, places=3)


# ---------------------------------------------------------------------------
# ImuSlipController tests
# ---------------------------------------------------------------------------

class TestImuSlipController(unittest.TestCase):

    def setUp(self):
        self.slip_mod = _load('imu_slip_controller')

    def _make_controller(self):
        ctrl = self.slip_mod.ImuSlipController.__new__(
            self.slip_mod.ImuSlipController
        )
        ctrl._params = {
            'slip_threshold': 0.15,
            'imu_topic': 'imu/data',
            'correction_gain': 0.8,
        }
        ctrl._slip_threshold = 0.15
        ctrl._correction_gain = 0.8
        ctrl._v_imu = 0.0
        ctrl._v_encoder = 0.0
        ctrl._last_imu_time = None

        class _FakePub:
            def __init__(self): self.last_msg = None
            def publish(self, msg): self.last_msg = msg

        ctrl._corrected_cmd_pub = _FakePub()
        ctrl._slip_detected_pub = _FakePub()
        ctrl._slip_ratio_pub = _FakePub()

        # Stub logger
        class _L:
            def warn(self, m): pass
            def warning(self, m): pass
            def info(self, m): pass
            def error(self, m): pass
        ctrl._logger = _L()

        # Attach get_logger
        ctrl.get_logger = lambda: ctrl._logger

        # Minimal cmd_vel
        class _Twist:
            linear = type('L', (), {'x': 0.1})()
            angular = type('A', (), {'z': 0.0})()
        ctrl._cmd_vel = _Twist()

        return ctrl

    def test_no_slip_when_equal_speeds(self):
        ctrl = self._make_controller()
        ctrl._v_encoder = 0.10
        ctrl._v_imu = 0.10
        ctrl._evaluate_slip()
        slip_msg = ctrl._slip_detected_pub.last_msg
        self.assertFalse(slip_msg.data)

    def test_slip_detected_on_large_discrepancy(self):
        ctrl = self._make_controller()
        ctrl._v_encoder = 0.10
        ctrl._v_imu = 0.0  # IMU says robot is not moving → encoder slip
        ctrl._evaluate_slip()
        slip_msg = ctrl._slip_detected_pub.last_msg
        self.assertTrue(slip_msg.data)

    def test_no_slip_below_min_speed(self):
        ctrl = self._make_controller()
        ctrl._v_encoder = 0.001  # below _MIN_SPEED_MS
        ctrl._v_imu = 0.0
        ctrl._evaluate_slip()
        slip_msg = ctrl._slip_detected_pub.last_msg
        # At near-zero speed with near-zero IMU, slip_ratio stays 0
        self.assertFalse(slip_msg.data)


# ---------------------------------------------------------------------------
# PaintingController tests
# ---------------------------------------------------------------------------

class TestPaintingController(unittest.TestCase):

    def setUp(self):
        self.paint_mod = _load('painting_controller')

    def _make_controller(self):
        ctrl = self.paint_mod.PaintingController.__new__(
            self.paint_mod.PaintingController
        )
        ctrl._params = {
            'nominal_speed': 0.10,
            'coverage_gain': 1.0,
            'min_duty': 0.05,
            'speed_deadband': 0.005,
        }
        ctrl._nominal_speed = 0.10
        ctrl._coverage_gain = 1.0
        ctrl._min_duty = 0.05
        ctrl._speed_deadband = 0.005
        ctrl._painting_active = True
        ctrl._current_speed = 0.0

        class _FakePub:
            def __init__(self): self.last_msg = None
            def publish(self, msg): self.last_msg = msg

        ctrl._spray_duty_pub = _FakePub()

        class _L:
            def info(self, m): pass
        ctrl.get_logger = lambda: _L()

        return ctrl

    def test_duty_zero_when_painting_off(self):
        ctrl = self._make_controller()
        ctrl._painting_active = False
        ctrl._control_loop()
        # Should return immediately without publishing
        self.assertIsNone(ctrl._spray_duty_pub.last_msg)

    def test_duty_zero_when_stopped(self):
        ctrl = self._make_controller()
        ctrl._current_speed = 0.0
        ctrl._control_loop()
        self.assertAlmostEqual(ctrl._spray_duty_pub.last_msg.data, 0.0)

    def test_duty_at_nominal_speed(self):
        ctrl = self._make_controller()
        ctrl._current_speed = 0.10  # nominal speed
        ctrl._control_loop()
        self.assertAlmostEqual(ctrl._spray_duty_pub.last_msg.data, 1.0)

    def test_duty_clamped_to_one(self):
        ctrl = self._make_controller()
        ctrl._current_speed = 0.50  # much faster than nominal
        ctrl._control_loop()
        self.assertLessEqual(ctrl._spray_duty_pub.last_msg.data, 1.0)

    def test_duty_at_min_when_slow(self):
        ctrl = self._make_controller()
        ctrl._current_speed = 0.005  # just above deadband
        ctrl._control_loop()
        self.assertGreaterEqual(ctrl._spray_duty_pub.last_msg.data, ctrl._min_duty)


# ---------------------------------------------------------------------------
# SafetyMonitor tests
# ---------------------------------------------------------------------------

class TestSafetyMonitor(unittest.TestCase):

    def setUp(self):
        self.safety_mod = _load('safety_monitor')

    def _make_monitor(self):
        mon = self.safety_mod.SafetyMonitor.__new__(
            self.safety_mod.SafetyMonitor
        )
        mon._params = {
            'rth_voltage': 14.4,
            'critical_voltage': 13.2,
            'slip_fault_timeout': 3.0,
            'nominal_voltage': 16.8,
        }
        mon._rth_voltage = 14.4
        mon._critical_voltage = 13.2
        mon._slip_timeout = 3.0
        mon._nominal_voltage = 16.8
        mon._battery_voltage = 16.8
        mon._rth_triggered = False
        mon._estop_triggered = False
        mon._painting_on = True
        mon._slip_duration = 0.0
        mon._slip_timer_dt = 0.2

        class _FakePub:
            def __init__(self): self.last_msg = None
            def publish(self, msg): self.last_msg = msg

        mon._rth_pub = _FakePub()
        mon._painting_pub = _FakePub()
        mon._estop_pub = _FakePub()

        class _L:
            def info(self, m): pass
            def warn(self, m): pass
            def warning(self, m): pass
            def error(self, m): pass
        mon.get_logger = lambda: _L()

        return mon

    def test_no_action_on_full_battery(self):
        mon = self._make_monitor()
        mon._monitor_loop()
        self.assertFalse(mon._rth_triggered)
        self.assertFalse(mon._estop_triggered)

    def test_rth_triggered_on_low_battery(self):
        mon = self._make_monitor()
        mon._battery_voltage = 14.0  # below rth_voltage
        mon._monitor_loop()
        self.assertTrue(mon._rth_triggered)
        self.assertFalse(mon._estop_triggered)

    def test_estop_triggered_on_critical_battery(self):
        mon = self._make_monitor()
        mon._battery_voltage = 13.0  # below critical_voltage
        mon._monitor_loop()
        self.assertTrue(mon._estop_triggered)

    def test_estop_triggered_on_persistent_slip(self):
        mon = self._make_monitor()
        mon._slip_duration = 4.0  # exceeds 3.0 s timeout
        mon._monitor_loop()
        self.assertTrue(mon._estop_triggered)

    def test_rth_not_retriggered(self):
        mon = self._make_monitor()
        mon._battery_voltage = 14.0
        mon._monitor_loop()
        self.assertTrue(mon._rth_triggered)
        # Second call should not re-publish
        mon._rth_pub.last_msg = None
        mon._monitor_loop()
        self.assertIsNone(mon._rth_pub.last_msg)


if __name__ == '__main__':
    unittest.main()
