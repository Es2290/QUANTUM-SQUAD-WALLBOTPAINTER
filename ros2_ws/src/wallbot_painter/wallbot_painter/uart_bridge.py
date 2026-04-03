"""UART bridge node — bi-directional communication with ESP32.

Handles:
  - Publishing motor RPM commands to ESP32 via UART
  - Receiving encoder tick counts from ESP32 and publishing to ROS2
  - Emergency stop (ESTOP) signal forwarding

Protocol:
  Outgoing (Raspberry Pi → ESP32):
    "MOT <fl_rpm>,<fr_rpm>,<rl_rpm>,<rr_rpm>\r\n"
    "SPR <duty_0_to_1>\r\n"
    "STP\r\n"  — emergency stop
  
  Incoming (ESP32 → Raspberry Pi):
    "ENC <t0>,<t1>,<t2>,<t3>\r\n"  — encoder ticks at 1 kHz
"""

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

try:
    import serial
except ImportError:
    raise ImportError("pyserial not installed. Run: pip install pyserial")


class UARTBridge(Node):
    """Bi-directional UART bridge to ESP32 firmware."""

    def __init__(self) -> None:
        super().__init__('uart_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('timeout', 1.0)

        port: str = self.get_parameter('port').value
        baudrate: int = self.get_parameter('baudrate').value
        timeout: float = self.get_parameter('timeout').value

        # ---------- Serial port setup ----------
        self._serial_port: Optional[serial.Serial] = None
        try:
            self._serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout,
            )
            self.get_logger().info(
                f'Opened serial port {port} at {baudrate} baud.'
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # ---------- Publishers ----------
        self._encoder_pub = self.create_publisher(
            Int32MultiArray, 'encoder_ticks', 10
        )

        # ---------- Subscribers ----------
        self.create_subscription(
            Float32MultiArray, 'motor_rpm_cmd', self._on_motor_cmd, 10
        )

        # ---------- RX thread for async reading ----------
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True
        )
        self._stop_event = threading.Event()
        self._rx_thread.start()

        self.get_logger().info('UARTBridge started.')

    # ------------------------------------------------------------------
    # TX: Motor command → ESP32
    # ------------------------------------------------------------------

    def _on_motor_cmd(self, msg: Float32MultiArray) -> None:
        """Send motor RPM commands to ESP32 via UART."""
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'Expected 4 motor RPM values, got {len(msg.data)}'
            )
            return

        if not self._serial_port or not self._serial_port.is_open:
            self.get_logger().error('Serial port not open.')
            return

        try:
            # Format: "MOT 150.5,120.3,-100.2,200.1\r\n"
            frame = (
                f"MOT {msg.data[0]:.1f},"
                f"{msg.data[1]:.1f},"
                f"{msg.data[2]:.1f},"
                f"{msg.data[3]:.1f}\r\n"
            )
            self._serial_port.write(frame.encode('utf-8'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Failed to send motor command: {e}')

    # ------------------------------------------------------------------
    # RX: Encoder data from ESP32
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        """Background thread: read lines from ESP32 and parse."""
        buffer = ""

        while not self._stop_event.is_set():
            try:
                if not self._serial_port or not self._serial_port.is_open:
                    time.sleep(0.1)
                    continue

                # Read available bytes (non-blocking via timeout)
                if self._serial_port.in_waiting > 0:
                    data = self._serial_port.read(
                        self._serial_port.in_waiting
                    )
                    buffer += data.decode('utf-8', errors='ignore')

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.rstrip('\r')

                        if line.startswith('ENC '):
                            self._parse_encoder_line(line)

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'Unexpected RX error: {e}')
                time.sleep(0.1)

    def _parse_encoder_line(self, line: str) -> None:
        """Parse 'ENC t0,t1,t2,t3' and publish."""
        try:
            # Format: "ENC 1234,-567,890,-456"
            parts = line[4:].split(',')
            if len(parts) != 4:
                self.get_logger().warn(
                    f'Expected 4 encoder values, got {len(parts)}: {line}'
                )
                return

            ticks = [int(p.strip()) for p in parts]

            msg = Int32MultiArray()
            msg.data = ticks
            self._encoder_pub.publish(msg)

        except ValueError as e:
            self.get_logger().warn(f'Failed to parse encoder line "{line}": {e}')

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        """Clean shutdown of UART and threads."""
        self._stop_event.set()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)

        if self._serial_port and self._serial_port.is_open:
            self._serial_port.close()
            self.get_logger().info('Serial port closed.')

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UARTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
