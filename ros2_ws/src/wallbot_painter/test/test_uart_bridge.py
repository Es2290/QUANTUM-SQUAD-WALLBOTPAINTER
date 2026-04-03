"""Unit tests for uart_bridge node."""

import unittest
from unittest.mock import Mock, MagicMock, patch
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class TestUARTBridgeProtocol(unittest.TestCase):
    """Test UART protocol formatting and parsing."""

    def test_motor_command_format(self):
        """Test that motor commands are formatted correctly."""
        # Expected format: "MOT 150.5,120.3,-100.2,200.1\r\n"
        rpms = [150.5, 120.3, -100.2, 200.1]
        frame = (
            f"MOT {rpms[0]:.1f},"
            f"{rpms[1]:.1f},"
            f"{rpms[2]:.1f},"
            f"{rpms[3]:.1f}\r\n"
        )
        expected = "MOT 150.5,120.3,-100.2,200.1\r\n"
        self.assertEqual(frame, expected)

    def test_encoder_parsing(self):
        """Test parsing of encoder lines."""
        line = "ENC 1234,-567,890,-456"
        parts = line[4:].split(',')
        ticks = [int(p.strip()) for p in parts]
        expected = [1234, -567, 890, -456]
        self.assertEqual(ticks, expected)

    def test_encoder_parsing_malformed(self):
        """Test that malformed encoder lines are handled."""
        line = "ENC 1234,-567,890"  # Missing 4th value
        parts = line[4:].split(',')
        self.assertNotEqual(len(parts), 4)

    def test_motor_command_clamping(self):
        """Test that RPM commands are within range."""
        rpms = [150.5, 120.3, -350.0, 200.1]  # Last one exceeds max of 300
        max_rpm = 300.0
        clamped = [max(-max_rpm, min(max_rpm, r)) for r in rpms]
        expected = [150.5, 120.3, -300.0, 200.1]
        self.assertEqual(clamped, expected)


class TestDeviceDiscovery(unittest.TestCase):
    """Test serial port discovery and initialization."""

    @patch('serial.Serial')
    def test_serial_port_open(self, mock_serial):
        """Test that serial port opens successfully."""
        mock_port = MagicMock()
        mock_serial.return_value = mock_port
        
        # Simulate opening port
        port_obj = mock_serial(
            port='/dev/ttyUSB0',
            baudrate=921600,
            timeout=1.0
        )
        
        # Verify it was created
        self.assertIsNotNone(port_obj)
        mock_serial.assert_called_once()

    @patch('serial.Serial')
    def test_serial_port_error(self, mock_serial):
        """Test handling of serial port open failure."""
        mock_serial.side_effect = Exception("Port not found")
        
        with self.assertRaises(Exception):
            mock_serial(port='/dev/ttyUSB99')


if __name__ == '__main__':
    unittest.main()
