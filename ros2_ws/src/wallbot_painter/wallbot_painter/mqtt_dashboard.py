"""MQTT telemetry gateway for the WallBot dashboard.

The gateway keeps ROS 2 as the source of truth and exposes a small, stable
MQTT API for remote dashboards:

  wallbot/telemetry       retained JSON snapshot, published at 2 Hz
  wallbot/command/estop  JSON ``{"enabled": true}``
  wallbot/command/painting JSON ``{"enabled": true}``

Commands are deliberately routed through the existing ROS safety topics
instead of directly controlling hardware.
"""

import json
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32MultiArray

try:
    import paho.mqtt.client as mqtt
except ImportError:  # Keep the package importable for ROS tooling without MQTT.
    mqtt = None


class MqttDashboard(Node):
    """Bridge robot health topics to MQTT and dashboard commands to ROS."""

    def __init__(self) -> None:
        super().__init__('mqtt_dashboard')
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('robot_id', 'wallbot')
        self.declare_parameter('publish_hz', 2.0)

        self._robot_id = str(self.get_parameter('robot_id').value)
        self._root_topic = f'{self._robot_id}/'
        self._state: Dict[str, Any] = {
            'online': True,
            'battery_voltage': None,
            'battery_percent': None,
            'slip_detected': False,
            'estop': False,
            'painting_active': False,
            'encoder_ticks': [0, 0, 0, 0],
            'freertos_free_heap': None,
            'freertos_uptime_s': None,
            'velocity': {'linear_x': 0.0, 'angular_z': 0.0},
            'updated_at': 0.0,
        }
        self._lock = threading.Lock()

        self._estop_pub = self.create_publisher(Bool, 'estop', 10)
        self._painting_pub = self.create_publisher(Bool, 'painting_active', 10)
        self.create_subscription(Float32, 'battery_voltage', self._battery_cb, 10)
        self.create_subscription(Bool, 'slip_detected', self._slip_cb, 10)
        self.create_subscription(Bool, 'estop', self._estop_cb, 10)
        self.create_subscription(Bool, 'painting_active', self._painting_cb, 10)
        self.create_subscription(Int32MultiArray, 'encoder_ticks', self._encoder_cb, 10)
        self.create_subscription(
            Float32MultiArray, 'freertos_status', self._freertos_cb, 10
        )
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

        if mqtt is None:
            raise RuntimeError(
                'paho-mqtt is required for mqtt_dashboard. '
                'Install it with: python3 -m pip install paho-mqtt'
            )

        self._mqtt = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'{self._robot_id}-ros2-gateway',
        )
        username = str(self.get_parameter('mqtt_username').value)
        if username:
            self._mqtt.username_pw_set(
                username, str(self.get_parameter('mqtt_password').value)
            )
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        host = str(self.get_parameter('mqtt_host').value)
        port = int(self.get_parameter('mqtt_port').value)
        self._mqtt.connect(host, port, keepalive=30)
        self._mqtt.loop_start()
        period = 1.0 / max(float(self.get_parameter('publish_hz').value), 0.1)
        self._timer = self.create_timer(period, self._publish_telemetry)
        self.get_logger().info(f'MQTT dashboard gateway connecting to {host}:{port}')

    def _on_connect(self, client: Any, userdata: Any, flags: Any, reason_code: Any, properties: Any = None) -> None:
        if reason_code != 0:
            self.get_logger().error(f'MQTT connection failed: {reason_code}')
            return
        client.subscribe(f'{self._root_topic}command/estop')
        client.subscribe(f'{self._root_topic}command/painting')
        self.get_logger().info('MQTT dashboard gateway connected.')

    def _on_message(self, client: Any, userdata: Any, msg: Any) -> None:
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            enabled = payload['enabled']
            if not isinstance(enabled, bool):
                raise ValueError('enabled must be a boolean')
        except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'Ignoring invalid MQTT command: {exc}')
            return

        ros_msg = Bool()
        ros_msg.data = enabled
        if msg.topic == f'{self._root_topic}command/estop':
            self._estop_pub.publish(ros_msg)
        elif msg.topic == f'{self._root_topic}command/painting':
            self._painting_pub.publish(ros_msg)

    def _update(self, **values: Any) -> None:
        with self._lock:
            self._state.update(values)

    def _battery_cb(self, msg: Float32) -> None:
        voltage = float(msg.data)
        percent = max(0.0, min(100.0, (voltage - 13.2) / 3.6 * 100.0))
        self._update(battery_voltage=voltage, battery_percent=percent)

    def _slip_cb(self, msg: Bool) -> None:
        self._update(slip_detected=bool(msg.data))

    def _estop_cb(self, msg: Bool) -> None:
        self._update(estop=bool(msg.data))

    def _painting_cb(self, msg: Bool) -> None:
        self._update(painting_active=bool(msg.data))

    def _encoder_cb(self, msg: Int32MultiArray) -> None:
        if len(msg.data) == 4:
            self._update(encoder_ticks=[int(value) for value in msg.data])

    def _freertos_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) == 2:
            self._update(
                freertos_free_heap=int(msg.data[0]),
                freertos_uptime_s=int(msg.data[1]),
            )

    def _odom_cb(self, msg: Odometry) -> None:
        self._update(velocity={
            'linear_x': float(msg.twist.twist.linear.x),
            'angular_z': float(msg.twist.twist.angular.z),
        })

    def _publish_telemetry(self) -> None:
        with self._lock:
            snapshot = dict(self._state)
        snapshot['updated_at'] = time.time()
        self._mqtt.publish(
            f'{self._root_topic}telemetry',
            json.dumps(snapshot, separators=(',', ':')),
            qos=1,
            retain=True,
        )

    def destroy_node(self) -> bool:
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MqttDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
