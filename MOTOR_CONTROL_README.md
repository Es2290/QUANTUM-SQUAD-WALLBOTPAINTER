# Motor Control Stack — Architecture & Usage

## Overview

The motor control stack bridges **ROS 2** on the Raspberry Pi 5 with **ESP32 firmware** for real-time motor control and odometry.

### System Architecture

```
┌──────────────────────────────────────┐
│   Raspberry Pi 5 (ROS 2 Humble)      │
├──────────────────────────────────────┤
│                                      │
│  ┌────────────────────────────────┐  │
│  │  navigation_node               │  │
│  │  (publishes /cmd_vel)          │  │
│  └────────────┬───────────────────┘  │
│               │                      │
│               │ Twist msg            │
│               ↓                      │
│  ┌────────────────────────────────┐  │
│  │  motor_controller              │  │
│  │  - Converts Twist → RPM        │  │
│  │  - PID closed-loop control     │  │
│  │  - Publishes /odom             │  │
│  └────────────┬───────────────────┘  │
│               │                      │
│               │ Float32MultiArray    │
│               ↓                      │
│  ┌────────────────────────────────┐  │
│  │  uart_bridge                   │  │
│  │  - TX: MOT <fl>,<fr>,...      │  │
│  │  - RX: ENC <t0>,<t1>,...      │  │
│  └────────────┬───────────────────┘  │
│               │                      │
│               │ UART (921600 baud)   │
│               │ /dev/ttyUSB0         │
└───────────────┼──────────────────────┘
                │
                │
┌───────────────┼──────────────────────┐
│       ESP32 (FreeRTOS)               │
├───────────────┼──────────────────────┤
│               ↓                      │
│  ┌────────────────────────────────┐  │
│  │  comms_task (priority 2)       │  │
│  │  - Parse UART messages         │  │
│  │  - Dispatch to motor/spray     │  │
│  └────────────┬───────────────────┘  │
│               │                      │
│       ┌───────┼────────┐             │
│       ↓       ↓        ↓             │
│  ┌─────────┐ ┌─────┐ ┌──────┐       │
│  │ motor   │ │pump │ │ESTOP │       │
│  │ task    │ │task │ │sig   │       │
│  │ (pri 4) │ │(pri │ │      │       │
│  │         │ │ 3)  │ │      │       │
│  └─────────┘ └─────┘ └──────┘       │
│       ↑                              │
│  ┌────┴────────────────────────┐    │
│  │  encoder_task (priority 5)  │    │
│  │  - Reads encoders @ 1 kHz   │    │
│  │  - Sends ENC frames via TX  │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

## Nodes

### 1. `uart_bridge` — UART ↔ ROS 2 Bridge

**Purpose**: Bi-directional communication with ESP32.

**Subscribes to**:
- `/motor_rpm_cmd` (Float32MultiArray) → Sends to ESP32 as `"MOT <fl>,<fr>,<rl>,<rr>\r\n"`

**Publishes**:
- `/encoder_ticks` (Int32MultiArray) ← Receives from ESP32 as `"ENC <t0>,<t1>,<t2>,<t3>\r\n"`

**Parameters** (in `config/uart_params.yaml`):
```yaml
uart_bridge:
  port: '/dev/ttyUSB0'          # Serial device
  baudrate: 921600              # Match ESP32 firmware
  timeout: 1.0                  # Read/write timeout (s)
```

**Usage**:
```bash
# Start standalone
ros2 run wallbot_painter uart_bridge

# Or via launch file
ros2 launch wallbot_painter motor_control.launch.py
```

---

### 2. `motor_controller` — Velocity → RPM + Odometry

**Purpose**: Convert ROS 2 velocity commands to motor RPM with closed-loop control and odometry.

**Subscribes to**:
- `/cmd_vel` (Twist) — Forward velocity + angular rate
- `/encoder_ticks` (Int32MultiArray) — Encoder feedback from ESP32

**Publishes**:
- `/motor_rpm_cmd` (Float32MultiArray) — Individual motor RPM commands
- `/odom` (Odometry) — Robot pose & velocity

**Services**:
- `/reset_odometry` (Empty) — Reset pose to (0, 0, 0)

**Parameters** (in `config/uart_params.yaml`):
```yaml
motor_controller:
  wheel_radius: 0.05            # metres
  wheel_base: 0.30              # track width
  max_rpm: 300.0
  encoder_ticks_per_rev: 4096
  
  # PID gains
  pid_kp: 0.05                  # Proportional
  pid_ki: 0.01                  # Integral
  pid_kd: 0.005                 # Derivative
  
  # Odometry covariance
  odom_pos_cov: 0.01            # m²
  odom_rot_cov: 0.05            # rad²
```

**Features**:
- ✅ Differential-drive kinematics
- ✅ Closed-loop PID speed control (corrects encoder feedback)
- ✅ Integrates odometry from encoders
- ✅ Full quaternion orientation
- ✅ Covariance matrices for uncertainty

**Usage**:
```bash
ros2 run wallbot_painter motor_controller

# Reset odometry
ros2 service call /reset_odometry std_srvs/srv/Empty
```

---

## Data Flow Example

### Sending motor commands:

```
1. Navigation node sends:
   /cmd_vel = Twist(linear.x=0.5 m/s, angular.z=0.2 rad/s)

2. motor_controller receives Twist
   - Computes: v_left = 0.5 - (0.2 * 0.30/2) = 0.47 m/s
   - Computes: v_right = 0.5 + (0.2 * 0.30/2) = 0.53 m/s
   - Converts to RPM: ~143 RPM (left), ~161 RPM (right)

3. motor_controller applies PID correction (if measured RPM differs)
   - Publishes /motor_rpm_cmd = [143, 161, 143, 161]

4. uart_bridge receives motor_rpm_cmd
   - Formats as: "MOT 143.0,161.0,143.0,161.0\r\n"
   - Sends via UART to ESP32

5. ESP32 firmware (main.c):
   - comms_task parses the MOT frame
   - motor_task sets individual ESC RPM values
```

### Receiving encoder feedback:

```
1. ESP32 encoder_task (runs @ 1 kHz)
   - Reads all 4 encoders
   - Sends: "ENC 1234,-567,890,-456\r\n" via UART

2. uart_bridge receives on RX thread
   - Parses the ENC frame
   - Publishes /encoder_ticks = Int32MultiArray([1234, -567, 890, -456])

3. motor_controller receives encoder_ticks
   - Computes actual RPM per motor
   - Updates PID error terms for next cycle
   - Integrates odometry: Δx, Δy, Δθ
   - Publishes /odom with updated pose
```

---

## Launch Configuration

### Full motor control stack:

```bash
ros2 launch wallbot_painter motor_control.launch.py
```

This starts:
- `uart_bridge` (ESP32 communication)
- `motor_controller` (velocity → RPM + odometry)

### Verify it's working:

```bash
# Monitor encoder feedback
ros2 topic echo /encoder_ticks

# Monitor motor commands
ros2 topic echo /motor_rpm_cmd

# Monitor odometry
ros2 topic echo /odom

# Send test velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist '{
  linear: {x: 0.1, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}
}'
```

---

## Troubleshooting

### No encoder data received
- Check `/dev/ttyUSB0` exists: `ls -la /dev/ttyUSB*`
- Verify baud rate matches ESP32 (921600)
- Check UART cable connection

### Odometry drifts
- Encoder calibration: Measure actual wheel circumference
- Adjust `wheel_radius` parameter
- Increase `odom_pos_cov` if uncertainty is high

### Motors don't respond
- Check UART tx/rx on oscilloscope
- Verify ESP32 firmware is running
- Test with direct UART command: `echo "MOT 100,100,100,100" > /dev/ttyUSB0`

### PID oscillation
- Reduce `pid_kp` (proportional gain)
- Increase `pid_kd` (derivative gain)

---

## Future Enhancements

- [ ] IMU integration (`imu_slip_controller`)
- [ ] Spray pump control (`painting_controller`)
- [ ] Safety monitoring (`safety_monitor`)
- [ ] Multi-threaded encoder processing
- [ ] Micro-ROS integration for lower latency
