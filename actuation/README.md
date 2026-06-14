# Actuation

Controls all eight CubeMars AK40-10 brushless motors via CAN bus using the MIT (Mini Cheetah) protocol.

---

## Package Layout

```
actuation/
├── actuation_bringup/          # Launch files
│   └── launch/
│       ├── can.launch.py       # Bring up a single CAN interface
│       └── cubemars.launch.py  # Launch one motor node
├── cubemars_v2_ros/            # Motor driver
│   └── cubemars_v2_ros/
│       └── motor_node.py
├── can_bus/                    # Low-level SocketCAN helpers
├── actuation_msgs/             # Custom ROS2 message definitions
└── motor_interfaces/           # Shared motor state message
```

---

## MIT Control Protocol

Each motor receives a five-tuple command every control cycle:

```
τ = Kp × (p_des − p_meas) + Kd × (v_des − v_meas) + τ_ff
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| `p_des` | ±12.5 rad | Desired position |
| `v_des` | ±45.5 rad/s (AK40-10) | Desired velocity |
| `Kp` | 0 – 500 | Position gain |
| `Kd` | 0 – 5 | Velocity (damping) gain |
| `τ_ff` | ±T_MAX | Feed-forward torque |

Unlike ODrive, the CubeMars motor applies this formula **onboard** with no filtering or ramping. Commands are sent as raw CAN frames at 100 Hz.

---

## Supported Motor Types

| Model | V_MAX (rad/s) | T_MAX (Nm) |
|-------|--------------|------------|
| AK10-9 | ±50 | ±65 |
| AK40-10 | ±45.5 | ±5 |
| AK60-6 | ±45 | ±15 |
| AK70-10 | ±50 | ±25 |
| AK80-6 | ±76 | ±12 |
| AK80-8 | ±37.5 | ±32 |
| AK80-9 | ±50 | ±18 |
| AK80-64 | ±9.2 | ±144 |

SELQIE Lite 2 uses **AK40-10** motors exclusively.

---

## ROS2 Interface

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `/{joint}/motor_state` | `MotorState` | Full feedback at 100 Hz |
| `/{joint}/estimate` | `MotorEstimate` | Position/velocity/torque for kinematics |
| `/{joint}/error_code` | `Int32` | Error bitfield with log message |

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `/{joint}/command` | `MotorCommand` | High-level command (position/velocity/torque mode) |
| `/{joint}/mit_cmd` | `Float64MultiArray` | Raw MIT 5-tuple `[p, v, kp, kd, τ]` |
| `/{joint}/special_cmd` | `String` | `"start"`, `"exit"`, `"zero"`, `"clear"` |

### Message Definitions

**MotorCommand**
```
uint32 control_mode    # 1=TORQUE  2=VELOCITY  3=POSITION
uint32 input_mode      # (ignored by CubeMars — always direct passthrough)
float32 pos_setpoint   # rad
float32 vel_setpoint   # rad/s
float32 torq_setpoint  # Nm
```

**MotorState**
```
string  name
float32 position       # rad (wrapped ±π)
float32 abs_position   # rad (unwrapped, tracks multiple revolutions)
float32 velocity       # rad/s
float32 torque         # Nm
float32 current        # A
int32   temperature    # °C
```

**MotorEstimate**
```
float32 pos_estimate   # rad
float32 vel_estimate   # rad/s
float32 torq_estimate  # Nm
```

---

## Motor Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `motor_id` | `0` | CAN node ID (0–7) |
| `motor_type` | `AK40-10` | Motor model string |
| `interface` | `can0` | SocketCAN interface name |
| `position_kp` | `1.1` | Kp for POSITION control mode |
| `position_kd` | `0.12` | Kd for POSITION control mode |
| `velocity_kd` | `0.5` | Kd for VELOCITY control mode |
| `reverse_polarity` | `false` | Negate position/velocity/torque |
| `cmd_timeout` | `0.5` | Seconds before a stale command is zeroed (0 = disabled) |
| `auto_start` | `false` | Enable motor on node startup |

---

## Motor Mapping and Gain Tuning

Gains are set per shaft group in `selqie_bringup/launch/actuation.launch.py`:

```python
INNER_KP     = '3.0'   # motors 0, 2, 4, 6 (reversed polarity)
INNER_KD     = '0.3'
INNER_VEL_KD = '0.5'

OUTER_KP     = '3.0'   # motors 1, 3, 5, 7
OUTER_KD     = '0.3'
OUTER_VEL_KD = '0.5'
```

Edit those constants to retune both groups without touching individual motor launch arguments.

### Why the gains are low

The AK40-10 in MIT mode applies `Kp` directly — there is no position filter or velocity ramp. With the original ODrive gains (Kp ≈ 20), a 0.25 rad position error already saturates the torque limit. Start conservatively:

| Gain | Starting point |
|------|---------------|
| Kp (position) | 1.0 – 3.0 |
| Kd (position) | 0.1 – 0.5 |
| Kd (velocity) | 0.3 – 0.8 |

---

## Command Timeout Safety

`cmd_timeout` (default 0.5 s) protects against a stalled gait node. If no new command arrives within that window the motor node:

1. Zeros the cached command `[0, 0, 0, 0, 0]`
2. Sets `_neutral_hold = true` (motor holds position passively)
3. Logs a warning

Set `cmd_timeout: 0.0` to disable (not recommended for hardware runs).

---

## CAN Bus

Two SocketCAN interfaces are used:

| Interface | Motors | Legs |
|-----------|--------|------|
| `can0` | 0, 1, 6, 7 | FL, FR |
| `can1` | 2, 3, 4, 5 | RL, RR |

The CAN interfaces are brought up by `actuation_bringup/launch/can.launch.py` using the `loadcan_jetson.sh` script, which writes pin-mux registers via `devmem` and calls `ip link set canN up type can bitrate 1000000`.

To verify CAN traffic:
```bash
candump can0
candump can1
```

---

## Special Commands

Send a string to `/{joint}/special_cmd`:

| Command | Effect |
|---------|--------|
| `start` | Enable motor (must be sent before motion commands) |
| `exit` | Disable motor (safe power-down) |
| `zero` | Set current position as encoder zero |
| `clear` | Clear fault register |

Example:
```bash
ros2 topic pub --once /motor0/special_cmd std_msgs/msg/String "data: 'start'"
```
