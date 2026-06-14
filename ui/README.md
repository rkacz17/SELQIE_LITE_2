# UI

Two packages provide the human interface to SELQIE Lite 2:

- **`selqie_python`** — `selqie.py`, a ROS2 node that wraps all robot subsystems in a single Python class.
- **`selqie_ui`** — `selqie_terminal.py`, an interactive `cmd`-based CLI that drives the `selqie.py` node.

---

## Package Layout

```
ui/
├── selqie_python/
│   └── selqie_python/
│       └── selqie.py           # Core SELQIE ROS2 node / API class
├── selqie_ui/
│   └── selqie_ui/
│       ├── selqie_terminal.py  # Interactive terminal
│       └── selqie_joint_publisher.py
├── selqie_tools/
│   └── selqie_tools/
│       └── interactive_rgb_viewer.py   # LED color picker utility
└── ui_bringup/
    └── launch/
        ├── urdf.launch.py      # Robot description publisher
        └── rviz.launch.py      # RViz2 visualization
```

---

## Running the Terminal

```bash
ros2 run selqie_ui selqie_terminal
```

The tmux session (`tmux/selqie.sh`) starts this automatically in the top-right pane.

---

## selqie_terminal Commands

Type `help` at the prompt to list all commands. Type `help <command>` for details.

### Motor Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `idle` | — | Send `"exit"` to all 8 motors (disable) |
| `ready` | — | Send `"start"` to all 8 motors (enable) |
| `zero` | — | Call `set_motor_position_zero` on all motors |
| `clear_errors` | — | Send `"clear"` to all motors |
| `set_motor_position` | `<motor_id> <pos_rad>` | Command one motor to a position |
| `set_gains` | `<kp> <kd>` | Update position gains on all motors |
| `default` | — | Restore default foot positions |

### Leg Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `set_leg_position` | `<leg\|*> <x> <y> <z>` | Move one or all legs to Cartesian position (m) |

### Sensor / Status Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `battery` | — | Print current battery voltage |

### LED Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `set_led_color` | `<r> <g> <b>` | Set WS2812B color (0–255 each channel) |
| `led_off` | — | Turn LED off (equivalent to `set_led_color 0 0 0`) |

### System

| Command | Arguments | Description |
|---------|-----------|-------------|
| `exit` | — | Quit the terminal |

---

## selqie.py API Reference

`selqie.py` is a `rclpy.node.Node` subclass. Import and spin it in your own Python scripts or Jupyter notebooks.

```python
import rclpy
from selqie_python.selqie import SELQIE

rclpy.init()
robot = SELQIE()
robot.init()         # Initialise all subsystems
```

### Initialisation Methods

| Method | Description |
|--------|-------------|
| `init()` | Call all `init_*` methods below |
| `init_motors()` | Publishers/subscribers for all 8 motors |
| `init_legs()` | Leg command/estimate topics for all 4 legs |
| `init_sensors()` | Subscribe to sensor topics |
| `init_led()` | LED color publisher |
| `init_localization()` | Localization service clients |
| `init_mapping()` | Mapping interfaces |
| `init_vision()` | Camera light control |
| `init_recording()` | Bag recording utilities |

### Motor Methods

```python
robot.start_motors()            # Send "start" to all motors
robot.stop_motors()             # Send "exit" to all motors
robot.zero_motors()             # Zero all encoders
robot.set_motor_position(n, pos_rad)
robot.set_motor_gains(kp, kd)  # Set gains on all motors
robot.get_motor_state(n)        # Returns MotorState for motor n
```

### Leg Methods

```python
robot.set_leg_position(leg_name, x, y, z)   # leg_name: "FL","RL","RR","FR"
robot.get_leg_estimate(leg_name)             # Returns LegEstimate
```

### LED Methods

```python
robot.set_led_color(r, g, b)   # r,g,b: int 0–255
robot.set_led_off()
```

### Sensor Methods

```python
voltage = robot.snapshot_battery_voltage()   # float, volts
```

### Localization Methods

```python
robot.send_localization_set_pose(pose)   # pose: PoseWithCovarianceStamped
```

### QoS Helpers

```python
from selqie_python.selqie import QOS_FAST, QOS_RELIABLE
```

`QOS_FAST` — best-effort, sensor data.
`QOS_RELIABLE` — reliable delivery, commands.

### Geometry Helpers

```python
from selqie_python.selqie import QUAT2EUL, EUL2QUAT
roll, pitch, yaw = QUAT2EUL(quaternion)
quaternion = EUL2QUAT(roll, pitch, yaw)
```

---

## Using selqie.py in a Script

```python
import rclpy
from selqie_python.selqie import SELQIE
import threading

rclpy.init()
robot = SELQIE()
robot.init()

# Spin in background thread
spin_thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
spin_thread.start()

# Command the robot
robot.start_motors()
robot.set_leg_position('FL', 0.0, 0.0, -0.15)
robot.set_led_color(0, 255, 0)   # Green = running

voltage = robot.snapshot_battery_voltage()
print(f"Battery: {voltage:.2f} V")

robot.stop_motors()
rclpy.shutdown()
```

---

## Interactive RGB Viewer

`selqie_tools/interactive_rgb_viewer.py` opens a simple GUI color picker that publishes to `led_colors` in real time — useful for visually calibrating LED colors.

```bash
ros2 run selqie_tools interactive_rgb_viewer
```
