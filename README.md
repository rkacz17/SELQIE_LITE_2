# SELQIE Lite 2

**S**ubmerged **E**xploration **L**egged **Q**uadruped **I**ntelligent **E**xplorer — second-generation lite platform.

SELQIE Lite 2 is a four-legged underwater robot built around a Jetson AGX Orin (JetPack 6.1) running ROS2 Humble. Each leg is a 5-bar (fivebar) planar linkage driven by two CubeMars AK40-10 motors via CAN bus. The software stack covers actuation, kinematics, gait generation, sensing, and a high-level Python API with an interactive terminal.

---

## Repository Structure

```
SELQIE_LITE_2/
├── actuation/          # Motor drivers and CAN bus interface
├── leg_control/        # Inverse kinematics and stride generation
├── sensing/            # Depth, LED, leak, reed switch, battery, IMU
├── ui/                 # selqie.py API and selqie_terminal CLI
├── selqie_bringup/     # Top-level launch files
├── battery/            # TinyBMS battery monitor
├── planning/           # Gait and motion planning (future)
├── mapping/            # Terrain mapping (future)
├── localization/       # Position estimation (future)
├── mpc/                # Model predictive control (future)
├── simulation/         # MuJoCo simulation
├── vision/             # Stereo camera processing
├── jetson/             # Jetson GPIO and PWM drivers
├── tools/              # install.sh and system configuration
├── tmux/               # tmux session launcher
└── docs/               # Additional documentation
```

---

## Quick Start

### 1. Install Dependencies

```bash
cd ~/selqie_ws/src/SELQIE_LITE_2/tools
./install.sh          # Jetson AGX Orin (JetPack 6.1)
./install.sh --devel  # Ubuntu 22.04 development machine
```

### 2. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/selqie_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the Robot

```bash
# All-in-one tmux session (recommended)
bash ~/selqie_ws/src/SELQIE_LITE_2/tmux/selqie.sh

# Or launch the hardware stack manually
ros2 launch selqie_bringup selqie_hw.launch.py
```

The tmux script opens four panes:

| Pane | Contents |
|------|----------|
| Top-left | `selqie_hw.launch.py` hardware stack |
| Top-right | `selqie_terminal` interactive CLI |
| Bottom-left | Sourced shell for ad-hoc ROS2 commands |
| Bottom-right | `jtop` system monitor |

---

## Hardware Overview

### Motors

| Motor ID | CAN Bus | Leg | Position | Polarity |
|----------|---------|-----|----------|----------|
| 0 | can0 | FL | Inner shaft | Reversed |
| 1 | can0 | FL | Outer shaft | Normal |
| 2 | can1 | RL | Inner shaft | Reversed |
| 3 | can1 | RL | Outer shaft | Normal |
| 4 | can1 | RR | Inner shaft | Reversed |
| 5 | can1 | RR | Outer shaft | Normal |
| 6 | can0 | FR | Inner shaft | Reversed |
| 7 | can0 | FR | Outer shaft | Normal |

Motor type: **CubeMars AK40-10** — MIT (Mini Cheetah) protocol, T_MAX = 65 Nm

### Jetson AGX Orin 40-Pin Header

| Pin(s) | Function | Device |
|--------|----------|--------|
| 29, 31 | CAN0 DIN/DOUT | Motor CAN bus (FL, FR motors) |
| 33, 37 | CAN1 DIN/DOUT | Motor CAN bus (RL, RR motors) |
| 18 | PWM5 | Camera / underwater lights |
| 19, 21, 23, 24 | SPI1 MOSI/MISO/CLK/CS0 | WS2812B LED (only MOSI wired) |
| 15 | GPIO (SOC_GPIO27) | Leak sensor input |
| 16 | GPIO (SOC_GPIO08) | Reed switch input |

---

## Key ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/motorN/command` | `MotorCommand` | Send position/velocity/torque setpoint |
| `/motorN/motor_state` | `MotorState` | Motor feedback (pos, vel, torque, temp) |
| `/motorN/estimate` | `MotorEstimate` | Kinematics-facing position/velocity feedback |
| `/motorN/special_cmd` | `String` | `"start"`, `"exit"`, `"zero"`, `"clear"` |
| `/leg_{FL,RL,RR,FR}/command` | `LegCommand` | Cartesian leg endpoint setpoint |
| `/leg_{FL,RL,RR,FR}/estimate` | `LegEstimate` | Cartesian leg endpoint feedback |
| `leak/detected` | `Bool` | True when water ingress detected |
| `reed_switch/closed` | `Bool` | True when magnet is present |
| `led_colors` | `UInt32MultiArray` | Packed 0xRRGGBB for each WS2812B pixel |
| `/battery/voltage` | `Float32` | Battery voltage from TinyBMS |
| `/bar100/depth` | `Float64` | Depth in metres |

---

## Subsystem Documentation

- [Actuation](actuation/README.md) — CAN bus, motor driver, MIT protocol, gain tuning
- [Leg Control](leg_control/README.md) — 5-bar kinematics, stride generation, gaits
- [Sensing](sensing/README.md) — BAR100, WS2812B, leak sensor, reed switch, battery
- [UI](ui/README.md) — `selqie.py` API reference, `selqie_terminal` command reference
- [Bringup](selqie_bringup/README.md) — Launch files and startup sequence
- [Tools](tools/README.md) — Installation, CAN setup, Jetson pin configuration
