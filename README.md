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
├── planning/           # Gait and motion planning (implemented, not launched by default)
├── mapping/            # Terrain mapping (implemented, not launched by default)
├── localization/       # Position estimation (implemented, not launched by default)
├── mpc/                # Model predictive control (implemented, not launched by default)
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

> **`selqie_hw.launch.py` only starts `actuation.launch.py` and `leg_control.launch.py` by default.** Sensing (leak sensor, reed switch, battery, Bar100, LED, servo), vision (camera/lights), mapping, planning, localization, and TF are commented out and must be launched separately, e.g. `ros2 launch selqie_bringup sensing.launch.py`. See [`selqie_bringup/README.md`](selqie_bringup/README.md) and [`docs/SELQIE_LITE_2_SOP.md`](docs/SELQIE_LITE_2_SOP.md#5-bring-up--launch-procedure) for details.

---

## Hardware Overview

### Physical Specifications

| Dimension | Imperial | Metric |
|-----------|----------|--------|
| Length | 22 in | 0.5588 m |
| Width | 7.5 in | 0.1905 m |
| Height (body) | 3.5 in | 0.0889 m |

The IMU is mounted at the center of mass, which is the geometric center of the body. In the ROS2 TF tree this corresponds to `base_link` origin with the `imu_link` transform at `[0, 0, 0]`.

### Motors

| Motor ID | CAN Bus | Leg | Position | Polarity |
|----------|---------|-----|----------|----------|
| 0 | can0 | FL | Inner shaft | Reversed* |
| 1 | can0 | FL | Outer shaft | Normal |
| 2 | can1 | RL | Inner shaft | Reversed* |
| 3 | can1 | RL | Outer shaft | Normal |
| 4 | can1 | RR | Inner shaft | Reversed* |
| 5 | can1 | RR | Outer shaft | Normal |
| 6 | can0 | FR | Inner shaft | Reversed* |
| 7 | can0 | FR | Outer shaft | Normal |

\* This is the documented intent (`reverse_polarity` is described as "true for inner shafts" in `actuation_bringup/launch/cubemars.launch.py`), but `selqie_bringup/launch/actuation.launch.py`'s `InnerShaft()` helper does not currently pass `reverse_polarity='true'`, so all motors launch with `reverse_polarity=false` in practice. Verify before relying on this column.

Motor type: **CubeMars AK40-10** — MIT (Mini Cheetah) protocol, T_MAX = 5 Nm

### Jetson AGX Orin 40-Pin Header

Full header pinout (physical/BOARD numbering — this is what `Jetson.GPIO`'s `GPIO.setmode(GPIO.BOARD)` addresses, used throughout this codebase). The "Default Signal" column is the pin's stock silkscreen function; several pins are repurposed as plain digital GPIO in software (via `jetson_drivers/gpio_node`) regardless of that default label.

| Pin | Default Signal | Pin | Default Signal |
|----:|-----------------|----:|-----------------|
| 1 | 3.3V | 2 | 5.0V |
| 3 | I2C5_DAT | 4 | 5.0V |
| 5 | I2C5_CLK | 6 | GND |
| 7 | MCLK05 | 8 | UART1_TX |
| 9 | GND | 10 | UART1_RX |
| 11 | UART1_RTS | 12 | I2S2_CLK |
| 13 | GPIO32 | 14 | GND |
| 15 | GPIO27 | 16 | GPIO08 |
| 17 | 3.3V | 18 | GPIO35 |
| 19 | SPI1_MOSI | 20 | GND |
| 21 | SPI1_MISO | 22 | GPIO17 |
| 23 | SPI1_SCK | 24 | SPI1_CS0_N |
| 25 | GND | 26 | SPI1_CS1_N |
| 27 | I2C2_DAT | 28 | I2C2_CLK |
| 29 | CAN0_DIN | 30 | GND |
| 31 | CAN0_DOUT | 32 | GPIO09 |
| 33 | CAN1_DOUT | 34 | GND |
| 35 | I2S_FS | 36 | UART1_CTS |
| 37 | CAN1_DIN | 38 | I2S_SDIN |
| 39 | GND | 40 | I2S_SDOUT |

**Pins actually used by SELQIE Lite 2:**

| Pin(s) | Default Signal | Used for | Source |
|--------|-----------------|----------|--------|
| 29, 31 | CAN0_DIN/DOUT | Motor CAN bus `can0` (FL, FR motors) | `tools/loadcan_jetson.sh` |
| 33, 37 | CAN1_DOUT/DIN | Motor CAN bus `can1` (RL, RR motors) | `tools/loadcan_jetson.sh` |
| 18 | GPIO35 | Reconfigured as **PWM5** for the camera/underwater-lights driver | `vision.launch.py` (`LIGHT_PWM_PIN = 18`) |
| 19 | SPI1_MOSI | WS2812B status LED data (MISO/CLK/CS0 — pins 21/23/24 — not wired) | `sensing/ws2812b.launch.py` |
| 35 | I2S_FS | Repurposed as a digital GPIO input for the **leak sensor** | `sensing/leak_sensor.launch.py` (`gpio_pin: 35`) |
| 38 | I2S_SDIN | Repurposed as a digital GPIO input for the **reed switch** (hull-door detection, *not* a leak sensor) | `sensing/reed_switch.launch.py` (`gpio_pin: 38`) |
| 13 or 32 | GPIO32 / GPIO09 | Hull-latch servo PWM — **inconsistent in the code itself**: `servo_node`'s own declared default and inline comment say pin **32**, but `sensing_bringup/launch/servo.launch.py` overrides it to pin **13** at launch time, so **13 is what actually runs**. Verify against the physical wiring and fix the stale comment or the override. | `sensing/servo/servo/servo_node.py`, `sensing/sensing_bringup/launch/servo.launch.py` |

Pins 15 (GPIO27) and 16 (GPIO08) were the leak sensor's and reed switch's pin assignments in an earlier revision (still referenced in some historical docs); the current code uses pins 35 and 38 instead (see `git log` — "Update leak sensor to GPIO pin 35 and reed switch to GPIO pin 38").

These functions are configured automatically by `tools/install.sh` using:
```bash
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1 pwm5 spi1
```

---

## Key ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/motorN/command` | `MotorCommand` | Send position/velocity/torque setpoint |
| `/motorN/motor_state` | `MotorState` | Motor feedback (pos, vel, torque, temp) |
| `/motorN/estimate` | `MotorEstimate` | Kinematics-facing position/velocity feedback |
| `/motorN/special_cmd` | `String` | `"start"`, `"exit"`, `"zero"`, `"clear"` |
| `leg{FL,RL,RR,FR}/command` | `LegCommand` | Cartesian leg endpoint setpoint (no underscore, e.g. `legFL/command`) |
| `leg{FL,RL,RR,FR}/estimate` | `LegEstimate` | Cartesian leg endpoint feedback |
| `leak/detected` | `Bool` | True when water ingress detected (requires `sensing.launch.py`) |
| `reed_switch/closed` | `Bool` | True when magnet is present (requires `sensing.launch.py`) |
| `led_colors` | `UInt32MultiArray` | Packed 0xRRGGBB for each WS2812B pixel |
| `/tinybms/pack_voltage` | `Float32` | Battery voltage from TinyBMS (there is no separate `/battery/voltage` topic) |
| `/bar100/depth` | `Float64` | Depth in metres |

> **Note:** only `actuation.launch.py` and `leg_control.launch.py` run by default, so the motor/leg topics above are always available, but `leak/detected`, `reed_switch/closed`, `led_colors`, and `/tinybms/pack_voltage` require `sensing.launch.py` to be launched separately. See [Quick Start](#3-launch-the-robot).

---

## Subsystem Documentation

- [Actuation](actuation/README.md) — CAN bus, motor driver, MIT protocol, gain tuning
- [Leg Control](leg_control/README.md) — 5-bar kinematics, stride generation, gaits
- [Sensing](sensing/README.md) — BAR100, WS2812B, leak sensor, reed switch, battery
- [UI](ui/README.md) — `selqie.py` API reference, `selqie_terminal` command reference
- [Bringup](selqie_bringup/README.md) — Launch files and startup sequence
- [Tools](tools/README.md) — Installation, CAN setup, Jetson pin configuration
- [Standard Operating Procedure](docs/SELQIE_LITE_2_SOP.md) — Full operator's guide: safety, power-up/down, terminal command reference, troubleshooting, glossary. Start here if you are new to SELQIE.
