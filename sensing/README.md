# Sensing

All environmental sensor nodes for SELQIE Lite 2: depth (BAR100), RGB LED feedback (WS2812B), water-ingress detection (leak sensor), hull door detection (reed switch), and battery monitoring (TinyBMS).

---

## Package Layout

```
sensing/
├── sensing_bringup/            # Launch files for all sensors
│   └── launch/
│       ├── bar100.launch.py
│       ├── ws2812b.launch.py
│       ├── leak_sensor.launch.py
│       ├── reed_switch.launch.py
│       └── imu.launch.py       # (disabled — requires hardware)
├── bar100_driver/              # Depth sensor (I2C)
├── ws2812b_ros/                # RGB LED driver (SPI)
├── leak_sensor/                # Water-ingress GPIO sensor
├── reed_switch/                # Magnetic reed switch GPIO sensor
└── imu_calibration/            # IMU calibration utilities
```

All sensors are started together by `selqie_bringup/launch/sensing.launch.py`.

---

## BAR100 Depth / Pressure Sensor

**Hardware:** Blue Robotics Bar100 connected via I2C (bus 7 on Jetson AGX Orin).

**Driver:** `bar100_driver/bar100_node.py` — reads pressure and temperature, then `depth2pose_node.py` converts depth to a `geometry_msgs/PoseWithCovarianceStamped` for the localization stack.

**Published topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/bar100/depth` | `Float64` | Depth in metres |
| `/bar100/pressure` | `Float64` | Absolute pressure in mbar |
| `/bar100/temperature` | `Float32` | Water temperature °C |

---

## WS2812B RGB LED

**Hardware:** One WS2812B addressable LED driven via SPI1 (Pin 19 — MOSI only wired).

**Driver:** `ws2812b_ros/led_node.py` encodes packed RGB values into the WS2812B bit-timing waveform and sends it over `/dev/spidev0.0`.

### Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_leds` | `1` | Number of addressable pixels |
| `brightness` | `1.0` | Global brightness scale (0.0 – 1.0) |
| `spi_bus` | `0` | SPI bus number |
| `spi_dev` | `0` | SPI device number |
| `spi_hz` | `2400000` | SPI clock frequency (2.4 MHz for WS2812B timing) |
| `pixel_order` | `GRB` | Color byte order (WS2812B uses GRB) |
| `startup_color` | `[0, 255, 0]` | RGB color shown immediately on node startup (before any `led_colors` message), as visual confirmation that the node and SPI wiring are working. Set to `[0, 0, 0]` to disable. |

### Subscribed Topic

| Topic | Type | Description |
|-------|------|-------------|
| `led_colors` | `UInt32MultiArray` | Packed 24-bit colors: `0x00RRGGBB` per pixel |

### Setting the LED Color

From the `selqie_terminal`:
```
set_led_color 255 0 0    # Red
set_led_color 0 255 0    # Green
led_off                  # Turn off
```

From Python (`selqie.py`):
```python
selqie.set_led_color(255, 128, 0)   # Orange
selqie.set_led_off()
```

From the command line:
```bash
ros2 topic pub --once led_colors std_msgs/msg/UInt32MultiArray \
  "data: [16711680]"   # 0xFF0000 = red
```

### SPI Hardware Setup

The WS2812B signal wire connects to **Pin 19 (SPI1_MOSI)**. The SPI1 interface must be enabled via Jetson-IO before first use — this is handled automatically by `tools/install.sh`.

Required permissions:
```bash
sudo groupadd -f spi
sudo usermod -a -G spi $USER
# Udev rule: SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
```

---

## Leak Sensor

**Hardware:** Digital leak detection board connected to **Pin 35 (BOARD numbering)**.

**Driver:** `leak_sensor.launch.py` starts a `jetson_drivers/gpio_node` configured as an input on pin 35, polling at 10 Hz and publishing the raw pin state as `Float32` on `leak/gpio_in`. `leak_sensor/leak_sensor_node.py` subscribes to `leak/gpio_in`, converts it to a boolean per `active_high`, and republishes it on `leak/detected`. Logs a throttled warning every second while a leak is active.

### `gpio_node` Parameters (set in `leak_sensor.launch.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gpio_pin` | `35` | BOARD pin number |
| `is_output` | `false` | Configures the pin as an input |
| `frequency` | `10.0` | Poll rate in Hz |

### `leak_sensor_node` Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `active_high` | `true` | `true` = pin HIGH means leak present |

### Published Topic

| Topic | Type | Description |
|-------|------|-------------|
| `leak/detected` | `Bool` | `true` when water ingress detected |

### Emergency Response

`water_shutdown.py` (repo root) is a **standalone script, independent of ROS and of this node.** It does not subscribe to `leak/detected` or send anything to the motor special-command topics. It polls three raw GPIO pins directly with `Jetson.GPIO` (pins 35, 22, and 38 — a holdover from the predecessor robot's 3-sensor layout) and runs `sudo shutdown -h now` on the whole Orin if any of them reads HIGH, logging to `/var/log/water_shutdown.log`. It is **not** installed as a running service by `tools/install.sh`; it only runs if someone has manually set it up as a `systemd` service per the instructions in its own file header.

> ⚠️ On this platform, GPIO pin 38 is wired to the reed switch (see below), not a leak sensor, and pin 22 has no sensor documented anywhere else in this repository. If `water_shutdown.py` is running as a service, it will currently misinterpret reed-switch activity on pin 38 as a leak. This script should be reconciled with the current leak-sensor/reed-switch pin assignment (35 = leak, 38 = reed switch) before being relied on for automatic leak protection — see `docs/SELQIE_LITE_2_SOP.md`, Appendix B.

---

## Reed Switch

**Hardware:** Magnetic reed switch connected to **Pin 38 (BOARD numbering)**.

Use cases: detecting hull panel closure (magnet on the lid), triggering autonomous behavior when the robot is deployed.

**Driver:** `reed_switch.launch.py` starts a `jetson_drivers/gpio_node` configured as an input on pin 38, polling at 50 Hz and publishing the raw pin state as `Float32` on `reed_switch/gpio_in`. `reed_switch/reed_switch_node.py` subscribes to `reed_switch/gpio_in`, converts it to a boolean per `active_high`, and republishes it on `reed_switch/closed`.

### `gpio_node` Parameters (set in `reed_switch.launch.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gpio_pin` | `38` | BOARD pin number |
| `is_output` | `false` | Configures the pin as an input |
| `frequency` | `50.0` | Poll rate in Hz |

### `reed_switch_node` Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `active_high` | `true` | `true` = pin HIGH means switch closed |

### Published Topic

| Topic | Type | Description |
|-------|------|-------------|
| `reed_switch/closed` | `Bool` | `true` when magnet is present (switch closed) |

---

## Battery Monitor (TinyBMS)

**Hardware:** TinyBMS battery management system connected via UART serial.

**Package:** `battery/` — `tinybms_voltage_uart.launch.py`

### Published Topic

| Topic | Type | Description |
|-------|------|-------------|
| `/tinybms/pack_voltage` | `Float32` | Pack voltage in volts (there is no separate `/battery/voltage` topic) |

---

## Jetson AGX Orin Pin Assignments

Physical/BOARD numbering, the numbering `Jetson.GPIO`'s `GPIO.setmode(GPIO.BOARD)` uses. See the [main README](../README.md#jetson-agx-orin-40-pin-header) for the complete 40-pin chart with default (silkscreen) signal names; this table only lists the pins sensing hardware uses.

| Pin | Use |
|-----|-----|
| 35 | Leak sensor input (default silkscreen label `I2S_FS`, repurposed as GPIO) |
| 38 | Reed switch input (default silkscreen label `I2S_SDIN`, repurposed as GPIO) — hull-door detection, **not** a leak sensor |
| 13 *or* 32 | Hull-latch servo PWM — `servo_node`'s own default/comment say pin 32, but `sensing_bringup/launch/servo.launch.py` overrides it to pin 13, so **13 is what actually runs**. See `docs/SELQIE_LITE_2_SOP.md`, Appendix B. |
| 18 | Camera / underwater-lights PWM (reconfigured as PWM5) |
| 19 | WS2812B data signal (SPI1_MOSI) |
| 21, 23, 24 | SPI1 MISO/CLK/CS0 (not connected) |
| 29, 31 | Motor CAN bus `can0` (FL, FR) — not a sensing pin, listed for reference |
| 33, 37 | Motor CAN bus `can1` (RL, RR) — not a sensing pin, listed for reference |

These functions are configured automatically by `tools/install.sh` using:
```bash
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1 pwm5 spi1
```

---

## IMU

The `imu.launch.py` file launches the Microstrain IMU driver. It is currently commented out in `sensing.launch.py` — enable it once the IMU is connected and the udev rules (`tools/100-microstrain.rules`) are confirmed working.
