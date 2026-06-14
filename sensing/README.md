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

**Hardware:** Digital leak detection board connected to **Pin 15 (SOC_GPIO27)**.

**Driver:** `leak_sensor/leak_sensor_node.py` polls the GPIO pin at 10 Hz and publishes `True` when a leak is detected. Logs a throttled warning every second while a leak is active.

### Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gpio_pin` | `35` | BOARD pin number |
| `frequency` | `10.0` | Poll rate in Hz |
| `active_high` | `true` | `true` = pin HIGH means leak present |

### Published Topic

| Topic | Type | Description |
|-------|------|-------------|
| `leak/detected` | `Bool` | `true` when water ingress detected |

### Emergency Response

`water_shutdown.py` (repo root) subscribes to `leak/detected` and sends `"exit"` to all motor special command topics if triggered. This gracefully disables all motors and shuts down actuators in the event of flooding.

---

## Reed Switch

**Hardware:** Magnetic reed switch connected to **Pin 16 (SOC_GPIO08)**.

Use cases: detecting hull panel closure (magnet on the lid), triggering autonomous behavior when the robot is deployed.

**Driver:** `reed_switch/reed_switch_node.py` polls at 50 Hz.

### Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gpio_pin` | `38` | BOARD pin number |
| `frequency` | `50.0` | Poll rate in Hz |
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
| `/battery/voltage` | `Float32` | Pack voltage in volts |

---

## Jetson AGX Orin Pin Assignments

| Pin | Signal | Use |
|-----|--------|-----|
| 35 | GPIO | Leak sensor input |
| 38 | GPIO | Reed switch input |
| 18 | PWM5 | Camera / lights PWM |
| 19 | SPI1_MOSI | WS2812B data signal |
| 21 | SPI1_MISO | (not connected) |
| 23 | SPI1_CLK | (not connected) |
| 24 | SPI1_CS0 | (not connected) |
| 29 | CAN0_DIN | Motor CAN bus (FL, FR) |
| 31 | CAN0_DOUT | Motor CAN bus (FL, FR) |
| 33 | CAN1_DIN | Motor CAN bus (RL, RR) |
| 37 | CAN1_DOUT | Motor CAN bus (RL, RR) |

These functions are configured automatically by `tools/install.sh` using:
```bash
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1 pwm5 spi1
```

---

## IMU

The `imu.launch.py` file launches the Microstrain IMU driver. It is currently commented out in `sensing.launch.py` — enable it once the IMU is connected and the udev rules (`tools/100-microstrain.rules`) are confirmed working.
