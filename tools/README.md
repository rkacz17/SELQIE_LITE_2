# Tools

Installation automation and system-level configuration scripts for SELQIE Lite 2.

---

## Files

| File | Description |
|------|-------------|
| `install.sh` | Full dependency installation and system setup |
| `loadcan_jetson.sh` | Bring up CAN0 and CAN1 interfaces |
| `load_can.service` | systemd unit — runs `loadcan_jetson.sh` at boot |
| `99-gpio.rules` | udev rule — grants GPIO group access to Jetson GPIO |
| `100-microstrain.rules` | udev rule — grants access to MicroStrain IMU serial device |
| `update_stereo_calibration.sh` | Upload stereo camera calibration YAML to camera |
| `configure_odrive_mj5208.py` | Legacy ODrive configuration (not used with CubeMars) |

---

## install.sh

Run once on a fresh Jetson AGX Orin or development machine.

```bash
cd ~/selqie_ws/src/SELQIE_LITE_2/tools

# Jetson AGX Orin (JetPack 6.1)
./install.sh

# Ubuntu 22.04 development machine
./install.sh --devel
```

### What it does

#### 1. OpenCV fix (Jetson only)

JetPack ships a custom OpenCV build that conflicts with ROS2. The script purges it and installs `libopencv-dev=4.5.*`, then holds the package to prevent unwanted upgrades.

#### 2. System packages

```
curl wget gpg apt-transport-https gdb tmux
python3-pip python3-smbus python3-can
libsocketcan-dev can-utils libeigen3-dev
```

Development machines also install MuJoCo graphics dependencies: `libx11-dev`, `libglfw3`, `libglfw3-dev`.

#### 3. ROS2 Humble

Adds the ROS2 apt repository and installs `ros-humble-desktop` plus:
- `python3-colcon-common-extensions`
- `ros-humble-camera-info-manager`, `image-proc`, `stereo-image-proc`
- `ros-humble-robot-localization`
- `ros-humble-microstrain-inertial-driver`
- `ros-humble-grid-map`

#### 4. External C++ libraries

| Library | Path | Purpose |
|---------|------|---------|
| MuJoCo | `~/.MuJoCo` | Physics simulation (development only) |
| OSQP | `~/.OSQP` | Convex quadratic programming for MPC |
| SBMPO | `~/.SBMPO` | Sampling-based motion planning |
| KellerLD | `~/.KellerLD` | Python driver for Bar100 depth sensor |

Each is cloned from GitHub and built with CMake only if the directory does not already exist.

#### 5. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/selqie_ws
colcon build --symlink-install
```

#### 6. Shell setup

Adds to `~/.bashrc` (if not already present):
```bash
source /opt/ros/humble/setup.bash
source ~/selqie_ws/install/setup.bash
```

#### 7. Jetson-specific setup

Skipped with `--devel`. On Jetson:

**GPIO permissions**
```bash
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo cp 99-gpio.rules /etc/udev/rules.d/
```

**SPI permissions** (for WS2812B LED)
```bash
sudo groupadd -f spi
sudo usermod -a -G spi $USER
# Creates: /etc/udev/rules.d/99-spi.rules
# SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
```

**CAN boot service**
```bash
sudo cp load_can.service /etc/systemd/system/
sudo systemctl enable load_can.service
sudo systemctl start load_can.service
```

**IMU rules**
```bash
sudo cp 100-microstrain.rules /etc/udev/rules.d/
```

**Jetson-IO pin-mux overlay**

Configures the 40-pin header for the required peripherals via device tree overlay:

```bash
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1 pwm5 spi1
```

| Function | Pin(s) | Device |
|----------|--------|--------|
| `can0` | 29, 31 | CAN0 DIN/DOUT (motors FL, FR) |
| `can1` | 33, 37 | CAN1 DIN/DOUT (motors RL, RR) |
| `pwm5` | 18 | Camera / underwater lights PWM |
| `spi1` | 19, 21, 23, 24 | WS2812B LED (only MOSI/pin 19 wired) |

This step requires a **reboot** to take effect.

---

## loadcan_jetson.sh

Brings up the two CAN interfaces at 1 Mbit/s. Called by `load_can.service` at boot.

```bash
sudo ./loadcan_jetson.sh
```

The script writes pin-mux register values with `devmem` to configure the CAN controller outputs, then calls:
```bash
ip link set can0 up type can bitrate 1000000
ip link set can1 up type can bitrate 1000000
```

To verify:
```bash
ip link show can0
ip link show can1
candump can0   # Should show CAN frames when motors are on
```

---

## load_can.service

systemd unit that ensures CAN interfaces are active before ROS2 nodes try to use them.

```ini
[Unit]
Description=Load CAN interfaces for SELQIE
After=network.target

[Service]
ExecStart=/path/to/loadcan_jetson.sh
...

[Install]
WantedBy=multi-user.target
```

Check status:
```bash
systemctl status load_can.service
journalctl -u load_can.service -n 50
```

---

## Post-Install Checklist

After running `install.sh` and **rebooting**:

- [ ] `ls /dev/spidev0.0` — SPI device exists
- [ ] `ip link show can0` — CAN0 is UP
- [ ] `ip link show can1` — CAN1 is UP
- [ ] `groups` includes `gpio`, `spi`, `dialout`
- [ ] `source ~/.bashrc` — ROS2 and workspace are sourced
- [ ] `ros2 launch selqie_bringup selqie_hw.launch.py` — launches without errors
