# Bringup

Top-level launch files that assemble all subsystems into a running robot.

---

## Launch Hierarchy

```
selqie_hw.launch.py           ← entry point
├── actuation.launch.py       ← 8 motors + CAN interfaces
├── sensing.launch.py         ← all sensors
├── vision.launch.py          ← camera lights (stereo camera disabled)
└── leg_control.launch.py     ← kinematics + stride generation
```

Commented-out entries in `selqie_hw.launch.py` (not started by default):
- `mapping.launch.py`
- `planning.launch.py`
- `tf.launch.py`
- `localization.launch.py`
- `marker_localization.launch.py`

---

## Recommended Startup: tmux Session

```bash
bash ~/selqie_ws/src/SELQIE_LITE_2/tmux/selqie.sh
```

Opens four panes simultaneously:

| Pane | Command |
|------|---------|
| Top-left | `ros2 launch selqie_bringup selqie_hw.launch.py` |
| Top-right | `ros2 run selqie_ui selqie_terminal` |
| Bottom-left | Sourced shell |
| Bottom-right | `jtop` |

The script checks for `python3-can` before launching to catch missing dependencies early.

---

## Manual Launch

```bash
source ~/selqie_ws/install/setup.bash
ros2 launch selqie_bringup selqie_hw.launch.py
```

---

## Launch File Reference

### `selqie_hw.launch.py`

Main hardware stack. Sets `use_sim_time: false` for all children.

### `actuation.launch.py`

Starts two CAN interfaces then eight motor nodes.

**Gain constants** (edit at the top of the file to retune):

```python
INNER_KP     = '3.0'   # motors 0, 2, 4, 6 — inner shafts (reversed)
INNER_KD     = '0.3'
INNER_VEL_KD = '0.5'

OUTER_KP     = '3.0'   # motors 1, 3, 5, 7 — outer shafts
OUTER_KD     = '0.3'
OUTER_VEL_KD = '0.5'
```

### `sensing.launch.py`

Starts:
- `bar100.launch.py` — depth sensor
- `ws2812b.launch.py` — RGB LED
- `leak_sensor.launch.py` — water-ingress GPIO
- `reed_switch.launch.py` — magnetic reed switch GPIO
- `tinybms_voltage_uart.launch.py` — battery voltage

IMU is commented out — enable when hardware is connected.

### `leg_control.launch.py`

Starts four fivebar kinematics nodes (one per leg), four trajectory publishers, and one stride generation node (which contains all gait sub-nodes).

Accepts `use_sim_time` argument — `false` for hardware, `true` for MuJoCo simulation.

### `selqie_sim.launch.py`

Simulation entry point. Mirrors `selqie_hw.launch.py` but sets `use_sim_time: true` and launches MuJoCo instead of actuation.

---

## Motor Startup Sequence

After launch, motors start in a disabled state. Use `selqie_terminal` to enable them:

```
ready       ← enables all motors ("start" command)
default     ← moves legs to default stance position
```

To shut down safely:
```
idle        ← disables all motors ("exit" command)
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motors don't respond | CAN interface not up | Run `ip link show can0` — if DOWN, check `tools/load_can.service` |
| Motor runs away after start | Missing `MotorEstimate` feedback to kinematics | Verify `motor_node.py` publishes `/{joint}/estimate` |
| Stale motion after gait stops | `cmd_timeout` too long | Lower `cmd_timeout` in `cubemars.launch.py` (default 0.5 s) |
| LED never lights | SPI not enabled | Re-run `install.sh` — it calls `config-by-function.py` with `spi1` |
| Leak sensor always reads True | Wrong `active_high` | Set `active_high: false` in `leak_sensor.launch.py` |
| `python-can` import error | Package not installed | `sudo apt install python3-can` |
