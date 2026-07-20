# Bringup

Top-level launch files that assemble all subsystems into a running robot.

---

## Launch Hierarchy

```
selqie_hw.launch.py           ← entry point
├── actuation.launch.py       ← 8 motors + CAN interfaces         [ACTIVE by default]
├── sensing.launch.py         ← all sensors                       [commented out]
├── vision.launch.py          ← camera + underwater lights        [commented out]
└── leg_control.launch.py     ← kinematics + stride generation    [ACTIVE by default]
```

**Only `actuation.launch.py` and `leg_control.launch.py` are started by default.** Everything else is commented out in `selqie_hw.launch.py` and must be launched separately if you need it:
- `sensing.launch.py` — leak sensor, reed switch, Bar100 depth, WS2812B LED, hull-latch servo, TinyBMS battery voltage
- `vision.launch.py` — camera + underwater lights
- `mapping.launch.py`
- `planning.launch.py`
- `tf.launch.py`
- `localization.launch.py`
- `marker_localization.launch.py`

This means that out of the box, a launched robot has **no leak protection, no battery-voltage reporting, and no depth/camera/LED data** — only motors and leg kinematics/gaits. See `docs/SELQIE_LITE_2_SOP.md` Section 5 for the operational implications and how to enable sensing/vision for a session.

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
INNER_KP     = '6.0'   # motors 0, 2, 4, 6 — inner shafts
INNER_KD     = '0.35'
INNER_VEL_KD = '0.5'

OUTER_KP     = '6.0'   # motors 1, 3, 5, 7 — outer shafts
OUTER_KD     = '0.35'
OUTER_VEL_KD = '0.5'
```

These are tuning constants a maintainer edits directly in this file — treat the values above as a snapshot of what's in the repo at the time of writing, not a guarantee; check the file itself if precision matters. Note also that despite the "(reversed)" label historically used for inner shafts, the `InnerShaft()` helper below these constants does not currently pass `reverse_polarity='true'` — see `actuation/README.md` and `docs/SELQIE_LITE_2_SOP.md` Appendix B.

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
