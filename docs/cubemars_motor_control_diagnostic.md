# CubeMars AK40-10 motor-control diagnostic

This diagnostic is based on the current SELQIE control path and the CubeMars AK Series Module Driver Manual V1.0.18 for AK 2.0 robotic actuators.

## Control path

`run_trajectory walk.txt 1 0.5` follows this path:

1. `selqie_python` parses the legacy text gait into per-leg `LegTrajectory` messages.
2. `leg_trajectory_publisher_node` releases due `LegCommand` samples by timestamp.
3. `leg_kinematics` converts foot position/velocity/force commands to `MotorCommand` messages.
4. `cubemars_v2_ros/motor_node.py` converts `MotorCommand` into CubeMars MIT CAN frames.

## Findings

### 1. AK40-10 with the AK 2.0 driver uses the standard MIT protocol

The AK 2.0 manual's MIT CAN example uses the standard MIT frame layout:

`position, velocity, Kp, Kd, torque`

on the motor's standard CAN ID. The driver also uses the standard special frames for MIT mode control: `0xFC` to enter MIT mode, `0xFD` to exit MIT mode, and `0xFE` to set the current position as zero.

For this robot, the default protocol must therefore be `ak2` (alias `tmotor`), not the AK 3.0 extended-frame protocol.

### 2. Runtime gain updates are necessary for autotuning

The motor node previously read `position_kp` and `position_kd` only at startup. Autotuning can only work if the motor node applies ROS parameter updates immediately.

### 3. MIT mode startup differs by protocol

The AK 2.0 protocol uses `0xFC/0xFD/0xFE` special frames for start/exit/zero. The driver sends those frames in `ak2`/`tmotor` mode only.

### 4. Feedback parsing differs by protocol

The AK 2.0 MIT reply layout is a driver ID byte followed by packed 16-bit position, 12-bit velocity, 12-bit torque, and temperature. The manual's receive routine subtracts 40 from the temperature byte to obtain °C, which is now reflected in the parser.

### 5. Remaining checks before high-gain walking

- Confirm each launched AK40-10 motor node uses `protocol:=ak2` or leaves the default `ak2` value unchanged.
- Confirm CAN bitrate is 1 Mbps.
- Confirm CubeMarsTool / driver settings enable CAN periodic feedback or query-reply as expected.
- Confirm the configured CAN IDs match `/motorN_node` launch arguments.
- Start with low candidate gains and current limits, then widen the autotune sweep only after observing stable low-speed movement.

## Recommended first autotune command

```bash
ros2 run selqie_python selqie_autotune_gains walk.txt --frequency 0.5 --loops 1 --kp 0.25,0.5,1,2 --kd 0,0.02,0.05,0.1 --max-current 4
```

If this does not move the motors, test one motor node directly with a small position command and verify that the node logs `Protocol: ak2`, sends standard CAN frames on the motor ID, and sends the `0xFC` MIT-mode entry frame when started.
