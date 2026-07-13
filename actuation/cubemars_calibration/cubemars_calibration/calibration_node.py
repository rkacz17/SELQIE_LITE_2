#!/usr/bin/env python3
"""
Automatic gain calibration for CubeMars AK-series motors.

Runs three sequential phases per motor joint:

  Phase 1 — Kp search
    Steps the motor through ±STEP_AMP_RAD increments with a fixed low Kd.
    Candidates are scored by settling time, overshoot, and oscillation.
    A binary refinement pass narrows the result.

  Phase 2 — Kd search
    With the chosen Kp, sweeps Kd candidates.  Finds the lightest damping that
    eliminates oscillation and keeps overshoot below TARGET_OVERSHOOT_PCT.

  Phase 3 — velocity_kd search
    Commands a sinusoidal velocity profile (kp=0 mode) at multiple amplitudes.
    Scores by RMS tracking error and inter-sample velocity variance (stutter).

After all joints are processed the node writes:
  • A machine-readable YAML file at `output_file`
  • A ready-to-paste Python snippet for selqie_bringup/launch/actuation.launch.py

Prerequisites:
  All motor_node instances must already be running and their motors started
  (special_cmd "start") before this node is launched.  Use the calibration
  launch file, which handles this automatically.
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from motor_interfaces.msg import MotorState

# ─────────────────────────────────────────────────────────────────────────────
# Protocol limits (must match cubemars_v2_ros/motor_node.py LIMITS table)
# ─────────────────────────────────────────────────────────────────────────────
MOTOR_LIMITS: Dict[str, dict] = {
    'AK10-9':  dict(kp_max=500.0, kd_max=5.0, v_max=50.0,  t_max=65.0),
    'AK60-6':  dict(kp_max=500.0, kd_max=5.0, v_max=45.0,  t_max=15.0),
    'AK70-10': dict(kp_max=500.0, kd_max=5.0, v_max=50.0,  t_max=25.0),
    'AK80-6':  dict(kp_max=500.0, kd_max=5.0, v_max=76.0,  t_max=12.0),
    'AK80-9':  dict(kp_max=500.0, kd_max=5.0, v_max=50.0,  t_max=18.0),
    'AK80-64': dict(kp_max=500.0, kd_max=5.0, v_max=8.0,   t_max=144.0),
    'AK80-8':  dict(kp_max=500.0, kd_max=5.0, v_max=37.5,  t_max=32.0),
    'AK40-10': dict(kp_max=500.0, kd_max=5.0, v_max=45.5,  t_max=5.0),
}

# ─────────────────────────────────────────────────────────────────────────────
# Calibration constants
# ─────────────────────────────────────────────────────────────────────────────
STEP_AMP_RAD        = 0.25    # Step size for position tests (rad, ~14°)
COLLECT_DURATION_S  = 1.2     # How long to collect samples per step test
HOLD_DURATION_S     = 0.4     # Hold time between tests (let motor settle)
SETTLE_BAND_FRAC    = 0.02    # 2 % of step amplitude defines "settled"
TARGET_OVERSHOOT    = 15.0    # % — accept up to this overshoot
OSCILLATION_ZC_MAX  = 6       # Velocity zero-crossings in tail → oscillating
TEMP_ABORT_C        = 70      # Abort calibration if temperature exceeds this

VEL_TEST_AMPS       = [5.0, 15.0, 30.0]   # rad/s — test at low, mid, high speed
VEL_TEST_FREQ_HZ    = 0.5                  # Sinusoid frequency for vel tests
VEL_COLLECT_S       = 4.0                  # Seconds per velocity test

# Conservative safe defaults used when a better value cannot be found
SAFE_KP     = 3.0
SAFE_KD     = 0.3
SAFE_VEL_KD = 0.5

# Candidate grids (coarse then refined)
KP_GRID     = [1.0, 3.0, 5.0, 10.0, 20.0, 35.0, 50.0, 75.0, 100.0]
KD_GRID     = [0.02, 0.05, 0.10, 0.20, 0.50, 1.00, 2.00]
VEL_KD_GRID = [0.05, 0.10, 0.20, 0.50, 1.00, 2.00, 3.00]


# ─────────────────────────────────────────────────────────────────────────────
# Thread-safe motor state buffer
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class MotorData:
    lock: threading.Lock = field(default_factory=threading.Lock)
    _event: threading.Event = field(default_factory=threading.Event)
    abs_position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: int = 0
    _history: list = field(default_factory=list)   # [(t, pos, vel, torq)]
    _initialized: bool = False

    def on_state(self, msg: MotorState) -> None:
        with self.lock:
            self.abs_position  = msg.abs_position
            self.velocity      = msg.velocity
            self.torque        = msg.torque
            self.temperature   = msg.temperature
            self._initialized  = True
        self._event.set()

    def wait(self, timeout: float = 2.0) -> bool:
        """Block until a new motor state arrives (or timeout)."""
        self._event.clear()
        return self._event.wait(timeout=timeout)

    def is_ready(self) -> bool:
        with self.lock:
            return self._initialized

    def record(self, t: float) -> None:
        with self.lock:
            self._history.append((t, self.abs_position, self.velocity, self.torque))

    def clear_history(self) -> None:
        with self.lock:
            self._history.clear()

    @property
    def history(self) -> list:
        with self.lock:
            return list(self._history)


# ─────────────────────────────────────────────────────────────────────────────
# Step-response metrics
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class StepMetrics:
    kp: float
    kd: float
    settling_time: float        # seconds to first reach 2 % band
    overshoot_pct: float        # % relative to step amplitude
    ss_error: float             # |mean(tail positions) − target| (rad)
    oscillating: bool           # True if too many velocity zero-crossings
    score: float = 0.0          # lower is better

    def compute_score(self) -> None:
        if self.oscillating:
            self.score = float('inf')
            return
        overshoot_pen = max(0.0, self.overshoot_pct - TARGET_OVERSHOOT) * 4.0
        self.score = (self.settling_time * 2.0
                      + overshoot_pen
                      + self.ss_error * 40.0)


@dataclass
class VelMetrics:
    vel_kd: float
    rms_error: float            # RMS of (v_cmd − v_actual)
    variance: float             # Variance of tracking error (stutter)
    score: float = 0.0

    def compute_score(self) -> None:
        self.score = self.rms_error + 2.0 * math.sqrt(max(self.variance, 0.0))


# ─────────────────────────────────────────────────────────────────────────────
# Calibration node
# ─────────────────────────────────────────────────────────────────────────────
class CubemarsCalibrationNode(Node):

    def __init__(self) -> None:
        super().__init__('cubemars_calibration_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('joint_names',
                               ['motor0', 'motor1', 'motor2', 'motor3',
                                'motor4', 'motor5', 'motor6', 'motor7'])
        self.declare_parameter('motor_type', 'AK40-10')
        self.declare_parameter('output_file',
                               '/tmp/cubemars_calibrated_gains.yaml')
        self.declare_parameter('dry_run', False)

        self._joints      = self.get_parameter('joint_names').value
        self._motor_type  = self.get_parameter('motor_type').value
        self._output_file = self.get_parameter('output_file').value
        self._dry_run     = self.get_parameter('dry_run').value
        self._limits      = MOTOR_LIMITS.get(self._motor_type,
                                              MOTOR_LIMITS['AK40-10'])

        # ── Per-joint ROS interfaces ────────────────────────────────────────
        self._data: Dict[str, MotorData] = {}
        self._cmd_pubs: Dict[str, object] = {}
        self._special_pubs: Dict[str, object] = {}

        for name in self._joints:
            self._data[name] = MotorData()
            self._cmd_pubs[name] = self.create_publisher(
                Float64MultiArray, f'/{name}/mit_cmd', 10)
            self._special_pubs[name] = self.create_publisher(
                String, f'/{name}/special_cmd', 10)
            self.create_subscription(
                MotorState, f'/{name}/motor_state',
                lambda msg, n=name: self._data[n].on_state(msg), 10)

        # ── Calibration runs in its own thread ──────────────────────────────
        self._results: dict = {}
        self._cal_thread = threading.Thread(
            target=self._run_all, name='cal_thread', daemon=True)
        self._cal_thread.start()

    # ── Low-level command helpers ─────────────────────────────────────────────

    def _send_mit(self, joint: str,
                  p: float, v: float, kp: float, kd: float, t: float) -> None:
        if self._dry_run:
            return
        msg = Float64MultiArray()
        msg.data = [float(p), float(v), float(kp), float(kd), float(t)]
        self._cmd_pubs[joint].publish(msg)

    def _send_special(self, joint: str, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._special_pubs[joint].publish(msg)

    # ── Safety ───────────────────────────────────────────────────────────────

    def _is_safe(self, joint: str) -> bool:
        temp = self._data[joint].temperature
        if temp > TEMP_ABORT_C:
            self.get_logger().error(
                f'{joint}: temperature {temp}°C > limit {TEMP_ABORT_C}°C — aborting')
            return False
        return True

    def _check_limits(self, value: float, lo: float, hi: float) -> bool:
        return lo <= value <= hi

    # ── Hold position (keeps motor commanded between tests) ──────────────────

    def _hold(self, joint: str, kp: float, kd: float,
              duration: float = HOLD_DURATION_S) -> None:
        d = self._data[joint]
        d.wait(timeout=0.5)
        with d.lock:
            pos = d.abs_position
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            self._send_mit(joint, pos, 0.0, kp, kd, 0.0)
            d.wait(timeout=0.02)

    # ── Step-response test ───────────────────────────────────────────────────

    def _step_test(self, joint: str,
                   kp: float, kd: float) -> Optional[StepMetrics]:
        d = self._data[joint]

        if not d.wait(timeout=1.0) or not self._is_safe(joint):
            return None

        with d.lock:
            origin = d.abs_position

        # Alternate direction to keep motor near home
        if not hasattr(self, '_step_sign'):
            self._step_sign: Dict[str, float] = {}
        sign = self._step_sign.get(joint, 1.0)
        self._step_sign[joint] = -sign
        target = origin + sign * STEP_AMP_RAD

        d.clear_history()
        t0 = time.monotonic()
        while time.monotonic() - t0 < COLLECT_DURATION_S:
            self._send_mit(joint, target, 0.0, kp, kd, 0.0)
            d.wait(timeout=0.02)
            d.record(time.monotonic() - t0)

        self._hold(joint, kp, kd)

        history = d.history
        if len(history) < 10:
            return None

        times  = [h[0] for h in history]
        pos    = [h[1] for h in history]
        vels   = [h[2] for h in history]

        amp = abs(target - origin)
        band = SETTLE_BAND_FRAC * amp if amp > 1e-6 else 0.005

        # Settling time
        settling_time = times[-1]
        for i, p in enumerate(pos):
            if abs(p - target) <= band:
                settling_time = times[i]
                break

        # Peak overshoot
        if sign > 0:
            peak = max(pos)
        else:
            peak = min(pos)
        overshoot_pct = max(0.0, (abs(peak - target) / amp) * 100.0) if amp > 1e-6 else 0.0

        # Steady-state error (last 15 % of samples)
        tail_n = max(5, len(pos) // 7)
        ss_error = abs(sum(pos[-tail_n:]) / tail_n - target)

        # Oscillation: zero-crossings in velocity tail
        mid = len(vels) // 2
        v_tail = vels[mid:]
        crossings = sum(
            1 for i in range(1, len(v_tail))
            if v_tail[i - 1] * v_tail[i] < -1e-6)
        oscillating = crossings > OSCILLATION_ZC_MAX

        m = StepMetrics(
            kp=kp, kd=kd,
            settling_time=settling_time,
            overshoot_pct=overshoot_pct,
            ss_error=ss_error,
            oscillating=oscillating)
        m.compute_score()
        return m

    def _log_step(self, joint: str, m: Optional[StepMetrics]) -> None:
        if m is None:
            self.get_logger().warn(f'  {joint}: no data received')
            return
        osc = 'OSCILLATING' if m.oscillating else 'stable'
        self.get_logger().info(
            f'  kp={m.kp:6.1f} kd={m.kd:.3f} | '
            f'settle={m.settling_time:.3f}s '
            f'overshoot={m.overshoot_pct:5.1f}% '
            f'ss_err={m.ss_error:.4f}rad '
            f'[{osc}] → score={m.score:.3f}')

    # ── Velocity-tracking test ───────────────────────────────────────────────

    def _vel_test(self, joint: str, kp: float, kd: float,
                  vel_kd: float) -> Optional[VelMetrics]:
        d = self._data[joint]
        errors: List[float] = []

        for amp in VEL_TEST_AMPS:
            if not self._is_safe(joint):
                return None

            t0 = time.monotonic()
            while time.monotonic() - t0 < VEL_COLLECT_S:
                t = time.monotonic() - t0
                v_des = amp * math.sin(2.0 * math.pi * VEL_TEST_FREQ_HZ * t)
                self._send_mit(joint, 0.0, v_des, 0.0, vel_kd, 0.0)
                d.wait(timeout=0.02)
                with d.lock:
                    v_actual = d.velocity
                errors.append(v_des - v_actual)

            # Bring motor back to rest
            self._hold(joint, kp, kd, duration=HOLD_DURATION_S)

        if not errors:
            return None

        n = len(errors)
        mean_e  = sum(errors) / n
        rms_e   = math.sqrt(sum(e * e for e in errors) / n)
        variance = sum((e - mean_e) ** 2 for e in errors) / n

        m = VelMetrics(vel_kd=vel_kd, rms_error=rms_e, variance=variance)
        m.compute_score()
        return m

    def _log_vel(self, joint: str, m: Optional[VelMetrics]) -> None:
        if m is None:
            self.get_logger().warn(f'  {joint}: no velocity data')
            return
        self.get_logger().info(
            f'  vel_kd={m.vel_kd:.3f} | '
            f'rms_err={m.rms_error:.4f}rad/s '
            f'variance={m.variance:.4f} '
            f'→ score={m.score:.4f}')

    # ── Phase 1: Kp search ───────────────────────────────────────────────────

    def _tune_kp(self, joint: str, init_kd: float = 0.20) -> float:
        self.get_logger().info(
            f'{joint} ▶ Phase 1: Kp sweep (Kd={init_kd})')

        kp_max = self._limits['kp_max']
        candidates = [k for k in KP_GRID if k <= kp_max]

        results: List[StepMetrics] = []
        for kp in candidates:
            m = self._step_test(joint, kp, init_kd)
            self._log_step(joint, m)
            if m is not None:
                results.append(m)
            # Stop early once oscillation starts (higher Kp won't help)
            if m is not None and m.oscillating:
                self.get_logger().info(
                    f'  → oscillation onset at Kp={kp}; stopping sweep')
                break

        if not results:
            self.get_logger().warn(f'{joint}: Phase 1 returned no data; using Kp={SAFE_KP}')
            return SAFE_KP

        # Pick lowest-score stable result, then do one binary refinement pass
        stable = [r for r in results if not r.oscillating]
        if not stable:
            self.get_logger().warn(
                f'{joint}: no stable Kp found; using safe default {SAFE_KP}')
            return SAFE_KP

        best = min(stable, key=lambda r: r.score)

        # Refinement: try halfway between best and the next-higher candidate
        idx = candidates.index(best.kp)
        if idx + 1 < len(candidates):
            kp_try = (best.kp + candidates[idx + 1]) / 2.0
            m2 = self._step_test(joint, kp_try, init_kd)
            self._log_step(joint, m2)
            if m2 is not None and not m2.oscillating and m2.score < best.score:
                best = m2

        self.get_logger().info(f'{joint}: ✓ Kp = {best.kp:.2f}  (score {best.score:.3f})')
        return best.kp

    # ── Phase 2: Kd search ───────────────────────────────────────────────────

    def _tune_kd(self, joint: str, kp: float) -> float:
        self.get_logger().info(f'{joint} ▶ Phase 2: Kd sweep (Kp={kp:.2f})')

        kd_max = self._limits['kd_max']
        candidates = [k for k in KD_GRID if k <= kd_max]

        results: List[StepMetrics] = []
        for kd in candidates:
            m = self._step_test(joint, kp, kd)
            self._log_step(joint, m)
            if m is not None:
                results.append(m)

        if not results:
            self.get_logger().warn(f'{joint}: Phase 2 no data; using Kd={SAFE_KD}')
            return SAFE_KD

        stable = [r for r in results if not r.oscillating]
        if not stable:
            stable = results  # Fall back to best-of-bad

        best = min(stable, key=lambda r: r.score)
        self.get_logger().info(f'{joint}: ✓ Kd = {best.kd:.3f}  (score {best.score:.3f})')
        return best.kd

    # ── Phase 3: velocity Kd search ──────────────────────────────────────────

    def _tune_vel_kd(self, joint: str, kp: float, kd: float) -> float:
        v_max = self._limits['v_max']
        amps  = [a for a in VEL_TEST_AMPS if a <= v_max]
        if not amps:
            return SAFE_VEL_KD

        self.get_logger().info(
            f'{joint} ▶ Phase 3: velocity Kd sweep '
            f'(amps={amps} rad/s, f={VEL_TEST_FREQ_HZ} Hz)')

        kd_max = self._limits['kd_max']
        candidates = [k for k in VEL_KD_GRID if k <= kd_max]

        results: List[VelMetrics] = []
        for vel_kd in candidates:
            m = self._vel_test(joint, kp, kd, vel_kd)
            self._log_vel(joint, m)
            if m is not None:
                results.append(m)

        if not results:
            self.get_logger().warn(f'{joint}: Phase 3 no data; using vel_kd={SAFE_VEL_KD}')
            return SAFE_VEL_KD

        best = min(results, key=lambda r: r.score)
        self.get_logger().info(
            f'{joint}: ✓ vel_kd = {best.vel_kd:.3f}  (score {best.score:.4f})')
        return best.vel_kd

    # ── Per-joint calibration entry point ────────────────────────────────────

    def _calibrate_joint(self, joint: str) -> None:
        self.get_logger().info(
            f'\n{"═"*60}\n  Calibrating {joint}  ({self._motor_type})\n{"═"*60}')

        d = self._data[joint]

        # Wait for first motor state before commanding anything
        if not d.wait(timeout=5.0):
            self.get_logger().error(
                f'{joint}: no motor_state received — is motor_node running '
                f'and the motor started?  Skipping.')
            return

        # Warn if temperature already elevated
        if d.temperature > TEMP_ABORT_C - 10:
            self.get_logger().warn(
                f'{joint}: temperature {d.temperature}°C is high — monitor closely')

        # Warm-up hold so the motor is actively commutated before we test
        self.get_logger().info(f'{joint}: warm-up hold (1.0 s) …')
        self._hold(joint, SAFE_KP, SAFE_KD, duration=1.0)

        best_kp     = self._tune_kp(joint)
        best_kd     = self._tune_kd(joint, best_kp)
        best_vel_kd = self._tune_vel_kd(joint, best_kp, best_kd)

        # Clamp to hardware limits
        best_kp     = min(max(best_kp,     0.0), self._limits['kp_max'])
        best_kd     = min(max(best_kd,     0.0), self._limits['kd_max'])
        best_vel_kd = min(max(best_vel_kd, 0.0), self._limits['kd_max'])

        self._results[joint] = {
            'position_kp': round(best_kp,     2),
            'position_kd': round(best_kd,     3),
            'velocity_kd': round(best_vel_kd, 3),
        }
        self.get_logger().info(
            f'{joint}: COMPLETE — '
            f'position_kp={best_kp:.2f}  '
            f'position_kd={best_kd:.3f}  '
            f'velocity_kd={best_vel_kd:.3f}')

        # Leave motor in a stable hold state
        self._hold(joint, best_kp, best_kd, duration=0.5)

    # ── Top-level sequencer ──────────────────────────────────────────────────

    def _run_all(self) -> None:
        # Brief delay so all subscriptions are confirmed live
        time.sleep(1.5)

        if self._dry_run:
            self.get_logger().warn(
                'DRY RUN — no MIT commands will be sent to motors')

        for joint in self._joints:
            self._calibrate_joint(joint)

        self._write_results()
        self.get_logger().info('All joints done.  Shutting down.')
        rclpy.shutdown()

    # ── Output ───────────────────────────────────────────────────────────────

    def _write_results(self) -> None:
        if not self._results:
            self.get_logger().warn('No calibration results to save.')
            return

        # ── Summary table ───────────────────────────────────────────────────
        self.get_logger().info('\n' + '═' * 60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('─' * 60)
        self.get_logger().info(
            f'{"Joint":<12} {"position_kp":>12} {"position_kd":>12} {"velocity_kd":>12}')
        self.get_logger().info('─' * 60)
        for j, g in self._results.items():
            self.get_logger().info(
                f'{j:<12} {g["position_kp"]:>12.2f} '
                f'{g["position_kd"]:>12.3f} '
                f'{g["velocity_kd"]:>12.3f}')

        # Suggest group gains (conservative: min across motors in each group)
        inner_ids = {'motor0', 'motor2', 'motor4', 'motor6'}
        outer_ids = {'motor1', 'motor3', 'motor5', 'motor7'}
        inner = [self._results[j] for j in self._results if j in inner_ids]
        outer = [self._results[j] for j in self._results if j in outer_ids]

        self.get_logger().info('\nSuggested group values for actuation.launch.py:')
        self.get_logger().info('(conservative: minimum across each shaft group)')

        def group_suggest(grp: list, label: str) -> dict:
            if not grp:
                return {}
            return {
                'position_kp': round(min(g['position_kp'] for g in grp), 2),
                'position_kd': round(min(g['position_kd'] for g in grp), 3),
                'velocity_kd': round(min(g['velocity_kd'] for g in grp), 3),
            }

        inner_sug = group_suggest(inner, 'INNER (reversed polarity)')
        outer_sug = group_suggest(outer, 'OUTER')

        snippet_lines = [
            '# ── paste these lines into selqie_bringup/launch/actuation.launch.py ──',
        ]
        if inner_sug:
            snippet_lines += [
                f"INNER_KP     = '{inner_sug['position_kp']}'",
                f"INNER_KD     = '{inner_sug['position_kd']}'",
                f"INNER_VEL_KD = '{inner_sug['velocity_kd']}'",
            ]
        if outer_sug:
            snippet_lines += [
                f"OUTER_KP     = '{outer_sug['position_kp']}'",
                f"OUTER_KD     = '{outer_sug['position_kd']}'",
                f"OUTER_VEL_KD = '{outer_sug['velocity_kd']}'",
            ]

        for line in snippet_lines:
            self.get_logger().info(line)

        # ── Write YAML ──────────────────────────────────────────────────────
        output = {
            'motor_type': self._motor_type,
            'joints': self._results,
            'suggested_groups': {
                'inner_shafts': inner_sug,
                'outer_shafts': outer_sug,
            },
            'actuation_launch_snippet': '\n'.join(snippet_lines),
        }
        try:
            with open(self._output_file, 'w') as f:
                yaml.dump(output, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f'Results written to: {self._output_file}')
        except OSError as exc:
            self.get_logger().error(f'Could not write output file: {exc}')


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = CubemarsCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
