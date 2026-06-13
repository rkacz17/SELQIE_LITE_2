#!/usr/bin/env python3
"""Conservative CubeMars position-gain sweep for legacy SELQIE gait files."""

import argparse
import math
import subprocess
import time

import rclpy
from actuation_msgs.msg import MotorCommand

from selqie_python.selqie import SELQIE, QOS_FAST


class GainAutoTuner:
    """Run candidate MIT gains, score command tracking, and keep the best pair."""

    def __init__(self, args):
        self.args = args
        self.robot = SELQIE()
        self.robot.init()
        self.commanded_positions = [0.0 for _ in range(self.robot.NUM_MOTORS)]
        self.samples = []

        for motor in range(self.robot.NUM_MOTORS):
            self.robot.create_subscription(
                MotorCommand,
                f'/motor{motor}/command',
                lambda msg, motor=motor: self._command_callback(motor, msg),
                QOS_FAST(),
            )

    def _command_callback(self, motor, msg):
        if msg.control_mode == MotorCommand.CONTROL_MODE_POSITION:
            self.commanded_positions[motor] = float(msg.pos_setpoint)

    def _set_gains(self, kp, kd):
        for motor in range(self.robot.NUM_MOTORS):
            node = f'/motor{motor}_node'
            for name, value in [('position_kp', kp), ('position_kd', kd)]:
                subprocess.run(
                    ['ros2', 'param', 'set', node, name, str(value)],
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    text=True,
                )

    def _record_sample(self):
        for motor, state in enumerate(self.robot._motor_states):
            error = self.commanded_positions[motor] - float(state.abs_position)
            self.samples.append((time.monotonic(), motor, error, float(state.velocity), float(state.current)))

    def _score_samples(self):
        if not self.samples:
            return math.inf, {}

        errors = [sample[2] for sample in self.samples]
        velocities = [sample[3] for sample in self.samples]
        currents = [sample[4] for sample in self.samples]
        rms_error = math.sqrt(sum(error * error for error in errors) / len(errors))
        rms_velocity = math.sqrt(sum(velocity * velocity for velocity in velocities) / len(velocities))
        rms_current = math.sqrt(sum(current * current for current in currents) / len(currents))
        peak_current = max(abs(current) for current in currents)
        score = rms_error + self.args.velocity_weight * rms_velocity + self.args.current_weight * rms_current
        if peak_current > self.args.max_current:
            score += (peak_current - self.args.max_current) * self.args.current_limit_weight
        return score, {
            'rms_error': rms_error,
            'rms_velocity': rms_velocity,
            'rms_current': rms_current,
            'peak_current': peak_current,
            'samples': len(self.samples),
        }

    def _run_trial(self, kp, kd):
        self._set_gains(kp, kd)
        time.sleep(self.args.settle_time)
        self.samples = []
        trajectories = self.robot.get_leg_trajectories_from_file(self.args.trajectory, self.args.frequency)
        rate = self.robot.create_rate(self.args.sample_hz)

        for loop in range(self.args.loops):
            self.robot.run_leg_trajectories(trajectories)
            end_time = time.monotonic() + (1.0 / self.args.frequency)
            while time.monotonic() < end_time:
                self._record_sample()
                rate.sleep()

        return self._score_samples()

    def run(self):
        self.robot.spin_background()
        best = (math.inf, None, None, {})
        try:
            for kp in self.args.kp:
                for kd in self.args.kd:
                    score, metrics = self._run_trial(kp, kd)
                    print(f'kp={kp:.4g} kd={kd:.4g} score={score:.6g} metrics={metrics}')
                    if score < best[0]:
                        best = (score, kp, kd, metrics)

            _, kp, kd, metrics = best
            self._set_gains(kp, kd)
            print(f'Best gains: position_kp={kp:.4g}, position_kd={kd:.4g}, metrics={metrics}')
        finally:
            self.robot.stop()


def _float_list(value):
    return [float(part) for part in value.split(',') if part]


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', help='Trajectory file in the leg_trajectory_publisher trajectories folder, e.g. walk.txt')
    parser.add_argument('--frequency', type=float, default=0.5, help='Trajectory playback frequency multiplier')
    parser.add_argument('--loops', type=int, default=1, help='Loops to run per candidate')
    parser.add_argument('--kp', type=_float_list, default=[0.5, 1.0, 2.0, 4.0, 8.0], help='Comma-separated Kp candidates')
    parser.add_argument('--kd', type=_float_list, default=[0.0, 0.02, 0.05, 0.1, 0.2], help='Comma-separated Kd candidates')
    parser.add_argument('--sample-hz', type=float, default=50.0, help='Scoring sample rate')
    parser.add_argument('--settle-time', type=float, default=0.5, help='Seconds to wait after each gain update')
    parser.add_argument('--velocity-weight', type=float, default=0.002, help='Penalty weight for RMS measured velocity')
    parser.add_argument('--current-weight', type=float, default=0.01, help='Penalty weight for RMS current')
    parser.add_argument('--max-current', type=float, default=8.0, help='Soft current limit before extra penalty')
    parser.add_argument('--current-limit-weight', type=float, default=10.0, help='Extra penalty per amp above max current')
    parsed = parser.parse_args(args)

    rclpy.init()
    try:
        GainAutoTuner(parsed).run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
