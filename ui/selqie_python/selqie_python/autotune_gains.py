#!/usr/bin/env python3
"""Conservative CubeMars position-gain sweep for legacy SELQIE gait files."""

import argparse

import rclpy

from selqie_python.selqie import SELQIE


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
    parser.add_argument('--no-ready', action='store_true', help='Do not send start/MIT-mode special commands before tuning')
    parsed = parser.parse_args(args)

    rclpy.init()
    robot = SELQIE()
    try:
        robot.init()
        robot.spin_background()
        summary = robot.autotune_cubemars_position_gains(
            parsed.trajectory,
            frequency=parsed.frequency,
            kp_candidates=parsed.kp,
            kd_candidates=parsed.kd,
            loops=parsed.loops,
            sample_hz=parsed.sample_hz,
            settle_time=parsed.settle_time,
            velocity_weight=parsed.velocity_weight,
            current_weight=parsed.current_weight,
            max_current=parsed.max_current,
            current_limit_weight=parsed.current_limit_weight,
            ready_motors=not parsed.no_ready,
        )
        for result in summary['results']:
            print(
                f"kp={result['kp']:.4g} kd={result['kd']:.4g} "
                f"score={result['score']:.6g} metrics={result['metrics']}"
            )
        best = summary['best']
        print(
            f"Best gains: position_kp={best['kp']:.4g}, "
            f"position_kd={best['kd']:.4g}, metrics={best['metrics']}"
        )
    finally:
        robot.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
