"""Non-realtime joint move.

Simulation prints a quintic trajectory and never contacts the robot.
``--execute`` sends the existing controller MoveAbsJ path.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae.control.interpolate import sample_joint_quintic
from rokae.control.model import XmateErPro7
from rokae.examples.common import (
    add_connection_args,
    add_motion_confirmation_args,
    build_driver,
    confirm_motion,
    load_config,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    parser.add_argument(
        "--delta",
        type=float,
        nargs=7,
        default=[0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
        help="Joint offset in radians added to the configured init pose",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    robot = XmateErPro7()
    start = np.asarray(config["robot"]["initial_joint_move"]["joint_position"], dtype=np.float64)
    goal = robot.clip(start + np.asarray(args.delta, dtype=np.float64))
    samples, duration = sample_joint_quintic(
        start, goal, robot.max_velocity, robot.max_acceleration
    )
    peak = np.max(np.abs(np.diff(samples, axis=0)) * (len(samples) - 1) / max(duration, 1e-9), axis=0)
    print(f"Quintic joint move: duration={duration:.3f} s, samples={len(samples)}")
    print("start", np.array2string(start, precision=5))
    print("goal ", np.array2string(goal, precision=5))
    print("peak inferred speed rad/s", np.array2string(peak, precision=3))
    print("final FK xyz", np.array2string(robot.fk(goal)[:3, 3], precision=5))
    if not args.execute:
        print("DRY RUN. Re-run with --execute to send MoveAbsJ.")
        return
    confirm_motion(args, "MoveAbsJ to the printed joint goal")
    non_realtime = config["non_realtime"]
    driver = build_driver(args, config)
    try:
        driver.connect()
        driver.move_joint(
            goal.tolist(),
            speed=int(non_realtime["joint_speed"]),
            timeout=float(non_realtime["timeout"]),
            max_joint_delta=max(0.5, float(np.max(np.abs(goal - start))) + 0.05),
            power_on=args.power_on,
        )
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
