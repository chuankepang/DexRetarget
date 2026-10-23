"""Non-realtime Cartesian move.

Simulation samples a quintic translation plus SLERP and does not move hardware.
``--execute`` sends the existing controller MoveL path.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae.control.interpolate import sample_cartesian
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
        "--translation",
        type=float,
        nargs=3,
        default=[0.02, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
        help="Base-frame translation added to the init FK, metres",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()
    config = load_config(args.config)
    robot = XmateErPro7()
    start_q = np.asarray(
        config["robot"]["initial_joint_move"]["joint_position"], dtype=np.float64
    )
    start = robot.fk(start_q)
    goal = start.copy()
    goal[:3, 3] += np.asarray(args.translation, dtype=np.float64)
    samples = sample_cartesian(start, goal, args.duration)
    travel = np.linalg.norm(np.diff(samples[:, :3, 3], axis=0), axis=1).sum()
    print(f"Cartesian samples={len(samples)} path_length={travel * 1000.0:.2f} mm")
    print("start xyz", np.array2string(start[:3, 3], precision=5))
    print("goal  xyz", np.array2string(goal[:3, 3], precision=5))
    if not args.execute:
        print("DRY RUN. Re-run with --execute to send MoveL.")
        return
    confirm_motion(args, "MoveL by the printed base-frame translation")
    non_realtime = config["non_realtime"]
    driver = build_driver(args, config)
    try:
        driver.connect()
        current = driver.get_tcp_pose()
        target = current.copy()
        target[:3, 3] += np.asarray(args.translation, dtype=np.float64)
        driver.move_cartesian(
            target,
            speed=int(non_realtime["cartesian_speed"]),
            timeout=float(non_realtime["timeout"]),
            max_translation_delta=max(
                0.05, float(np.linalg.norm(args.translation)) + 0.01
            ),
            max_rotation_delta=float(non_realtime["max_rotation_delta"]),
            power_on=args.power_on,
        )
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
