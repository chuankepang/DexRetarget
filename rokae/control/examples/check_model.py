"""Compare host Pinocchio FK with itself, or with the controller when connected.

Without ``--execute`` this only prints the nominal init TCP. With ``--execute``
it reads the live robot and reports the FK mismatch. It never starts motion.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae.control.model import XmateErPro7
from rokae.examples.common import add_connection_args, build_driver, load_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Connect and compare against controller FK. Does not move.",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    robot = XmateErPro7()
    joints = np.asarray(
        config["robot"]["initial_joint_move"]["joint_position"], dtype=np.float64
    )
    pose = robot.fk(joints)
    print("nominal init joints", np.array2string(joints, precision=6))
    print("nominal TCP xyz", np.array2string(pose[:3, 3], precision=6))
    print(
        "tool +Z in base",
        np.array2string(pose[:3, 2], precision=4),
        "sigma_min",
        f"{robot.smallest_singular_value(joints):.4f}",
    )
    if not args.execute:
        print("Model self-check only. Re-run with --execute to read the robot.")
        return
    driver = build_driver(args, config)
    try:
        driver.connect()
        measured_q = driver.get_joint_positions()
        measured_pose = driver.get_tcp_pose()
        if measured_q is None:
            raise RuntimeError("robot did not return joints")
        translation, rotation = robot.pose_mismatch(measured_q, measured_pose)
        print(
            f"controller mismatch: {translation * 1000.0:.2f} mm / "
            f"{np.rad2deg(rotation):.2f} deg"
        )
        robot.require_controller_match(measured_q, measured_pose)
        print("controller FK is inside the joint-stream start tolerance")
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
