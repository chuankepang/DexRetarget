"""Standalone ROKAE joint-state reader and guarded non-realtime joint mover.

This tool never starts the Cartesian realtime callback. Reading is the default
safe workflow. A joint move requires both an explicit target and ``--execute``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from output.real.drivers_rokae import RokaeXCoreDriver


ROBOT_DOF = {
    "xmate-6": 6,
    "xmate-er-pro-7": 7,
    "standard-6": 6,
}


def _format_values(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{float(value):+.8f}" for value in values) + "]"


def _print_state(driver: RokaeXCoreDriver) -> np.ndarray:
    joints = driver.get_joint_positions()
    if joints is None:
        raise RuntimeError("real ROKAE backend did not return joint positions")
    tcp = driver.get_tcp_pose()
    print(f"joint_position_rad: {_format_values(joints)}")
    print("base_T_tcp (row-major 4x4, translation in metres):")
    print(np.array2string(tcp, precision=8, suppress_small=False))
    print("\nCopy into rokae/config/rokae.yaml:")
    print("initial_joint_move:")
    print("  enabled: false  # keep false until the target is physically verified")
    print(f"  joint_position: {_format_values(joints)}")
    return joints


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read ROKAE state or execute one guarded MoveAbsJ command",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--robot-ip", default="192.168.0.160")
    parser.add_argument("--local-ip", default="192.168.0.100")
    parser.add_argument(
        "--robot-type",
        choices=sorted(ROBOT_DOF),
        default="xmate-er-pro-7",
    )
    parser.add_argument("--rt-network-tolerance", type=int, default=20)

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser(
        "read",
        help="Read joint positions and TCP without powering on or starting motion",
    )

    move = subparsers.add_parser(
        "move",
        help="Execute one non-realtime absolute joint move",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    move.add_argument(
        "--joints",
        type=float,
        nargs="+",
        required=True,
        metavar="RAD",
        help="Absolute joint target in radians; count must match robot type",
    )
    move.add_argument("--speed", type=float, default=0.05, help="Joint speed ratio")
    move.add_argument("--timeout", type=float, default=60.0)
    move.add_argument(
        "--max-joint-delta",
        type=float,
        default=0.25,
        help="Reject if any target-current difference exceeds this many radians",
    )
    move.add_argument(
        "--power-on",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Request power-on through xCoreSDK (default); use --no-power-on "
            "to require external power-on"
        ),
    )
    move.add_argument(
        "--execute",
        action="store_true",
        help="Required acknowledgement that the physical joint move may proceed",
    )
    move.add_argument(
        "--yes",
        action="store_true",
        help="Skip typing MOVE (requires --execute; use only after validation)",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if not 0 <= args.rt_network_tolerance <= 100:
        parser.error("--rt-network-tolerance must be in [0, 100]")
    if args.command == "move":
        expected_dof = ROBOT_DOF[args.robot_type]
        if len(args.joints) != expected_dof:
            parser.error(
                f"{args.robot_type} requires {expected_dof} joint values, "
                f"got {len(args.joints)}"
            )
        if not args.execute:
            parser.error("joint motion requires the explicit --execute flag")
        if not np.all(np.isfinite(args.joints)):
            parser.error("--joints must contain only finite values")
        if not 0.0 < args.speed <= 1.0:
            parser.error("--speed must be in (0, 1]")
        if args.timeout <= 0.0 or args.max_joint_delta <= 0.0:
            parser.error("--timeout and --max-joint-delta must be positive")

    driver = RokaeXCoreDriver(
        robot_ip=args.robot_ip,
        local_ip=args.local_ip,
        robot_type=args.robot_type,
        rt_network_tolerance=args.rt_network_tolerance,
    )
    try:
        print(f"Connecting ROKAE {args.robot_ip} from Orin {args.local_ip} ...")
        driver.connect()
        print("Current state:")
        current = _print_state(driver)
        if args.command == "read":
            print("\nRead complete. No power-on or motion command was requested.")
            return

        target = np.asarray(args.joints, dtype=np.float64)
        delta = target - current
        print("\nRequested absolute joint target (rad):")
        print(_format_values(target))
        print(f"target-current (rad): {_format_values(delta)}")
        print(
            f"Executing MoveAbsJ: speed={args.speed:.3f}, "
            f"max_joint_delta={args.max_joint_delta:.3f} rad"
        )
        if not args.yes:
            answer = input("Type MOVE to send this command: ").strip()
            if answer != "MOVE":
                raise SystemExit("Motion cancelled")
        driver.move_to_init(
            target,
            joint_speed=args.speed,
            timeout=args.timeout,
            max_joint_delta=args.max_joint_delta,
            power_on=args.power_on,
        )
        print("\nMove completed; measured final state:")
        _print_state(driver)
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
