"""State-only xCoreSDK connection check; never powers on or starts motion."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from output.real.drivers_rokae import RokaeXCoreDriver


def main() -> None:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--robot-ip", default="192.168.0.160")
    parser.add_argument("--local-ip", required=True)
    parser.add_argument(
        "--robot-type",
        choices=["xmate-6", "xmate-er-pro-7", "standard-6"],
        default="xmate-er-pro-7",
    )
    parser.add_argument("--rt-network-tolerance", type=int, default=20)
    args = parser.parse_args()
    driver = RokaeXCoreDriver(
        args.robot_ip,
        args.local_ip,
        args.robot_type,
        args.rt_network_tolerance,
    )
    try:
        driver.connect()
        joints = driver.get_joint_positions()
        pose = driver.get_tcp_pose()
        print("Connected; no power-on or realtime motion was requested.")
        print("joint positions (rad):")
        print(joints)
        print("base_T_tcp (row-major 4x4):")
        print(pose)
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
