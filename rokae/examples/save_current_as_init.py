#!/usr/bin/env python3
from __future__ import annotations

import argparse

from common import add_connection_args, build_driver, format_pose, format_vector, load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Print current ROKAE state as init YAML; never overwrites files")
    add_connection_args(parser)
    args = parser.parse_args()
    driver = build_driver(args, load_config(args.config))
    try:
        driver.connect()
        joints = driver.get_joint_positions()
        tcp = driver.get_tcp_pose()
        print("Copy only after checking the values:\n")
        print("robot:")
        print("  initial_joint_move:")
        print("    enabled: true")
        print(f"    joint_position: {joints.tolist()}")
        print("\n# base_T_tcp at capture time")
        print(f"# flat row-major: {tcp.reshape(-1).tolist()}")
        print("\nReadable values:")
        print("joints:", format_vector(joints))
        print("base_T_tcp:\n" + format_pose(tcp))
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
