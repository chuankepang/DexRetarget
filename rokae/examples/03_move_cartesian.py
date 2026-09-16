#!/usr/bin/env python3
from __future__ import annotations

import argparse

import numpy as np

from common import (
    add_connection_args,
    add_motion_confirmation_args,
    build_driver,
    confirm_motion,
    format_pose,
    load_config,
)
from anydexretarget.teleop import make_transform, quaternion_xyzw_to_matrix
from anydexretarget.teleop.pose import rotation_vector_to_matrix


def main() -> None:
    parser = argparse.ArgumentParser(description="Guarded ROKAE MoveL test")
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    targets = parser.add_mutually_exclusive_group(required=True)
    targets.add_argument(
        "--relative", type=float, nargs=6, metavar=("DX", "DY", "DZ", "RX", "RY", "RZ")
    )
    targets.add_argument(
        "--pose-xyzw", type=float, nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW")
    )
    parser.add_argument("--relative-frame", choices=("base", "tcp"), default="base")
    parser.add_argument("--speed", type=int, default=None)
    args = parser.parse_args()
    config = load_config(args.config)
    nrt = config["non_realtime"]
    driver = build_driver(args, config)
    try:
        driver.connect()
        current = driver.get_tcp_pose()
        if args.pose_xyzw:
            values = np.asarray(args.pose_xyzw, dtype=np.float64)
            target = make_transform(values[:3], quaternion_xyzw_to_matrix(values[3:]))
            description = "absolute base_T_tcp MoveL"
        else:
            values = np.asarray(args.relative, dtype=np.float64)
            delta = make_transform(values[:3], rotation_vector_to_matrix(values[3:]))
            if args.relative_frame == "tcp":
                target = current @ delta
            else:
                target = current.copy()
                target[:3, 3] += values[:3]
                target[:3, :3] = delta[:3, :3] @ current[:3, :3]
            description = f"relative {args.relative_frame}-frame MoveL {values.tolist()}"
        print("current base_T_tcp:\n" + format_pose(current))
        print("target base_T_tcp:\n" + format_pose(target))
        nrt_target = driver.preview_nrt_cartesian_target(target)
        print("converted NRT ref_T_end target:\n" + format_pose(nrt_target))
        confirm_motion(args, description)
        print("Sending MoveL to the C++ driver; progress will print every second...", flush=True)
        driver.move_cartesian(
            target,
            speed=args.speed or int(nrt["cartesian_speed"]),
            timeout=float(nrt["timeout"]),
            max_translation_delta=float(nrt["max_translation_delta"]),
            max_rotation_delta=float(nrt["max_rotation_delta"]),
            power_on=args.power_on,
        )
        print("final base_T_tcp:\n" + format_pose(driver.get_tcp_pose()))
    except BaseException:
        if args.execute:
            try:
                driver.stop()
            except Exception as stop_error:
                print(f"warning: stop failed: {stop_error}")
        raise
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
