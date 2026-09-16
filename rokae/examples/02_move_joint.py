#!/usr/bin/env python3
from __future__ import annotations

import argparse

from common import (
    add_connection_args,
    add_motion_confirmation_args,
    build_driver,
    confirm_motion,
    format_vector,
    load_config,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Guarded ROKAE absolute joint move")
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    parser.add_argument("--joints", type=float, nargs="+", required=True)
    parser.add_argument(
        "--speed",
        type=int,
        default=None,
        help=(
            "xCoreSDK joint speed setting (5..1000); tiers are approximately "
            "<100=10%%, 100..200=30%%, 200..500=50%%, "
            "500..800=80%%, >800=100%%"
        ),
    )
    parser.add_argument("--max-joint-delta", type=float, default=None)
    args = parser.parse_args()
    config = load_config(args.config)
    nrt = config["non_realtime"]
    driver = build_driver(args, config)
    try:
        driver.connect()
        current = driver.get_joint_positions()
        print(f"current: {format_vector(current)}")
        print(f"target:  {format_vector(args.joints)}")
        confirm_motion(args, "absolute MoveAbsJ; values are radians")
        print("Sending MoveAbsJ to the C++ driver; progress will print every second...", flush=True)
        driver.move_joint(
            args.joints,
            speed=args.speed or int(nrt["joint_speed"]),
            timeout=float(nrt["timeout"]),
            max_joint_delta=args.max_joint_delta or float(nrt["max_joint_delta"]),
            power_on=args.power_on,
        )
        print(f"final:   {format_vector(driver.get_joint_positions())}")
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
