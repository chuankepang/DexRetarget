#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time

from common import (
    add_connection_args,
    add_motion_confirmation_args,
    build_driver,
    confirm_motion,
    format_pose,
    load_config,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Small guarded realtime Cartesian sine test")
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    parser.add_argument("--axis", choices=("x", "y", "z"), default=None)
    parser.add_argument("--amplitude", type=float, default=None)
    parser.add_argument("--frequency", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--update-hz", type=float, default=None)
    args = parser.parse_args()
    config = load_config(args.config)
    test = config["realtime_test"]
    axis_name = args.axis or test["axis"]
    axis = {"x": 0, "y": 1, "z": 2}[axis_name]
    amplitude = args.amplitude or float(test["amplitude"])
    frequency = args.frequency or float(test["frequency"])
    duration = args.duration or float(test["duration"])
    update_hz = args.update_hz or float(test["update_hz"])
    if not 0.0 < amplitude <= 0.01:
        parser.error("amplitude must be in (0, 0.01] m")
    if not 0.0 < frequency <= 0.5 or not 0.0 < duration <= 30.0:
        parser.error("frequency must be <=0.5 Hz and duration <=30 s")
    confirm_motion(
        args,
        f"realtime {axis_name} sine, amplitude={amplitude} m, "
        f"frequency={frequency} Hz, duration={duration} s",
    )
    driver = build_driver(args, config)
    started = False
    try:
        driver.connect()
        origin = driver.get_tcp_pose()
        print("initial base_T_tcp:\n" + format_pose(origin))
        driver.start(power_on=args.power_on)
        started = True
        sequence = 0
        start = time.monotonic()
        period = 1.0 / update_hz
        deadline = start
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= duration:
                break
            target = origin.copy()
            target[axis, 3] += amplitude * math.sin(2.0 * math.pi * frequency * elapsed)
            if not driver.set_target_pose(target, source_timestamp=now, sequence=sequence):
                raise RuntimeError("C++ latest-value buffer rejected a fresh test target")
            sequence += 1
            deadline += period
            time.sleep(max(0.0, deadline - time.monotonic()))
        driver.hold()
        print("diagnostics:", driver.diagnostics())
    finally:
        try:
            if started:
                driver.stop()
        finally:
            driver.disconnect()


if __name__ == "__main__":
    main()
