"""1 kHz joint-position servo with jerk limiting.

Default is a hardware-free simulation of the C++ JointRealtimeCore. ``--execute``
streams the same small joint offset to the robot in jointPosition mode.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae.control.model import XmateErPro7
from rokae.control.smoothing import JointStreamConfig
from rokae.examples.common import (
    add_connection_args,
    add_motion_confirmation_args,
    build_driver,
    confirm_motion,
    load_config,
)
from rokae.python import _rokae_cpp


def _cpp_config(limits: JointStreamConfig):
    cfg = _rokae_cpp.JointRealtimeConfig()
    cfg.max_velocity = limits.max_velocity.tolist()
    cfg.max_acceleration = limits.max_acceleration.tolist()
    cfg.max_jerk = limits.max_jerk.tolist()
    cfg.position_min = limits.position_min.tolist()
    cfg.position_max = limits.position_max.tolist()
    cfg.reference_cutoff_hz = limits.reference_cutoff_hz
    cfg.deadband = limits.deadband
    cfg.max_target_delta = limits.max_target_delta
    cfg.hold_timeout = limits.hold_timeout
    cfg.stop_timeout = limits.stop_timeout
    cfg.max_source_age = limits.max_source_age
    cfg.future_tolerance = limits.future_tolerance
    cfg.validate()
    return cfg


def simulate(start: np.ndarray, goal: np.ndarray, limits: JointStreamConfig, seconds: float) -> None:
    core = _rokae_cpp.JointRealtimeCore(_cpp_config(limits))
    core.reset(start.tolist(), 0.0)
    sequence = 1
    if not core.publish(goal.tolist(), 0.0, sequence, 0.0):
        raise RuntimeError("simulation rejected the joint target")
    previous = start.copy()
    peak = 0.0
    steps = int(seconds * 1000)
    for index in range(1, steps + 1):
        timestamp = index * 0.001
        if index % 50 == 0:
            sequence += 1
            core.publish(goal.tolist(), timestamp, sequence, timestamp)
        command = np.asarray(core.step(timestamp), dtype=np.float64)
        peak = max(peak, float(np.max(np.abs(command - previous))))
        previous = command
    state = core.diagnostics(steps * 0.001)["watchdog_state"]
    final_error = float(np.max(np.abs(previous - goal)))
    print(
        f"sim steps={steps} peak_step={peak:.6f} rad "
        f"final_error={final_error:.6f} rad state={state}"
    )
    if state != "tracking" or final_error > 0.005:
        raise RuntimeError("joint servo did not settle on the simulated target")
    if peak > float(np.max(limits.max_velocity)) * 0.001 * 1.05:
        raise RuntimeError("joint command jumped faster than the velocity limit")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    parser.add_argument("--joint", type=int, default=2, help="1-based joint index")
    parser.add_argument("--delta", type=float, default=0.05, help="radians")
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()
    config = load_config(args.config)
    robot = XmateErPro7()
    start = np.asarray(config["robot"]["initial_joint_move"]["joint_position"], dtype=np.float64)
    goal = start.copy()
    goal[args.joint - 1] += args.delta
    goal = robot.clip(goal)
    limits = JointStreamConfig.from_limits(robot.stream_limits)
    print("servo joint", args.joint, "delta", args.delta)
    simulate(start, goal, limits, args.seconds)
    if not args.execute:
        print("SIMULATION only. Re-run with --execute to stream on the robot.")
        return
    confirm_motion(args, f"1 kHz jointPosition offset on J{args.joint}")
    driver = build_driver(args, config)
    try:
        driver.connect()
        measured = driver.get_joint_positions()
        if measured is None:
            raise RuntimeError("robot did not return joints")
        driver.start(power_on=args.power_on, control_mode="pinocchio", kinematics=robot)
        target = measured.copy()
        target[args.joint - 1] += args.delta
        target = robot.clip(target)
        deadline = time.monotonic() + args.seconds
        sequence = 1
        while time.monotonic() < deadline:
            now = time.monotonic()
            driver.set_target_joints(target, source_timestamp=now, sequence=sequence)
            sequence += 1
            time.sleep(0.01)
        driver.hold()
        time.sleep(0.2)
    finally:
        driver.stop()
        driver.disconnect()


if __name__ == "__main__":
    main()
