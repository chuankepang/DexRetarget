"""Track a Cartesian waypoint with Pinocchio differential IK and a 1 kHz joint stream.

The default run is simulated: IK updates at 50 Hz and the C++ joint core runs
at 1 kHz. ``--execute`` is the same loop on the robot, for a small translation
only, after the controller-FK check.
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

from rokae.control.differential_ik import DifferentialIK
from rokae.control.model import XmateErPro7, rotation_angle
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


def simulate(translation: np.ndarray, seconds: float) -> tuple[float, float]:
    robot = XmateErPro7()
    start = robot.neutral.copy()
    goal = robot.fk(start)
    goal[:3, 3] += translation
    ik = DifferentialIK(robot, start)
    core = _rokae_cpp.JointRealtimeCore(
        _cpp_config(JointStreamConfig.from_limits(robot.stream_limits))
    )
    core.reset(start.tolist(), 0.0)
    previous = start.copy()
    peak = 0.0
    steps = int(seconds * 1000)
    for index in range(1, steps + 1):
        timestamp = index * 0.001
        if index % 20 == 0:
            fraction = min(1.0, timestamp / max(seconds * 0.7, 1e-3))
            target = robot.fk(start)
            target[:3, 3] += translation * fraction
            joints = ik.step(target, 0.02)
            if not core.publish(joints.tolist(), timestamp, index, timestamp):
                ik.reset(np.asarray(core.command_joints(), dtype=np.float64))
        command = np.asarray(core.step(timestamp), dtype=np.float64)
        peak = max(peak, float(np.max(np.abs(command - previous))))
        previous = command
    reached = robot.fk(previous)
    translation_error = float(np.linalg.norm(reached[:3, 3] - goal[:3, 3]))
    rotation_error = rotation_angle(reached[:3, :3].T @ goal[:3, :3])
    print(
        f"sim peak_step={peak:.6f} rad translation_error={translation_error * 1000.0:.2f} mm "
        f"rotation_error={np.rad2deg(rotation_error):.2f} deg "
        f"sigma_min={robot.smallest_singular_value(previous):.4f}"
    )
    limit = float(np.max(robot.max_velocity)) * 0.001 * 1.05
    if peak > limit:
        raise RuntimeError(f"joint step {peak:.6f} rad exceeds {limit:.6f} rad")
    return translation_error, rotation_error


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    parser.add_argument(
        "--translation",
        type=float,
        nargs=3,
        default=[0.03, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()
    translation = np.asarray(args.translation, dtype=np.float64)
    translation_error, _rotation_error = simulate(translation, args.seconds)
    if translation_error > 0.008:
        raise RuntimeError(
            f"simulation did not reach the waypoint ({translation_error * 1000.0:.1f} mm)"
        )
    if not args.execute:
        print("SIMULATION only. Re-run with --execute for a real joint stream.")
        return
    confirm_motion(args, "Pinocchio IK joint stream of the printed translation")
    config = load_config(args.config)
    robot = XmateErPro7()
    driver = build_driver(args, config)
    try:
        driver.connect()
        measured = driver.get_joint_positions()
        if measured is None:
            raise RuntimeError("robot did not return joints")
        robot.require_controller_match(measured, driver.get_tcp_pose())
        ik = DifferentialIK(robot, measured)
        driver.start(power_on=args.power_on, control_mode="pinocchio", kinematics=robot)
        origin = robot.fk(driver.get_command_joints())
        deadline = time.monotonic() + args.seconds
        sequence = 1
        previous_time = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            fraction = min(1.0, 1.0 - (deadline - now) / args.seconds)
            target = origin.copy()
            target[:3, 3] += translation * fraction
            joints = ik.step(target, max(1e-3, now - previous_time))
            previous_time = now
            if not driver.set_target_joints(joints, source_timestamp=now, sequence=sequence):
                ik.reset(driver.get_command_joints())
            sequence += 1
            time.sleep(0.02)
        driver.hold()
        time.sleep(0.3)
    finally:
        driver.stop()
        driver.disconnect()


if __name__ == "__main__":
    main()
