"""Walk the eight vertices of a box above the table.

Order, viewed from above (+Z looking down), counterclockwise:

  init joints
  -> A -> B -> C -> D -> A
  -> E -> F -> G -> H -> E
  -> init flange pose

A is the top corner nearer the robot and to the robot's right. E is directly
below A. The commanded point is the ER3 Pro flange. The open Inspire
fingertips are 18 cm further along flange +Z, which points down, and stay at
least 10 cm above the table.

Default is a dry run. Motion requires ``--execute`` and typing MOVE.

The downward init is not the mechanical zero. The ER3 Pro manual defines zero
as every joint at 0, which is the straight-arm singularity. These joints are
the elbow-up, tool-down pose computed from the nominal SRS model
(J2=25 deg, J4=70 deg, J6=85 deg, J7=0 deg). J7=0 keeps the wrist at the
middle of its ±360 deg range. Flange +Z is base -Z; flange +X points back
toward the robot.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae.control.cell import (
    BASE_HEIGHT_M,
    FINGERTIP_CLEARANCE_M,
    FLANGE_TO_FINGERTIP_M,
    TABLE_HEIGHT_M,
    TABLE_Z_IN_BASE_M,
    TOOL_DOWN_ROTATION,
    fingertip_position,
    table_clearance,
)
from rokae.control.model import XmateErPro7
from rokae.examples.common import (
    add_connection_args,
    add_motion_confirmation_args,
    build_driver,
    confirm_motion,
    init_joint_position,
    load_config,
)

# Box of flange positions in the base frame, metres.
# X edges are 20 cm (twice the previous 10 cm). Y edges are 30 cm, so the
# horizontal side ratio is X:Y = 2:3. Vertical edges stay 10 cm.
BOX_CENTER_XY = np.array([0.55, 0.0], dtype=np.float64)
BOX_HALF_X_M = 0.10
BOX_HALF_Y_M = 0.15
BOX_TOP_Z_M = 0.40
BOX_BOTTOM_Z_M = 0.30

# Wider than the 20 deg teleop guard. The near corners of this larger box
# bend J2/J4/J6 to about 17 deg, still clear of the straight-arm zero.
SINGULAR_GUARD_RAD = np.deg2rad(15.0)


def _corner(x_sign: float, y_sign: float, z: float) -> np.ndarray:
    return np.array(
        [
            BOX_CENTER_XY[0] + x_sign * BOX_HALF_X_M,
            BOX_CENTER_XY[1] + y_sign * BOX_HALF_Y_M,
            z,
        ],
        dtype=np.float64,
    )


def flange_pose(xyz: np.ndarray) -> np.ndarray:
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = TOOL_DOWN_ROTATION
    pose[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return pose


def box_waypoints(init_joints: np.ndarray) -> list[tuple[str, np.ndarray]]:
    """Flange waypoints after the joint init, including the return to init."""
    top = {
        "A": _corner(-1.0, -1.0, BOX_TOP_Z_M),
        "B": _corner(+1.0, -1.0, BOX_TOP_Z_M),
        "C": _corner(+1.0, +1.0, BOX_TOP_Z_M),
        "D": _corner(-1.0, +1.0, BOX_TOP_Z_M),
    }
    bottom = {
        "E": _corner(-1.0, -1.0, BOX_BOTTOM_Z_M),
        "F": _corner(+1.0, -1.0, BOX_BOTTOM_Z_M),
        "G": _corner(+1.0, +1.0, BOX_BOTTOM_Z_M),
        "H": _corner(-1.0, +1.0, BOX_BOTTOM_Z_M),
    }
    robot = XmateErPro7()
    init_xyz = robot.fk(init_joints)[:3, 3]
    names = ("A", "B", "C", "D", "A", "E", "F", "G", "H", "E", "init")
    points = {**top, **bottom, "init": init_xyz}
    return [(name, points[name]) for name in names]


def _check_plan(
    waypoints: list[tuple[str, np.ndarray]], init_joints: np.ndarray
) -> None:
    robot = XmateErPro7()
    init_pose = robot.fk(init_joints)
    tool_down = float(init_pose[:3, 2] @ np.array([0.0, 0.0, -1.0]))
    if tool_down < 0.99:
        raise RuntimeError("init flange +Z is not pointing down")
    init_clearance = table_clearance(init_pose[:3, 3])
    if init_clearance < FINGERTIP_CLEARANCE_M:
        raise RuntimeError(
            f"init fingertip clearance is only {init_clearance * 1000.0:.1f} mm"
        )
    print(
        f"Cell: base {BASE_HEIGHT_M * 100:.1f} cm above the floor, "
        f"table {TABLE_HEIGHT_M * 100:.1f} cm, "
        f"table z in base {TABLE_Z_IN_BASE_M * 1000:.1f} mm"
    )
    print(
        "Manipulation init deg "
        + np.array2string(np.rad2deg(init_joints), precision=1)
        + "  flange xyz "
        + np.array2string(init_pose[:3, 3], precision=3)
    )
    print(
        f"Init fingertip clearance {init_clearance * 1000.0:.0f} mm "
        f"(hand extends {FLANGE_TO_FINGERTIP_M * 100:.0f} cm below the flange)"
    )
    print("Waypoint         flange xyz m              fingertip z  clearance")
    previous = init_joints.copy()
    for name, xyz in waypoints:
        clearance = table_clearance(xyz)
        if clearance < FINGERTIP_CLEARANCE_M:
            raise RuntimeError(
                f"{name} fingertip clearance {clearance * 1000.0:.1f} mm "
                f"is below {FINGERTIP_CLEARANCE_M * 1000.0:.0f} mm"
            )
        solved = _solve_tool_down(robot, xyz, previous, init_joints)
        bends = np.abs(solved[np.array([1, 3, 5])])
        if float(np.min(bends)) < SINGULAR_GUARD_RAD:
            raise RuntimeError(
                f"{name} comes within {np.rad2deg(float(np.min(bends))):.1f} deg "
                "of a straight J2/J4/J6 singularity"
            )
        previous = solved
        tip = fingertip_position(xyz)
        print(
            f"  {name:<6} {np.array2string(xyz, precision=3, suppress_small=True):<28}"
            f" {tip[2]:+.3f} m    {clearance * 1000.0:6.1f} mm"
        )


def _solve_tool_down(
    robot: XmateErPro7,
    xyz: np.ndarray,
    seed: np.ndarray,
    home: np.ndarray,
) -> np.ndarray:
    """Nominal IK used only to reject a corner that would straighten the arm."""
    joints = np.asarray(seed, dtype=np.float64).copy()
    target = flange_pose(xyz)
    for _ in range(300):
        error = robot.local_error(joints, target)
        if np.linalg.norm(error[:3]) < 5e-4 and np.linalg.norm(error[3:]) < 2e-3:
            break
        jacobian = robot.jacobian(joints)
        damping = 0.03
        system = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
        delta = jacobian.T @ np.linalg.solve(system, error)
        nullspace = np.eye(robot.dof) - jacobian.T @ np.linalg.solve(system, jacobian)
        delta = delta + nullspace @ (0.2 * (home - joints))
        norm = float(np.linalg.norm(delta))
        if norm > 0.08:
            delta *= 0.08 / norm
        joints = robot.clip(joints + delta)
    reached = robot.fk(joints)
    if np.linalg.norm(reached[:3, 3] - xyz) > 0.005:
        raise RuntimeError(f"nominal IK missed {xyz.tolist()}")
    if float(reached[:3, 2] @ np.array([0.0, 0.0, -1.0])) < 0.98:
        raise RuntimeError("nominal IK lost the downward flange axis")
    return joints


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    parser.add_argument(
        "--controller",
        choices=["position", "impedance", "pinocchio", "cartesian"],
        default=None,
        help="Override tasks.box.controller. Default stays on the position path.",
    )
    parser.add_argument(
        "--cartesian-speed",
        type=int,
        default=100,
        help="MoveL speed setting, 5..1000. 100 is 2.5 times the previous test speed.",
    )
    args = parser.parse_args()
    if not 5 <= args.cartesian_speed <= 1000:
        parser.error("--cartesian-speed must be 5..1000")
    config = load_config(args.config)
    init_joints = init_joint_position(config, "manipulation")
    waypoints = box_waypoints(init_joints)
    _check_plan(waypoints, init_joints)
    print(
        "Path: init -> A -> B -> C -> D -> A -> E -> F -> G -> H -> E -> init"
    )
    print("Top loop and bottom loop are both counterclockwise when viewed from above.")
    if not args.execute:
        print("DRY RUN. Re-run with --execute to move the arm.")
        return
    confirm_motion(
        args,
        "MoveAbsJ to the manipulation init, then MoveL around the box and back",
    )
    from rokae.control.command import PathSegment
    from rokae.control.controllers import (
        apply_realtime_gains,
        create_controller,
        resolve_controller,
    )

    spec = resolve_controller(config, cli=args.controller, task="box")
    print(f"Controller: {spec.kind} route={spec.route}")
    driver = build_driver(args, config, nrt_online_speed_scale=0.4)
    apply_realtime_gains(driver, spec)
    segments = []
    for name, xyz in waypoints:
        # Y edges are 30 cm, so the one-shot translation guard must clear that.
        limits = (0.45, 1.2) if name in {"A", "init"} else (0.40, 0.50)
        segments.append(
            PathSegment(
                name=name,
                pose=flange_pose(xyz),
                speed=args.cartesian_speed,
                max_translation_delta=limits[0],
                max_rotation_delta=limits[1],
            )
        )
    try:
        driver.connect()
        print("Moving to the manipulation init with MoveAbsJ.")
        driver.move_to_init(
            init_joints.tolist(),
            joint_speed=0.20,
            timeout=90.0,
            max_segment_delta=0.30,
            power_on=args.power_on,
        )
        create_controller(spec, driver).follow(segments, power_on=args.power_on)
        print("Box path finished at the downward init pose.")
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
