"""Virtual handled-mug motion in Cartesian space.

The mug is not real, but the flange poses follow one. It starts on its side:
diameter 9 cm, length 7 cm, mouth toward +Y (left). From the tool-down
manipulation init the hand tilts 45 degrees in the YZ plane, so tool +Z
points along +Y and -Z together, and grasps the bottom. Tool +X stays toward
the robot, the same roll as init; flipping that axis makes the MoveL a 180
degree spin. After a 5 cm lift, a 90 degree turn about base +X points the
mouth down. The hand is then 45 degrees from vertical, pointing down and to
the right, not parallel to the table. Place keeps that angle and moves to
another spot, with the mouth 1 cm above the table.
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

from rokae.control.cell import (
    BASE_HEIGHT_M,
    FLANGE_TO_FINGERTIP_M,
    TABLE_HEIGHT_M,
    TABLE_Z_IN_BASE_M,
    TOOL_DOWN_ROTATION,
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

_S = float(np.sqrt(2.0) / 2.0)
# Rx(+45 deg) applied to the tool-down home. Tool +Z = (+Y, -Z) / sqrt(2).
# Tool +X stays at base -X. Pointing it forward instead is a 180 deg MoveL.
_RX_PLUS_45 = np.array(
    [[1.0, 0.0, 0.0], [0.0, _S, -_S], [0.0, _S, _S]],
    dtype=np.float64,
)
GRASP_ROTATION = _RX_PLUS_45 @ TOOL_DOWN_ROTATION
# +90 deg is the wrong sense: Rx(-90 deg) carries the mouth from +Y to -Z.
_RX_MINUS_90 = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)
FLIP_ROTATION = _RX_MINUS_90 @ GRASP_ROTATION
MOUTH_IN_TOOL = GRASP_ROTATION.T @ np.array([0.0, 1.0, 0.0])

MUG_DIAMETER_M = 0.09
MUG_LENGTH_M = 0.07
LIFT_M = 0.05
MOUTH_CLEARANCE_M = 0.01
# Lying body center, mouth toward +Y, grasp on the bottom end.
_CUP_CENTER = np.array([0.52, 0.0, TABLE_Z_IN_BASE_M + 0.5 * MUG_DIAMETER_M])
GRASP_TIP = _CUP_CENTER + np.array([0.0, -0.5 * MUG_LENGTH_M, 0.0])
GRASP_XYZ = GRASP_TIP - FLANGE_TO_FINGERTIP_M * GRASP_ROTATION[:, 2]
LIFT_XYZ = GRASP_XYZ + np.array([0.0, 0.0, LIFT_M])
_LIFT_TIP = GRASP_TIP + np.array([0.0, 0.0, LIFT_M])
FLIP_XYZ = _LIFT_TIP - FLANGE_TO_FINGERTIP_M * FLIP_ROTATION[:, 2]
# Another table spot. Mouth points down, so the fingertips (on the bottom)
# sit one mug-length plus 1 cm above the table.
PLACE_TIP = np.array([0.40, 0.16, TABLE_Z_IN_BASE_M + MUG_LENGTH_M + MOUTH_CLEARANCE_M])
PLACE_XYZ = PLACE_TIP - FLANGE_TO_FINGERTIP_M * FLIP_ROTATION[:, 2]

SINGULAR_GUARD_RAD = 0.35
MIN_CLEARANCE_M = 0.008


def _pose(rotation: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = rotation
    pose[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return pose


def _slerp(start: np.ndarray, goal: np.ndarray, fraction: float) -> np.ndarray:
    relative = start.T @ goal
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-8:
        return start.copy()
    skew = relative - relative.T
    axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-8:
        return start.copy()
    axis /= norm
    theta = angle * float(fraction)
    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    rotation = np.eye(3) + np.sin(theta) * cross + (1.0 - np.cos(theta)) * (cross @ cross)
    return start @ rotation


def _fingertip(rotation: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    return np.asarray(xyz, dtype=np.float64) + rotation[:, 2] * FLANGE_TO_FINGERTIP_M


def _mouth(rotation: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    tip = _fingertip(rotation, xyz)
    return tip + rotation @ (MOUTH_IN_TOOL * MUG_LENGTH_M)


def _clearance(rotation: np.ndarray, xyz: np.ndarray) -> float:
    """Lowest of flange, fingertips and the virtual mouth, above the table."""
    points = (
        np.asarray(xyz, dtype=np.float64),
        _fingertip(rotation, xyz),
        _mouth(rotation, xyz),
    )
    return float(min(point[2] for point in points) - TABLE_Z_IN_BASE_M)


def _segment_min_clearance(
    start_xyz: np.ndarray,
    start_rotation: np.ndarray,
    goal_xyz: np.ndarray,
    goal_rotation: np.ndarray,
    samples: int = 31,
) -> float:
    """Lowest fingertip gap on the straight MoveL blend between two poses."""
    lowest = float("inf")
    for fraction in np.linspace(0.0, 1.0, samples):
        xyz = start_xyz + float(fraction) * (goal_xyz - start_xyz)
        rotation = _slerp(start_rotation, goal_rotation, float(fraction))
        lowest = min(lowest, _clearance(rotation, xyz))
    return lowest


def _solve(
    robot: XmateErPro7,
    xyz: np.ndarray,
    rotation: np.ndarray,
    seed: np.ndarray,
    home: np.ndarray,
) -> np.ndarray:
    joints = np.asarray(seed, dtype=np.float64).copy()
    target = _pose(rotation, xyz)
    for _ in range(500):
        error = robot.local_error(joints, target)
        if np.linalg.norm(error[:3]) < 1.5e-3 and np.linalg.norm(error[3:]) < 3e-3:
            break
        jacobian = robot.jacobian(joints)
        damping = 0.04
        system = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
        delta = jacobian.T @ np.linalg.solve(system, error)
        nullspace = np.eye(robot.dof) - jacobian.T @ np.linalg.solve(system, jacobian)
        delta = delta + nullspace @ (0.15 * (home - joints))
        norm = float(np.linalg.norm(delta))
        if norm > 0.10:
            delta *= 0.10 / norm
        joints = robot.clip(joints + delta)
    reached = robot.fk(joints)
    if np.linalg.norm(reached[:3, 3] - xyz) > 0.008:
        raise RuntimeError(
            f"nominal IK missed {np.array2string(xyz, precision=3)} by "
            f"{np.linalg.norm(reached[:3, 3] - xyz) * 1000.0:.1f} mm"
        )
    if float(reached[:3, 2] @ rotation[:, 2]) < 0.98:
        raise RuntimeError("nominal IK lost the requested flange axis")
    bends = np.abs(joints[np.array([1, 3, 5])])
    if float(np.min(bends)) < SINGULAR_GUARD_RAD:
        raise RuntimeError(
            f"J2/J4/J6 bend {np.rad2deg(float(np.min(bends))):.1f} deg "
            "is inside the straight-arm guard"
        )
    return joints


def _rotation_about_x(angle: float) -> np.ndarray:
    cosine = float(np.cos(angle))
    sine = float(np.sin(angle))
    turn = np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
        dtype=np.float64,
    )
    return turn @ GRASP_ROTATION


def _flip_arc(steps: int = 3) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """Rotate about the lifted bottom so the mouth swings down above the table.

    One MoveL of the flange cuts across that arc and drives the mouth into
    the table. Short steps keep the bottom fixed and only reorient the hand.
    """
    arc = []
    for index in range(1, steps + 1):
        rotation = _rotation_about_x(-0.5 * np.pi * index / steps)
        flange = _LIFT_TIP - FLANGE_TO_FINGERTIP_M * rotation[:, 2]
        arc.append((f"flip{index}", flange, rotation))
    return arc


def _waypoints(init_xyz: np.ndarray) -> list[tuple[str, np.ndarray, np.ndarray]]:
    return [
        ("grasp", GRASP_XYZ, GRASP_ROTATION),
        ("lift", LIFT_XYZ, GRASP_ROTATION),
        *_flip_arc(),
        ("place", PLACE_XYZ, FLIP_ROTATION),
        ("init", init_xyz, TOOL_DOWN_ROTATION),
    ]


def _check_plan(
    init_joints: np.ndarray,
) -> list[tuple[str, np.ndarray, np.ndarray]]:
    robot = XmateErPro7()
    init_pose = robot.fk(init_joints)
    if float(init_pose[:3, 2] @ np.array([0.0, 0.0, -1.0])) < 0.99:
        raise RuntimeError("init flange +Z is not pointing down")
    def _tilt_from_down(rotation: np.ndarray) -> float:
        cosine = float(np.clip(rotation[:, 2] @ np.array([0.0, 0.0, -1.0]), -1.0, 1.0))
        return float(np.arccos(cosine))

    for name, rotation in (("grasp", GRASP_ROTATION), ("place", FLIP_ROTATION)):
        if abs(np.linalg.det(rotation) - 1.0) > 1e-8:
            raise RuntimeError(f"{name} rotation is not a proper rotation")
        tilt = _tilt_from_down(rotation)
        if abs(tilt - np.pi / 4.0) > 0.05:
            raise RuntimeError(f"{name} hand tilt is {np.rad2deg(tilt):.1f} deg, not 45")
    relative = GRASP_ROTATION.T @ FLIP_ROTATION
    turn = float(np.arccos(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)))
    if abs(turn - np.pi / 2.0) > 0.05:
        raise RuntimeError(f"flip is {np.rad2deg(turn):.1f} deg, not 90 deg")
    grasp_from_init = float(
        np.arccos(np.clip((np.trace(TOOL_DOWN_ROTATION.T @ GRASP_ROTATION) - 1.0) * 0.5, -1.0, 1.0))
    )
    if abs(grasp_from_init - np.pi / 4.0) > 0.05:
        raise RuntimeError(
            f"init to grasp is {np.rad2deg(grasp_from_init):.1f} deg; "
            "tool +X must stay toward the robot or MoveL spins 180 deg"
        )
    mouth_place = _mouth(FLIP_ROTATION, PLACE_XYZ)
    if float(mouth_place[2] - TABLE_Z_IN_BASE_M) < MOUTH_CLEARANCE_M - 1e-6:
        raise RuntimeError("place mouth is below the 1 cm table clearance")
    mouth_offset = mouth_place - _fingertip(FLIP_ROTATION, PLACE_XYZ)
    if float(mouth_offset @ np.array([0.0, 0.0, -1.0])) < 0.9 * MUG_LENGTH_M:
        raise RuntimeError("after the flip the mouth does not point down")
    grasp_tip = _fingertip(GRASP_ROTATION, GRASP_XYZ)
    place_tip = _fingertip(FLIP_ROTATION, PLACE_XYZ)
    if np.linalg.norm(place_tip[:2] - grasp_tip[:2]) < 0.08:
        raise RuntimeError("place must be a different table location from the grasp")
    if abs(float(LIFT_XYZ[2] - GRASP_XYZ[2]) - LIFT_M) > 1e-9:
        raise RuntimeError("lift must be 5 cm")

    init_xyz = init_pose[:3, 3].copy()
    waypoints = _waypoints(init_xyz)
    print(
        f"Cell: base {BASE_HEIGHT_M * 100:.1f} cm, table {TABLE_HEIGHT_M * 100:.1f} cm, "
        f"table z in base {TABLE_Z_IN_BASE_M * 1000:.1f} mm"
    )
    print(
        "Manipulation init deg "
        + np.array2string(np.rad2deg(init_joints), precision=1)
        + "  flange "
        + np.array2string(init_xyz, precision=3)
    )
    print(
        f"Lying mug: diameter {MUG_DIAMETER_M * 100:.0f} cm, length {MUG_LENGTH_M * 100:.0f} cm, "
        "mouth toward +Y. Grasp the bottom at 45 deg down-left."
    )
    print(
        "Flip is 90 deg about base +X: mouth turns down, hand points down-right "
        "at 45 deg from vertical. Place keeps that angle at another table spot."
    )
    print(f"{'move':<8} {'flange xyz':<26} {'axis':<16} {'tip clear':>10} {'Jbend':>8}")
    previous_xyz = init_xyz
    previous_rotation = TOOL_DOWN_ROTATION
    previous_joints = init_joints.copy()
    for name, xyz, rotation in waypoints:
        joints = _solve(robot, xyz, rotation, previous_joints, init_joints)
        bend = float(np.rad2deg(np.min(np.abs(joints[np.array([1, 3, 5])]))))
        clearance = _clearance(rotation, xyz)
        blend = _segment_min_clearance(previous_xyz, previous_rotation, xyz, rotation)
        if min(clearance, blend) < MIN_CLEARANCE_M:
            raise RuntimeError(
                f"{name} fingertip clearance {min(clearance, blend) * 1000.0:.1f} mm "
                f"is below {MIN_CLEARANCE_M * 1000.0:.0f} mm"
            )
        axis = rotation[:, 2]
        print(
            f"{name:<8} {np.array2string(xyz, precision=3):<26} "
            f"{np.array2string(axis, precision=2):<16} "
            f"{clearance * 1000.0:7.0f} mm {bend:7.1f} deg"
            f"   blend {blend * 1000.0:.0f} mm"
        )
        previous_xyz = xyz
        previous_rotation = rotation
        previous_joints = joints
    return waypoints


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    parser.add_argument(
        "--cartesian-speed",
        type=int,
        default=100,
        help="MoveL speed for straight segments, 5..1000",
    )
    parser.add_argument(
        "--flip-speed",
        type=int,
        default=40,
        help="Slower MoveL speed for the 90 deg flip, 5..1000",
    )
    parser.add_argument(
        "--dwell",
        type=float,
        default=2.0,
        help="Seconds to hold the place pose before returning to init",
    )
    parser.add_argument(
        "--controller",
        choices=["position", "impedance", "pinocchio", "cartesian"],
        default=None,
        help="Override tasks.cup.controller. Default stays on the position path.",
    )
    args = parser.parse_args()
    if not 5 <= args.cartesian_speed <= 1000 or not 5 <= args.flip_speed <= 1000:
        parser.error("speed settings must be 5..1000")
    config = load_config(args.config)
    init_joints = init_joint_position(config, "manipulation")
    waypoints = _check_plan(init_joints)
    print(
        "Path: init -> grasp the bottom at 45 deg -> lift 5 cm -> "
        "flip mouth down -> place -> dwell -> init"
    )
    if not args.execute:
        print("DRY RUN. Re-run with --execute to move the arm.")
        return
    confirm_motion(
        args,
        "MoveAbsJ to the manipulation init, then MoveL through the cup task",
    )
    from rokae.control.command import PathSegment
    from rokae.control.controllers import (
        apply_realtime_gains,
        create_controller,
        resolve_controller,
    )

    spec = resolve_controller(config, cli=args.controller, task="cup")
    print(f"Controller: {spec.kind} route={spec.route}")
    driver = build_driver(args, config, nrt_online_speed_scale=0.4)
    apply_realtime_gains(driver, spec)
    segments = []
    for name, xyz, rotation in waypoints:
        # grasp and the return are 45 deg tilts. flip steps share the
        # 90 deg turn. A 180 deg reading means tool +X was reversed.
        if name in {"grasp", "init"} or name.startswith("flip"):
            rotation_limit = 1.20
        else:
            rotation_limit = 0.40
        speed = args.flip_speed if name.startswith("flip") else args.cartesian_speed
        translation_limit = 0.50 if name == "init" else 0.40
        segments.append(
            PathSegment(
                name=name,
                pose=_pose(rotation, xyz),
                speed=speed,
                max_translation_delta=translation_limit,
                max_rotation_delta=rotation_limit,
                dwell_s=args.dwell if name == "place" else 0.0,
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
        print("Cup-flip path finished at init.")
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
