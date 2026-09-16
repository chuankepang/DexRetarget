#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math

import numpy as np

from common import (
    add_connection_args,
    add_motion_confirmation_args,
    build_driver,
    confirm_motion,
    format_pose,
    format_vector,
    load_config,
)
from example.output.real.drivers_rokae import plan_init_joint_targets
from anydexretarget.teleop import matrix_to_rotation_vector


# xMate ER3 Pro hardware manual, section 4.2. These are mechanical ranges;
# the controller's configured soft limits can be narrower and remain authoritative.
_ER3_PRO_JOINT_LIMITS = np.deg2rad(
    np.array([170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 360.0])
)


def _pose_xyzw(values: list[float]) -> np.ndarray:
    if len(values) != 7:
        raise ValueError("base_transform.pose_xyzw must contain xyz + xyzw")
    x, y, z, qx, qy, qz, qw = map(float, values)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError("base_transform quaternion is zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [x, y, z]
    return transform


def _angle_degrees(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _preflight(current: np.ndarray, target: np.ndarray, init: dict) -> None:
    if target.shape != (7,):
        raise ValueError("ER3 Pro init must contain seven joint angles")
    margins = _ER3_PRO_JOINT_LIMITS - np.abs(target)
    required_margin = float(init.get("min_joint_limit_margin", math.radians(10.0)))
    if np.any(margins < required_margin):
        axis = int(np.argmin(margins))
        raise RuntimeError(
            f"init J{axis + 1} leaves only {math.degrees(margins[axis]):.1f} deg "
            f"to the manual joint limit; required {math.degrees(required_margin):.1f} deg"
        )
    shoulder_bend = abs(float(target[1]))
    elbow_bend = abs(float(target[3]))
    wrist_bend = abs(float(target[5]))

    def check_bend_escape(index: int, minimum: float, label: str) -> None:
        current_value = float(current[index])
        target_value = float(target[index])
        if abs(target_value) >= minimum:
            return
        # The robot may already start inside the commissioning guard. Permit
        # only a same-side, strictly monotonic escape away from exact zero;
        # never reject the safe escape merely because its first fine segment
        # has not reached the full guard angle yet.
        same_side = (
            abs(current_value) < 1e-9
            or current_value * target_value > 0.0
        )
        moving_away = abs(target_value) > abs(current_value) + 1e-9
        if same_side and moving_away:
            print(
                f"preflight: {label} is inside the software guard but moves "
                f"monotonically away from zero ({current_value:+.4f} -> "
                f"{target_value:+.4f} rad)"
            )
            return
        raise RuntimeError(
            f"init path approaches/crosses the official ER Pro {label}=0 "
            f"singularity ({current_value:+.4f} -> {target_value:+.4f} rad)"
        )

    check_bend_escape(1, float(init.get("min_shoulder_bend", 0.35)), "J2")
    check_bend_escape(3, float(init.get("min_elbow_bend", 0.35)), "J4")
    check_bend_escape(5, float(init.get("min_wrist_bend", 0.35)), "J6")
    delta = np.abs(target - current)
    print(
        "preflight: max_delta=J%d:%.4f rad, min_limit_margin=J%d:%.1f deg, "
        "J2/J4/J6_bend=[%.1f, %.1f, %.1f] deg"
        % (
            int(np.argmax(delta)) + 1,
            float(np.max(delta)),
            int(np.argmin(margins)) + 1,
            math.degrees(float(np.min(margins))),
            math.degrees(shoulder_bend),
            math.degrees(elbow_bend),
            math.degrees(wrist_bend),
        )
    )


def _report_world_alignment(config: dict, base_T_tcp: np.ndarray) -> None:
    world_T_base = _pose_xyzw(config["robot"]["base_transform"]["pose_xyzw"])
    world_T_tcp = world_T_base @ base_T_tcp
    tcp_forward = world_T_tcp[:3, 2]   # configured hand mount: TCP +Z -> fingers
    palm_down = -world_T_tcp[:3, 1]    # configured hand mount: TCP -Y -> palm normal
    forward_error = _angle_degrees(tcp_forward, np.array([1.0, 0.0, 0.0]))
    down_error = _angle_degrees(palm_down, np.array([0.0, 0.0, -1.0]))
    print("final world_T_tcp:\n" + format_pose(world_T_tcp))
    print(
        f"hand alignment: forward(+TCP_Z vs +world_X)={forward_error:.2f} deg, "
        f"palm_down(-TCP_Y vs -world_Z)={down_error:.2f} deg"
    )
    if forward_error > 5.0 or down_error > 5.0:
        print(
            "WARNING: measured TCP is not the expected forward/palm-down orientation; "
            "verify base_transform and the active TCP/tool mounting before teleoperation."
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Move ROKAE to configured teleoperation init")
    add_connection_args(parser)
    add_motion_confirmation_args(parser)
    args = parser.parse_args()
    config = load_config(args.config)
    init = config["robot"]["initial_joint_move"]
    target_values = init.get("joint_position")
    if target_values is None:
        parser.error("initial_joint_move.joint_position is null; run save_current_as_init.py")
    target = np.asarray(target_values, dtype=np.float64)
    driver = build_driver(
        args,
        config,
        nrt_online_speed_scale=float(
            init.get(
                "online_speed_scale",
                config.get("non_realtime", {}).get("online_speed_scale", 1.0),
            )
        ),
    )
    try:
        driver.connect()
        controller_world_T_base = driver.get_controller_base_frame()
        print("controller world_T_base:\n" + format_pose(controller_world_T_base))
        controller_base_angle = float(
            np.linalg.norm(
                matrix_to_rotation_vector(controller_world_T_base[:3, :3])
            )
        )
        if bool(config["robot"].get("require_controller_base_rotation_identity", False)):
            tolerance = float(
                config["robot"].get("controller_base_rotation_tolerance", 0.02)
            )
            if controller_base_angle > tolerance:
                raise RuntimeError(
                    "controller baseFrame rotation is not identity for the upright "
                    f"installation: angle={controller_base_angle:.6f} rad, "
                    f"limit={tolerance:.6f} rad. Correct Robot Assist installation, "
                    "base orientation and load before retrying; no motion was sent."
                )
        current = np.asarray(driver.get_joint_positions(), dtype=np.float64)
        print("current joints:", format_vector(current))
        print("target joints: ", format_vector(target))
        planned_targets = plan_init_joint_targets(
            current,
            target,
            init.get("joint_waypoints"),
            float(init["max_joint_delta"]),
            float(init["max_segment_delta"]),
        )
        cursor = current
        for segment_index, segment_target in enumerate(planned_targets, start=1):
            print(f"checking init segment {segment_index}/{len(planned_targets)}")
            _preflight(cursor, segment_target, init)
            cursor = segment_target
        confirm_motion(
            args,
            f"{len(planned_targets)} controller-planned MoveAbsJ segment(s) to the "
            "upright forward/palm-down init (smooth acceleration/deceleration)",
        )
        print("Sending smooth init MoveAbsJ; progress will print every second...", flush=True)
        driver.move_to_init(
            target.tolist(),
            joint_speed=float(init["joint_speed"]),
            timeout=float(init["timeout"]),
            max_joint_delta=float(init["max_joint_delta"]),
            joint_waypoints=init.get("joint_waypoints"),
            max_segment_delta=float(init["max_segment_delta"]),
            power_on=args.power_on,
        )
        print("final joints:  ", format_vector(driver.get_joint_positions()))
        final_pose = driver.get_tcp_pose()
        print("final base_T_tcp:\n" + format_pose(final_pose))
        _report_world_alignment(config, final_pose)
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
