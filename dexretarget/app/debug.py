"""Optional diagnostics; not part of training storage or the real-time callback."""
from __future__ import annotations
import json
import time
import numpy as np
from anydexretarget.teleop import matrix_to_rotation_vector, ArmCommand
from dexretarget.teleop.devices.rokae import RokaeDriverBase

def _format_matrix(matrix: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:+.7f}" for value in matrix.reshape(-1)) + "]"


def _record_command(
    record_file,
    command: ArmCommand,
    sequence: int,
    arm_driver: RokaeDriverBase,
) -> None:
    measured = arm_driver.get_tcp_pose()
    diagnostics = arm_driver.diagnostics()
    record_file.write(
        json.dumps(
            {
                "t_monotonic": command.timestamp,
                "sequence": sequence,
                "status": command.status,
                "vr_pose": command.vr_pose.reshape(-1).tolist(),
                "vr_relative": command.vr_relative.reshape(-1).tolist(),
                "mapped_relative": command.mapped_relative.reshape(-1).tolist(),
                "world_target": command.world_target.reshape(-1).tolist(),
                "base_target_raw": command.base_target_raw.reshape(-1).tolist(),
                "base_target_safe": command.base_target_safe.reshape(-1).tolist(),
                "base_tcp_measured": measured.reshape(-1).tolist(),
                "driver_watchdog": diagnostics.get("watchdog_state"),
                "driver_command_age": diagnostics.get("latest_command_age"),
            },
            separators=(",", ":"),
        )
        + "\n"
    )
    record_file.flush()



def print_arm_status(command, driver, status):
    raw_target = command.base_target_raw
    target = driver.get_command_pose()
    measured = driver.get_tcp_pose()
    diagnostics = driver.diagnostics()
    xyz = target[:3, 3]
    vr_dp = float(
        np.linalg.norm(command.vr_relative[:3, 3])
    )
    vr_dr_deg = float(
        np.rad2deg(
            np.linalg.norm(
                matrix_to_rotation_vector(
                    command.vr_relative[:3, :3]
                )
            )
        )
    )
    vr_body_dr_deg = np.rad2deg(
        matrix_to_rotation_vector(
            command.vr_relative[:3, :3]
        )
    )
    mapped_world_dr_deg = np.rad2deg(
        matrix_to_rotation_vector(
            command.mapped_relative[:3, :3]
        )
    )
    tracking_dp = float(
        np.linalg.norm(target[:3, 3] - measured[:3, 3])
    )
    tracking_dr_deg = float(
        np.rad2deg(
            np.linalg.norm(
                matrix_to_rotation_vector(
                    measured[:3, :3].T @ target[:3, :3]
                )
            )
        )
    )
    tracking_world_dr_deg = np.rad2deg(
        matrix_to_rotation_vector(
            target[:3, :3] @ measured[:3, :3].T
        )
    )
    measured_joints = diagnostics.get("measured_joint_position", [])
    joints_text = (
        "[" + ",".join(f"{float(value):+.4f}" for value in measured_joints) + "]"
        if measured_joints
        else "n/a"
    )
    elbow_text = (
        f"{float(diagnostics['measured_elbow']):+.4f}/"
        f"{float(diagnostics['commanded_elbow']):+.4f}"
        if diagnostics.get("elbow_lock_enabled")
        else "off"
    )
    print(
        f"ARM_TARGET status={status}/"
        f"{diagnostics['watchdog_state']} "
        f"vr_delta={vr_dp * 1000.0:.2f}mm/{vr_dr_deg:.2f}deg "
        f"vr_body_dr_deg={np.array2string(vr_body_dr_deg, precision=2)} "
        f"mapped_world_dr_deg={np.array2string(mapped_world_dr_deg, precision=2)} "
        f"tracking_error={tracking_dp * 1000.0:.2f}mm/"
        f"{tracking_dr_deg:.2f}deg "
        f"tracking_world_dr_deg={np.array2string(tracking_world_dr_deg, precision=2)} "
        f"q={joints_text} elbow(meas/cmd)={elbow_text} "
        f"xyz=[{xyz[0]:+.5f}, {xyz[1]:+.5f}, {xyz[2]:+.5f}] "
        f"base_T_tcp_cmd={_format_matrix(target)} "
        f"base_T_tcp_raw={_format_matrix(raw_target)}"
    )
