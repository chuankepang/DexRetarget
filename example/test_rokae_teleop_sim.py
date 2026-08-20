"""Hardware-free numerical demo of the complete Quest-to-ROKAE pose path."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from anydexretarget.teleop import (
    ArmTeleopController,
    PoseSafetyConfig,
    PoseSafetyLimiter,
    RelativePoseMapper,
    make_transform,
)
from anydexretarget.teleop.pose import rotation_vector_to_matrix
from output.real.drivers_rokae import MockRokaeDriver


def mounting_transform(name: str) -> np.ndarray:
    if name == "normal":
        return np.eye(4)
    if name == "upside-down":
        return make_transform(
            [0.2, -0.1, 1.0],
            rotation_vector_to_matrix(np.array([np.pi, 0.0, 0.0])),
        )
    if name == "tilted":
        yaw = rotation_vector_to_matrix(np.array([0.0, 0.0, np.deg2rad(30.0)]))
        pitch = rotation_vector_to_matrix(np.array([0.0, np.deg2rad(20.0), 0.0]))
        return make_transform([0.3, -0.2, 0.6], yaw @ pitch)
    raise ValueError(name)


def quest_trajectory(t: float) -> np.ndarray:
    position = np.zeros(3)
    rotation = np.eye(3)
    if t < 2.0:
        position[0] = 0.10 * (t / 2.0)
    elif t < 4.0:
        position[:] = [0.10, 0.08 * ((t - 2.0) / 2.0), 0.0]
    elif t < 6.0:
        position[:] = [0.10, 0.08, 0.0]
        rotation = rotation_vector_to_matrix(
            np.array([0.0, 0.0, np.deg2rad(45.0) * ((t - 4.0) / 2.0)])
        )
    else:
        phase = (t - 6.0) / 2.0
        position[:] = [0.10 + 0.04 * phase, 0.08 - 0.03 * phase, 0.05 * phase]
        rotation = rotation_vector_to_matrix(
            np.array([np.deg2rad(20.0) * phase, 0.0, np.deg2rad(45.0)])
        )
    return make_transform(position, rotation)


def run(base_name: str, csv_path: Path | None, dt: float) -> None:
    T_world_base = mounting_transform(base_name)
    initial_robot_pose = make_transform([0.45, 0.0, 0.35])
    mapper = RelativePoseMapper(
        quest_to_robot_rotation=np.eye(3),
        T_world_base=T_world_base,
        translation_scale=0.5,
        rotation_scale=0.7,
    )
    limiter = PoseSafetyLimiter(
        PoseSafetyConfig(
            max_translation_speed=0.20,
            max_angular_speed=0.80,
            max_translation_delta=0.005,
            max_rotation_delta=0.02,
            low_pass_alpha=0.25,
            workspace_min=(-1.2, -1.2, -0.1),
            workspace_max=(1.2, 1.2, 1.5),
        )
    )
    controller = ArmTeleopController(mapper, limiter, command_timeout=0.20)
    driver = MockRokaeDriver(initial_robot_pose, control_hz=1000.0)
    driver.connect()
    driver.start()

    csv_file = None
    writer = None
    if csv_path is not None:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "t",
                "vr_x",
                "vr_y",
                "vr_z",
                "rel_x",
                "rel_y",
                "rel_z",
                "world_x",
                "world_y",
                "world_z",
                "base_raw_x",
                "base_raw_y",
                "base_raw_z",
                "base_safe_x",
                "base_safe_y",
                "base_safe_z",
            ]
        )

    try:
        reference = quest_trajectory(0.0)
        command = controller.initialize(reference, driver.get_tcp_pose(), 0.0)
        driver.set_target_pose(command.base_target_safe)
        steps = int(round(8.0 / dt)) + 1
        print(f"Simulation base={base_name}, steps={steps}, dt={dt:.3f}s")
        print("Columns: VR | VR-relative | world target | base raw | filtered/limited")
        for index in range(steps):
            timestamp = index * dt
            vr_pose = quest_trajectory(timestamp)
            command = controller.update(vr_pose, timestamp)
            driver.set_target_pose(command.base_target_safe)
            values = [
                timestamp,
                *vr_pose[:3, 3],
                *command.vr_relative[:3, 3],
                *command.world_target[:3, 3],
                *command.base_target_raw[:3, 3],
                *command.base_target_safe[:3, 3],
            ]
            if writer is not None:
                writer.writerow(values)
            if index % max(1, int(round(0.5 / dt))) == 0:
                groups = [
                    values[1:4],
                    values[4:7],
                    values[7:10],
                    values[10:13],
                    values[13:16],
                ]
                print(
                    f"t={timestamp:4.1f} "
                    + " | ".join(
                        "[" + ", ".join(f"{value:+.4f}" for value in group) + "]"
                        for group in groups
                    )
                )
        print("Simulation completed successfully.")
    finally:
        if csv_file is not None:
            csv_file.close()
        driver.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base",
        choices=["normal", "upside-down", "tilted"],
        default="normal",
    )
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--dt", type=float, default=0.02)
    args = parser.parse_args()
    if args.dt <= 0.0:
        parser.error("--dt must be positive")
    run(args.base, args.csv, args.dt)


if __name__ == "__main__":
    main()
