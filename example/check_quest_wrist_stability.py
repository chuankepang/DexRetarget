"""Measure raw HTS wrist jitter/drift without connecting to a robot."""

from __future__ import annotations

import argparse
import time

import numpy as np

from input.quest3 import Quest3
from anydexretarget.teleop import matrix_to_rotation_vector


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q)) if values.size else 0.0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Measure Quest/HTS wrist stability; this never connects to ROKAE"
    )
    parser.add_argument("--protocol", choices=("udp", "tcp"), default="udp")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")

    receiver = Quest3(port=args.port, protocol=args.protocol)
    positions: list[np.ndarray] = []
    rotations: list[np.ndarray] = []
    timestamps: list[float] = []
    last_timestamp = None
    print(
        f"Collecting {args.duration:.1f}s of raw {args.hand} wrist data. "
        "Keep the hand stationary; repeat once with the head stationary and "
        "once while moving only the head."
    )
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            wrist = receiver.get_wrist_pose(args.hand)
            if wrist is None or wrist["timestamp"] == last_timestamp:
                time.sleep(0.001)
                continue
            last_timestamp = wrist["timestamp"]
            positions.append(np.asarray(wrist["position"], dtype=np.float64))
            rotations.append(np.asarray(wrist["rotation"], dtype=np.float64))
            timestamps.append(float(wrist["timestamp"]))
    finally:
        receiver.stop()

    if len(timestamps) < 2:
        raise RuntimeError("fewer than two wrist samples received")
    position = np.stack(positions)
    reference_rotation = rotations[0]
    rotation_delta = np.asarray(
        [
            np.linalg.norm(matrix_to_rotation_vector(r @ reference_rotation.T))
            for r in rotations
        ]
    )
    step_translation = np.linalg.norm(np.diff(position, axis=0), axis=1)
    step_rotation = np.asarray(
        [
            np.linalg.norm(matrix_to_rotation_vector(a.T @ b))
            for a, b in zip(rotations[:-1], rotations[1:])
        ]
    )
    duration = max(timestamps[-1] - timestamps[0], 1e-9)
    centered = position - np.mean(position, axis=0)
    print(f"samples={len(timestamps)} rate={len(timestamps) / duration:.1f} Hz")
    print(
        "position_std_xyz_mm="
        + np.array2string(np.std(centered, axis=0) * 1000.0, precision=3)
    )
    print(
        f"position_drift_first_last_mm="
        f"{np.linalg.norm(position[-1] - position[0]) * 1000.0:.3f}"
    )
    print(
        f"position_step_p95/max_mm={percentile(step_translation, 95) * 1000.0:.3f}/"
        f"{float(np.max(step_translation)) * 1000.0:.3f}"
    )
    print(
        f"rotation_from_first_p95/max_deg="
        f"{np.rad2deg(percentile(rotation_delta, 95)):.3f}/"
        f"{np.rad2deg(float(np.max(rotation_delta))):.3f}"
    )
    print(
        f"rotation_step_p95/max_deg="
        f"{np.rad2deg(percentile(step_rotation, 95)):.3f}/"
        f"{np.rad2deg(float(np.max(step_rotation))):.3f}"
    )


if __name__ == "__main__":
    main()
