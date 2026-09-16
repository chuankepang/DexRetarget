"""Offline Quest/ROKAE timing simulation with jitter, delay, and packet loss."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from anydexretarget.teleop import (
    make_transform,
    rotation_vector_to_matrix,
)
from rokae.python import _rokae_cpp


def simulate(
    quest_hz: float,
    robot_hz: float,
    duration: float,
    jitter_ms: float,
    delay_ms: float,
    loss: float,
    spike_rate: float,
    spike_min_ms: float,
    spike_max_ms: float,
    pause_start: float,
    pause_duration: float,
    seed: int,
) -> tuple[dict, list[list[float | str]]]:
    cfg = _rokae_cpp.RealtimeConfig()
    cfg.translation_cutoff_hz = 8.0
    cfg.rotation_cutoff_hz = 8.0
    cfg.max_translation_speed = 0.15
    cfg.max_angular_speed = 0.60
    cfg.max_translation_acceleration = 0.50
    cfg.max_angular_acceleration = 2.0
    cfg.max_translation_jerk = 4.0
    cfg.max_angular_jerk = 15.0
    cfg.workspace_min = (-1.0, -1.0, -1.0)
    cfg.workspace_max = (1.0, 1.0, 1.0)
    cfg.hold_timeout = 0.15
    cfg.stop_timeout = 0.60
    cfg.max_source_age = max(0.20, (delay_ms + 4 * jitter_ms) / 1000.0)
    generator = _rokae_cpp.RealtimeCore(cfg)
    generator.reset(np.eye(4), 0.0)
    rng = np.random.default_rng(seed)
    arrivals = []
    network_delays = []
    sequence = 0
    for source in np.arange(0.0, duration, 1.0 / quest_hz):
        # Model a blocked Python producer (retargeting, logging, or policy
        # inference). The C++ consumer keeps running and no backlog is replayed.
        if pause_start <= source < pause_start + pause_duration:
            sequence += 1
            continue
        if rng.random() < loss:
            sequence += 1
            continue
        delay = max(0.0, (delay_ms + rng.normal(0.0, jitter_ms)) / 1000.0)
        if rng.random() < spike_rate:
            delay += rng.uniform(spike_min_ms, spike_max_ms) / 1000.0
        arrivals.append((source + delay, sequence, source))
        network_delays.append(delay)
        sequence += 1
    arrivals.sort()

    pending = 0
    rows: list[list[float | str]] = []
    max_v = max_a = max_j = 0.0
    max_step = 0.0
    tracking_errors = []
    watchdog_counts = {"tracking": 0, "holding_stale": 0, "safe_stop": 0}
    last_target_position = np.zeros(3)
    previous_position = np.zeros(3)
    previous_acceleration = np.zeros(3)
    dt = 1.0 / robot_hz
    # Continue past the last packet so both watchdog transitions are visible.
    for index in range(int((duration + cfg.stop_timeout + 0.1) * robot_hz)):
        now = index * dt
        while pending < len(arrivals) and arrivals[pending][0] <= now:
            arrival, seq, source = arrivals[pending]
            raw = make_transform(
                [
                    0.18 * math.sin(1.1 * source),
                    0.10 * math.sin(1.7 * source),
                    0.06 * math.sin(0.8 * source),
                ],
                rotation_vector_to_matrix(
                    np.array([0.2, -0.1, 0.35]) * math.sin(1.3 * source)
                ),
            )
            accepted = generator.publish(raw, source, seq, arrival)
            if accepted:
                last_target_position = raw[:3, 3].copy()
            pending += 1
        pose = generator.step(now)
        diag = generator.diagnostics(now)
        state = str(diag["watchdog_state"])
        if state in watchdog_counts:
            watchdog_counts[state] += 1
        velocity_vector = np.asarray(diag["linear_velocity"])
        acceleration_vector = np.asarray(diag["linear_acceleration"])
        velocity = float(np.linalg.norm(velocity_vector))
        acceleration = float(np.linalg.norm(acceleration_vector))
        jerk = float(np.linalg.norm((acceleration_vector - previous_acceleration) / dt))
        previous_acceleration = acceleration_vector
        max_v, max_a, max_j = (
            max(max_v, velocity),
            max(max_a, acceleration),
            max(max_j, jerk),
        )
        max_step = max(
            max_step, float(np.linalg.norm(pose[:3, 3] - previous_position))
        )
        previous_position = pose[:3, 3].copy()
        if diag["latest_sequence"] >= 0 and diag["watchdog_state"] == "tracking":
            tracking_errors.append(
                float(np.linalg.norm(last_target_position - pose[:3, 3]))
            )
        rows.append(
            [
                now,
                diag["watchdog_state"],
                diag["latest_sequence"],
                diag["latest_command_age"],
                *pose[:3, 3],
                velocity,
                acceleration,
                jerk,
            ]
        )
    stats = generator.diagnostics(rows[-1][0])
    return (
        {
            "quest_hz": quest_hz,
            "robot_hz": robot_hz,
            "accepted": stats["accepted"],
            "rejected": stats["rejected_sequence"]
            + stats["rejected_source_time"]
            + stats["rejected_source_age"],
            "max_v": max_v,
            "max_a": max_a,
            "max_j": max_j,
            "max_step": max_step,
            "mean_delay_ms": (
                1000.0 * float(np.mean(network_delays)) if network_delays else 0.0
            ),
            "rms_tracking_error": (
                float(np.sqrt(np.mean(np.square(tracking_errors))))
                if tracking_errors
                else 0.0
            ),
            "holding_steps": watchdog_counts["holding_stale"],
            "safe_stop_steps": watchdog_counts["safe_stop"],
            "final_state": rows[-1][1],
        },
        rows,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--quest-hz", type=float, nargs="+", default=[60, 72, 90])
    parser.add_argument("--robot-hz", type=float, nargs="+", default=[1000])
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--jitter-ms", type=float, default=12.0)
    parser.add_argument("--delay-ms", type=float, default=25.0)
    parser.add_argument("--loss", type=float, default=0.08)
    parser.add_argument("--spike-rate", type=float, default=0.03)
    parser.add_argument("--spike-min-ms", type=float, default=50.0)
    parser.add_argument("--spike-max-ms", type=float, default=200.0)
    parser.add_argument(
        "--pause-start",
        type=float,
        default=1.0,
        help="Start of a simulated blocked Python producer, seconds",
    )
    parser.add_argument(
        "--pause-duration",
        type=float,
        default=0.25,
        help="Duration of the simulated producer pause, seconds",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--csv", type=Path, default=None)
    args = parser.parse_args()
    if any(value <= 0.0 for value in (*args.quest_hz, *args.robot_hz)):
        parser.error("all Quest/robot frequencies must be positive")
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if not 0.0 <= args.loss <= 1.0 or not 0.0 <= args.spike_rate <= 1.0:
        parser.error("--loss and --spike-rate must be in [0, 1]")
    if args.pause_start < 0.0 or args.pause_duration < 0.0:
        parser.error("--pause-start and --pause-duration must be non-negative")

    selected_rows = None
    for quest_hz in args.quest_hz:
        for robot_hz in args.robot_hz:
            result, rows = simulate(
                quest_hz,
                robot_hz,
                args.duration,
                args.jitter_ms,
                args.delay_ms,
                args.loss,
                args.spike_rate,
                args.spike_min_ms,
                args.spike_max_ms,
                args.pause_start,
                args.pause_duration,
                args.seed,
            )
            selected_rows = rows
            print(
                "Quest={quest_hz:>5.0f}Hz Robot={robot_hz:>5.0f}Hz "
                "accepted={accepted:>4} rejected={rejected:>3} "
                "max_v={max_v:.4f} max_a={max_a:.4f} max_j={max_j:.4f} "
                "max_step={max_step:.6f} mean_delay={mean_delay_ms:.1f}ms "
                "rms_error={rms_tracking_error:.4f}m "
                "hold_steps={holding_steps} stop_steps={safe_stop_steps} "
                "final={final_state}".format(**result)
            )
    if args.csv is not None and selected_rows is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["time", "state", "sequence", "input_age", "x", "y", "z", "v", "a", "j"]
            )
            writer.writerows(selected_rows)
        print(f"Wrote final frequency pair to {args.csv}")


if __name__ == "__main__":
    main()
