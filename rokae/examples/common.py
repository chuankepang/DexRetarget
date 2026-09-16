from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from example.output.real.drivers_rokae import RokaeXCoreDriver
from anydexretarget.teleop import RealtimeCartesianConfig

DEFAULT_CONFIG = PROJECT_ROOT / "rokae" / "config" / "rokae.yaml"


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"invalid YAML mapping: {path}")
    return config


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--robot-ip", default=None)
    parser.add_argument("--local-ip", default=None)
    parser.add_argument(
        "--robot-type",
        choices=("xmate-6", "xmate-er-pro-7", "standard-6"),
        default=None,
    )


def add_motion_confirmation_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--execute", action="store_true", help="Actually send the motion command"
    )
    parser.add_argument(
        "--yes", action="store_true", help="Skip typing MOVE (requires --execute)"
    )
    parser.add_argument(
        "--power-on",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Request power-on automatically (default); use --no-power-on "
            "only when automatic power-on is intentionally disabled"
        ),
    )


def confirm_motion(args: argparse.Namespace, description: str) -> None:
    print(f"Planned motion: {description}")
    if not args.execute:
        raise SystemExit("DRY RUN only. Re-run with --execute after checking the target.")
    if args.yes:
        return
    answer = input("Type MOVE to send this command: ").strip()
    if answer != "MOVE":
        raise SystemExit("Motion cancelled")


def build_driver(
    args: argparse.Namespace,
    config: dict[str, Any],
    *,
    nrt_online_speed_scale: float | None = None,
) -> RokaeXCoreDriver:
    robot = config["robot"]
    safety = config["safety"]
    realtime = RealtimeCartesianConfig(
        control_hz=float(robot["control_hz"]),
        translation_cutoff_hz=float(safety["translation_cutoff_hz"]),
        rotation_cutoff_hz=float(safety["rotation_cutoff_hz"]),
        translation_deadband=float(safety["translation_deadband"]),
        rotation_deadband=float(safety["rotation_deadband"]),
        max_translation_speed=float(safety["max_translation_speed"]),
        max_angular_speed=float(safety["max_angular_speed"]),
        max_translation_acceleration=float(safety["max_translation_acceleration"]),
        max_angular_acceleration=float(safety["max_angular_acceleration"]),
        max_translation_jerk=float(safety["max_translation_jerk"]),
        max_angular_jerk=float(safety["max_angular_jerk"]),
        max_target_translation_delta=float(safety["max_target_translation_delta"]),
        max_target_rotation_delta=float(safety["max_target_rotation_delta"]),
        workspace_min=tuple(safety["workspace_min"]),
        workspace_max=tuple(safety["workspace_max"]),
        hold_timeout=float(safety["hold_timeout"]),
        stop_timeout=float(safety["stop_timeout"]),
        max_source_age=float(safety["max_source_age"]),
        future_tolerance=float(safety["future_tolerance"]),
    )
    return RokaeXCoreDriver(
        robot_ip=args.robot_ip or robot["ip"],
        local_ip=args.local_ip or robot["local_ip"],
        robot_type=args.robot_type or robot["type"],
        rt_network_tolerance=int(robot["rt_network_tolerance"]),
        realtime_config=realtime,
        controller_rate_limit=bool(robot["controller_rate_limit"]),
        controller_filter_cutoff_hz=float(robot["controller_filter_cutoff_hz"]),
        nrt_online_speed_scale=float(
            config.get("non_realtime", {}).get("online_speed_scale", 1.0)
            if nrt_online_speed_scale is None
            else nrt_online_speed_scale
        ),
    )


def format_vector(values: Any) -> str:
    return "[" + ", ".join(f"{float(v):+.8f}" for v in values) + "]"


def format_pose(pose: np.ndarray) -> str:
    return np.array2string(
        np.asarray(pose), precision=7, suppress_small=False, sign="+"
    )
