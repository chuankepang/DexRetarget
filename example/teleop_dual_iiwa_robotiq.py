"""Publish Quest 3 commands for dual iiwa arms and binary Robotiq grippers.

This process deliberately has no KUKA/Robotiq SDK dependency.  It converts
Quest tracking into absolute TCP targets in a shared world frame and in each
robot base frame, then publishes a latest-state JSON contract for a separate
low-level robot process.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
from typing import Any, Optional, TextIO

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from anydexretarget.teleop import (
    BinaryGripperCommand,
    BinaryGripperMapper,
    ReferenceRelativeWorldMapper,
    WorldPoseCommand,
    invert_transform,
    make_transform,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
)


SIDES = ("left", "right")
SCHEMA = "dexretarget.dual_iiwa_robotiq.v2"
GRIPPER_COMMAND_CONTRACT = "binary_position.v1"


def _pose_xyzw_to_matrix(values: list[float], name: str) -> np.ndarray:
    pose = np.asarray(values, dtype=np.float64)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must contain X Y Z QX QY QZ QW")
    return make_transform(pose[:3], quaternion_xyzw_to_matrix(pose[3:]))


def _wrist_to_matrix(wrist: dict[str, Any]) -> np.ndarray:
    return make_transform(wrist["position"], wrist["rotation"])


def _matrix_list(transform: np.ndarray) -> list[float]:
    # Fixed precision keeps a dual-arm UDP datagram comfortably below the
    # usual Ethernet MTU while retaining sub-micrometre numeric resolution.
    return [round(float(value), 7) for value in transform.reshape(-1)]


def _vector_list(vector: np.ndarray) -> list[float]:
    return [round(float(value), 7) for value in vector]


def _load_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"teleoperation config not found: {path}")
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("teleoperation config must be a mapping")
    for section in ("quest", "publisher", "arms", "grippers"):
        if section not in config or not isinstance(config[section], dict):
            raise ValueError(f"missing config mapping: {section}")
    for side in SIDES:
        if side not in config["arms"] or not isinstance(config["arms"][side], dict):
            raise ValueError(f"missing arm configuration: {side}")
    return config


@dataclass
class SideRuntime:
    mapper: ReferenceRelativeWorldMapper
    gripper: BinaryGripperMapper
    base_world: np.ndarray
    base_frame: str
    tcp_frame: str
    last_wrist_timestamp: Optional[float] = None
    last_landmarks_timestamp: Optional[float] = None
    arm_command: Optional[WorldPoseCommand] = None
    gripper_command: Optional[BinaryGripperCommand] = None
    gripper_error: Optional[str] = None


class JsonCommandSink:
    """Send compact JSON through UDP/stdout, with optional JSONL recording."""

    def __init__(
        self,
        mode: str,
        host: str,
        port: int,
        record_path: Optional[Path],
    ) -> None:
        self.mode = mode
        self.address = (host, port)
        self.socket = (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if mode in ("udp", "both")
            else None
        )
        self.record: Optional[TextIO] = None
        if record_path is not None:
            record_path.parent.mkdir(parents=True, exist_ok=True)
            self.record = record_path.open("w", encoding="utf-8")

    def send(self, packet: dict[str, Any]) -> None:
        encoded = json.dumps(packet, separators=(",", ":"), allow_nan=False)
        if self.socket is not None:
            self.socket.sendto(encoded.encode("utf-8"), self.address)
        if self.mode in ("stdout", "both"):
            print(encoded, flush=True)
        if self.record is not None:
            self.record.write(encoded + "\n")
            self.record.flush()

    def close(self) -> None:
        if self.socket is not None:
            self.socket.close()
        if self.record is not None:
            self.record.close()


def _make_runtime(
    side_config: dict[str, Any], gripper_config: dict[str, Any]
) -> SideRuntime:
    # Prefer an explicit homogeneous transform. Keep the older
    # xyz+quaternion form readable for existing calibration files.
    if "world_T_base" in side_config:
        world_base = np.asarray(side_config["world_T_base"], dtype=np.float64)
        if world_base.shape != (4, 4) or not np.all(np.isfinite(world_base)):
            raise ValueError("world_T_base must be a finite 4x4 matrix")
        invert_transform(world_base)  # full rigid-transform validation
    else:
        world_base = _pose_xyzw_to_matrix(
            side_config["world_base_pose_xyzw"], "world_base_pose_xyzw"
        )
    base_tcp_start = _pose_xyzw_to_matrix(
        side_config["base_tcp_reference_pose_xyzw"],
        "base_tcp_reference_pose_xyzw",
    )
    world_tcp_start = world_base @ base_tcp_start
    mapper = ReferenceRelativeWorldMapper(
        quest_to_world_rotation=np.asarray(
            side_config["quest_to_world_rotation"], dtype=np.float64
        ),
        world_tcp_reference=world_tcp_start,
        translation_scale=float(side_config["translation_scale"]),
        rotation_scale=float(side_config["rotation_scale"]),
        low_pass_alpha=float(side_config["low_pass_alpha"]),
        max_translation=float(side_config["max_translation_m"]),
        max_rotation=float(side_config["max_rotation_rad"]),
    )
    return SideRuntime(
        mapper=mapper,
        gripper=BinaryGripperMapper(
            close_threshold=float(gripper_config["close_threshold"]),
            open_threshold=float(gripper_config["open_threshold"]),
        ),
        base_world=invert_transform(world_base),
        base_frame=str(side_config["base_frame"]),
        tcp_frame=str(side_config["tcp_frame"]),
    )


def _update_side(
    runtime: SideRuntime,
    snapshot: dict[str, Any],
    recenter: bool,
    armed: bool,
) -> bool:
    """Consume one snapshot and return whether a pending recenter was applied."""
    recentered = False
    wrist = snapshot["wrist"]
    if wrist is not None:
        timestamp = float(wrist["timestamp"])
        wrist_transform = _wrist_to_matrix(wrist)
        is_new = timestamp != runtime.last_wrist_timestamp

        if not runtime.mapper.referenced:
            runtime.mapper.set_reference(wrist_transform)
            runtime.arm_command = runtime.mapper.update(wrist_transform, timestamp)
        elif recenter:
            # Clutch/re-enable at the latest tracked wrist without changing the
            # held world target.  Subsequent deltas start from this wrist pose.
            runtime.mapper.set_reference(
                wrist_transform, runtime.mapper.last_world_target
            )
            runtime.arm_command = runtime.mapper.update(wrist_transform, timestamp)
            recentered = True
        elif armed and is_new:
            runtime.arm_command = runtime.mapper.update(wrist_transform, timestamp)

        if is_new:
            runtime.last_wrist_timestamp = timestamp

    landmarks_timestamp = snapshot["landmarks_timestamp"]
    fingers = snapshot["fingers"]
    if (
        landmarks_timestamp is not None
        and fingers is not None
        and float(landmarks_timestamp) != runtime.last_landmarks_timestamp
    ):
        try:
            runtime.gripper_command = runtime.gripper.update(fingers)
            runtime.gripper_error = None
        except ValueError as exc:
            runtime.gripper_error = str(exc)
        runtime.last_landmarks_timestamp = float(landmarks_timestamp)
    return recentered


def _side_packet(
    runtime: SideRuntime,
    now: float,
    tracking_timeout: float,
    armed: bool,
) -> tuple[dict[str, Any], dict[str, Any]]:
    wrist_age = (
        None
        if runtime.last_wrist_timestamp is None
        else max(0.0, now - runtime.last_wrist_timestamp)
    )
    arm_fresh = wrist_age is not None and wrist_age <= tracking_timeout
    if runtime.arm_command is None:
        arm_status = "waiting_for_reference"
        world_target = runtime.mapper.last_world_target
        reference_id = runtime.mapper.reference_id
        translation_delta = np.zeros(3)
        rotation_delta = np.zeros(3)
    else:
        arm_status = "active" if arm_fresh else "tracking_timeout"
        world_target = runtime.arm_command.world_target
        reference_id = runtime.arm_command.reference_id
        translation_delta = runtime.arm_command.world_translation_delta
        rotation_delta = runtime.arm_command.world_rotation_delta_vector
    if not armed and arm_status != "waiting_for_reference":
        arm_status = "disarmed"

    base_target = runtime.base_world @ world_target
    arm_packet = {
        "valid": bool(armed and arm_fresh and runtime.arm_command is not None),
        "status": arm_status,
        "reference_id": reference_id,
        "source_age_s": None if wrist_age is None else round(wrist_age, 4),
        "base_frame": runtime.base_frame,
        "tcp_frame": runtime.tcp_frame,
        "base_T_tcp_target": _matrix_list(base_target),
        "world_T_tcp_target": _matrix_list(world_target),
        "world_translation_delta_m": _vector_list(translation_delta),
        "world_rotation_delta_vector_rad": _vector_list(rotation_delta),
    }

    landmark_age = (
        None
        if runtime.last_landmarks_timestamp is None
        else max(0.0, now - runtime.last_landmarks_timestamp)
    )
    gripper_fresh = landmark_age is not None and landmark_age <= tracking_timeout
    if runtime.gripper_command is None:
        gripper_status = (
            "invalid_landmarks" if runtime.gripper_error else "waiting_for_hand"
        )
        closure_score = None
    elif runtime.gripper_error is not None:
        gripper_status = "invalid_landmarks"
        closure_score = runtime.gripper_command.closure_score
    else:
        gripper_status = "active" if gripper_fresh else "tracking_timeout"
        closure_score = runtime.gripper_command.closure_score
    if not armed and gripper_status not in ("waiting_for_hand", "invalid_landmarks"):
        gripper_status = "disarmed"

    gripper_packet = {
        "valid": bool(
            armed
            and gripper_fresh
            and runtime.gripper_command is not None
            and runtime.gripper_error is None
        ),
        "status": gripper_status,
        "source_age_s": None if landmark_age is None else round(landmark_age, 4),
        "command": int(runtime.gripper.command),
        "closure_score": (
            None if closure_score is None else round(float(closure_score), 4)
        ),
    }
    return arm_packet, gripper_packet


def build_packet(
    sequence: int,
    runtimes: dict[str, SideRuntime],
    now: float,
    tracking_timeout: float,
    armed: bool,
    command_contract: str = "absolute_tcp.v1",
) -> dict[str, Any]:
    arms: dict[str, Any] = {}
    grippers: dict[str, Any] = {}
    for side in SIDES:
        arms[side], grippers[side] = _side_packet(
            runtimes[side], now, tracking_timeout, armed
        )
    return {
        "schema": SCHEMA,
        "sequence": sequence,
        "timestamp_unix_ns": time.time_ns(),
        "teleop_enabled": bool(armed),
        "arm_command_contract": command_contract,
        "gripper_command_contract": GRIPPER_COMMAND_CONTRACT,
        "arms": arms,
        "grippers": grippers,
    }


def _summary(
    packet: dict[str, Any],
    print_world_tcp_target: bool = True,
    sides: tuple[str, ...] = SIDES,
    quest_stats: Optional[dict[str, object]] = None,
) -> str:
    fields = ["ENABLED" if packet["teleop_enabled"] else "DISARMED"]
    for side in sides:
        arm = packet["arms"][side]
        grip = packet["grippers"][side]
        target_text = ""
        if print_world_tcp_target:
            if packet.get("arm_command_contract") == "reference_relative_world_delta.v1":
                dp = arm["world_translation_delta_m"]
                dr = arm["world_rotation_delta_vector_rad"]
                target_text = (
                    f" world_dp=[{dp[0]:+.4f},{dp[1]:+.4f},{dp[2]:+.4f}]"
                    f" world_dr=[{dr[0]:+.4f},{dr[1]:+.4f},{dr[2]:+.4f}]"
                )
            else:
                transform = np.asarray(
                    arm["world_T_tcp_target"], dtype=np.float64
                ).reshape(4, 4)
                xyz = transform[:3, 3]
                quat = matrix_to_quaternion_xyzw(transform[:3, :3])
                target_text = (
                    f" world_p=[{xyz[0]:+.4f},{xyz[1]:+.4f},{xyz[2]:+.4f}]"
                    f" world_qxyzw=[{quat[0]:+.4f},{quat[1]:+.4f},"
                    f"{quat[2]:+.4f},{quat[3]:+.4f}]"
                )
        fields.append(
            f"{side}:arm={arm['status']}{target_text} "
            f"grip={grip['command']} score={grip['closure_score']}({grip['status']})"
        )
    if quest_stats is not None:
        fields.append(
            "QuestRX:"
            f"packets={quest_stats['packets']} peer={quest_stats['last_peer']} "
            f"left_wrist={quest_stats['valid_left_wrist']} "
            f"left_landmarks={quest_stats['valid_left_landmarks']} "
            f"right_wrist={quest_stats['valid_right_wrist']} "
            f"right_landmarks={quest_stats['valid_right_landmarks']} "
            f"rejected={int(quest_stats['rejected_lines']) + int(quest_stats['rejected_payloads'])}"
        )
        if quest_stats.get("last_rejected_line"):
            fields.append(f"last_rejected={quest_stats['last_rejected_line']!r}")
    return " | ".join(fields)


def _calibration_errors(config: dict[str, Any]) -> list[str]:
    errors = []
    delta_contract = config["publisher"].get("arm_command_contract") == (
        "reference_relative_world_delta.v1"
    )
    required = [("quest_to_world_rotation_configured", "Quest-to-world rotation")]
    if not delta_contract:
        required.extend(
            [
                ("reference_pose_configured", "base TCP start pose"),
                ("world_base_pose_configured", "world-to-base pose"),
            ]
        )
    for side in SIDES:
        if not bool(config["arms"][side].get("enabled", True)):
            continue
        for flag, description in required:
            if not bool(config["arms"][side].get(flag, False)):
                errors.append(f"{side} {description}")
    return errors


def run(args: argparse.Namespace) -> None:
    from input.quest3 import Quest3

    config_path = args.config
    if not config_path.is_absolute():
        # Accept paths relative to the current shell (natural for an explicit
        # --config) while preserving the example-directory-relative default.
        cwd_candidate = Path.cwd() / config_path
        config_path = (
            cwd_candidate
            if cwd_candidate.is_file()
            else Path(__file__).parent / config_path
        )
    config = _load_config(config_path)
    for side in SIDES:
        base_override = getattr(args, f"{side}_base_tcp_start")
        world_override = getattr(args, f"{side}_world_base")
        rotation_override = getattr(args, f"{side}_quest_to_world_rotation")
        if base_override is not None:
            config["arms"][side]["base_tcp_reference_pose_xyzw"] = base_override
            config["arms"][side]["reference_pose_configured"] = True
        if world_override is not None:
            config["arms"][side]["world_base_pose_xyzw"] = world_override
            config["arms"][side].pop("world_T_base", None)
            config["arms"][side]["world_base_pose_configured"] = True
        if rotation_override is not None:
            config["arms"][side]["quest_to_world_rotation"] = np.asarray(
                rotation_override, dtype=np.float64
            ).reshape(3, 3).tolist()
            config["arms"][side]["quest_to_world_rotation_configured"] = True

    quest_config = config["quest"]
    publisher_config = config["publisher"]
    quest_host = args.quest_host or str(quest_config["host"])
    quest_port = args.quest_port or int(quest_config["port"])
    quest_protocol = args.quest_protocol or str(quest_config["protocol"])
    # New explicit names describe the three-machine topology. Keep host/port
    # as backward-compatible aliases for earlier local-only configurations.
    configured_command_host = publisher_config.get(
        "command_host", publisher_config.get("host", "127.0.0.1")
    )
    configured_command_port = publisher_config.get(
        "command_port", publisher_config.get("port", 10000)
    )
    command_host = args.command_host or str(configured_command_host)
    command_port = args.command_port or int(configured_command_port)
    configured_period = publisher_config.get("command_period_s")
    if args.command_period_s is not None:
        if args.command_period_s <= 0.0:
            raise ValueError("--command-period-s must be positive")
        rate_hz = 1.0 / args.command_period_s
    elif args.rate_hz is not None:
        rate_hz = args.rate_hz
    elif configured_period is not None:
        if float(configured_period) <= 0.0:
            raise ValueError("publisher.command_period_s must be positive")
        rate_hz = 1.0 / float(configured_period)
    else:
        rate_hz = float(publisher_config["rate_hz"])
    timeout = args.tracking_timeout or float(publisher_config["tracking_timeout"])
    print_hz = (
        args.print_hz
        if args.print_hz is not None
        else float(publisher_config["print_hz"])
    )
    print_world_tcp_target = (
        bool(publisher_config.get("print_world_tcp_target", True))
        if args.print_world_tcp_target is None
        else bool(args.print_world_tcp_target)
    )
    armed = (
        bool(publisher_config.get("start_armed", False))
        if args.start_armed is None
        else bool(args.start_armed)
    )
    command_contract = str(
        publisher_config.get("arm_command_contract", "absolute_tcp.v1")
    )

    if not 1 <= quest_port <= 65535 or not 1 <= command_port <= 65535:
        raise ValueError("UDP/TCP ports must be in 1..65535")
    if rate_hz <= 0.0 or timeout <= 0.0 or print_hz < 0.0:
        raise ValueError("rates and timeout must be positive; print rate may be zero")
    calibration_errors = _calibration_errors(config)
    if args.output in ("udp", "both") and calibration_errors:
        raise ValueError(
            "real UDP output blocked because calibration placeholders remain: "
            + ", ".join(calibration_errors)
            + ". Confirm the YAML *_configured flags or supply CLI overrides."
        )

    runtimes = {
        side: _make_runtime(config["arms"][side], config["grippers"])
        for side in SIDES
    }
    active_sides = tuple(
        side for side in SIDES if bool(config["arms"][side].get("enabled", True))
    )
    input_device = Quest3(host=quest_host, port=quest_port, protocol=quest_protocol)
    sink = JsonCommandSink(args.output, command_host, command_port, args.record)
    recenter_pending = {side: False for side in SIDES}
    toggle_requested = False

    def request_recenter(_signum=None, _frame=None) -> None:
        for side in SIDES:
            recenter_pending[side] = True

    def request_toggle(_signum=None, _frame=None) -> None:
        nonlocal toggle_requested
        toggle_requested = True

    if hasattr(signal, "SIGUSR1"):
        signal.signal(signal.SIGUSR1, request_recenter)
    if hasattr(signal, "SIGUSR2"):
        signal.signal(signal.SIGUSR2, request_toggle)

    print("Starting Quest 3 dual-arm command publisher")
    print(f"  Quest input: {quest_protocol} {quest_host}:{quest_port}")
    print(
        f"  Output: {args.output} {command_host}:{command_port} "
        f"every {1.0 / rate_hz:g}s ({rate_hz:g} Hz, latest Quest sample only)"
    )
    print("  Arm contract: absolute world/base TCP targets; world-relative motion")
    print(f"  Robot execution contract: {command_contract}")
    print(
        f"  Gripper contract: {GRIPPER_COMMAND_CONTRACT}, "
        "0=open, 1=close, edge-triggered"
    )
    print(f"  Teleoperation: {'ENABLED' if armed else 'DISARMED'}")
    print(f"  Print world TCP target: {print_world_tcp_target}")
    print("  No KUKA or Robotiq SDK is imported by this process.")
    if hasattr(signal, "SIGUSR1"):
        print(f"  Recenter/clutch: kill -USR1 {os.getpid()}")
    if hasattr(signal, "SIGUSR2"):
        print(f"  Enable/disable:  kill -USR2 {os.getpid()}")

    period = 1.0 / rate_hz
    print_period = float("inf") if print_hz == 0.0 else 1.0 / print_hz
    next_tick = time.monotonic()
    next_print = next_tick
    sequence = 0
    try:
        while True:
            now = time.monotonic()
            if input_device.error is not None:
                raise RuntimeError("Quest receiver stopped") from input_device.error
            if toggle_requested:
                armed = not armed
                toggle_requested = False
                if armed:
                    request_recenter()
                print(f"Teleoperation {'ENABLED' if armed else 'DISARMED'}")

            for side in SIDES:
                snapshot = input_device.get_hand_tracking_data(side)
                applied = _update_side(
                    runtimes[side], snapshot, recenter_pending[side], armed
                )
                if applied:
                    recenter_pending[side] = False

            packet = build_packet(
                sequence, runtimes, now, timeout, armed, command_contract
            )
            sink.send(packet)
            if args.output in ("udp", "summary") and now >= next_print:
                print(
                    _summary(
                        packet,
                        print_world_tcp_target,
                        active_sides,
                        input_device.stats,
                    ),
                    flush=True,
                )
                next_print = now + print_period

            sequence += 1
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopping command publisher...")
    finally:
        input_device.stop()
        sink.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Quest 3 -> dual iiwa world TCP targets + Robotiq 0/1",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--config", type=Path, default=Path("config/iiwa_robotiq_teleop.yaml")
    )
    parser.add_argument("--quest-host", default=None)
    parser.add_argument("--quest-port", type=int, default=None)
    parser.add_argument("--quest-protocol", choices=["udp", "tcp"], default=None)
    parser.add_argument(
        "--output", choices=["udp", "stdout", "both", "summary"], default="udp"
    )
    parser.add_argument("--command-host", default=None)
    parser.add_argument("--command-port", type=int, default=None)
    parser.add_argument("--rate-hz", type=float, default=None)
    parser.add_argument(
        "--command-period-s",
        type=float,
        default=None,
        help="seconds between published robot targets; overrides --rate-hz",
    )
    parser.add_argument("--tracking-timeout", type=float, default=None)
    parser.add_argument("--print-hz", type=float, default=None)
    parser.add_argument(
        "--print-world-tcp-target",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="print expected world-frame TCP xyz and quaternion in status summaries",
    )
    parser.add_argument("--record", type=Path, default=None, metavar="JSONL")
    parser.add_argument(
        "--start-armed",
        action="store_true",
        default=None,
        help="enable output immediately (staged testing should leave this off)",
    )
    for side in SIDES:
        parser.add_argument(
            f"--{side}-base-tcp-start",
            type=float,
            nargs=7,
            metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
            default=None,
            help=f"program-start {side} base_T_tcp pose",
        )
        parser.add_argument(
            f"--{side}-world-base",
            type=float,
            nargs=7,
            metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
            default=None,
            help=f"calibrated {side} world_T_base pose",
        )
        parser.add_argument(
            f"--{side}-quest-to-world-rotation",
            type=float,
            nargs=9,
            metavar=("R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22"),
            default=None,
            help=f"row-major proper rotation from Quest axes to world axes ({side})",
        )
    return parser


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
