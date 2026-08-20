"""Quest 3 teleoperation for an Inspire hand and a ROKAE robot arm.

The Inspire path keeps the proven direct-serial output from ``teleop_real.py``.
The arm path maps reference-relative Quest wrist motion to a ROKAE Cartesian
pose, applies safety limits, and publishes it through a latest-value driver.
Use ``--mock-rokae`` for the complete control path without robot hardware.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from anydexretarget import Retargeter
from anydexretarget.teleop import (
    ArmCommand,
    ArmTeleopController,
    PoseSafetyConfig,
    PoseSafetyLimiter,
    RelativePoseMapper,
    make_transform,
    quaternion_xyzw_to_matrix,
)
from output.real.drivers_inspire import InspireSerialOutput
from output.real.drivers_rokae import (
    MockRokaeDriver,
    RokaeDriverBase,
    RokaeXCoreDriver,
)


def _pose_xyz_quat_to_matrix(pose: list[float], name: str) -> np.ndarray:
    values = np.asarray(pose, dtype=np.float64)
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must be X Y Z QX QY QZ QW")
    return make_transform(values[:3], quaternion_xyzw_to_matrix(values[3:]))


def _wrist_dict_to_matrix(wrist: dict) -> np.ndarray:
    return make_transform(wrist["position"], wrist["rotation"])


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"ROKAE teleoperation config not found: {path}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"ROKAE teleoperation config must be a mapping: {path}")
    return data


def _config_value(argument, section: dict[str, Any], key: str):
    return section[key] if argument is None else argument


def _format_matrix(matrix: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:+.7f}" for value in matrix.reshape(-1)) + "]"


def _record_command(record_file, command: ArmCommand) -> None:
    record_file.write(
        json.dumps(
            {
                "t_monotonic": command.timestamp,
                "status": command.status,
                "vr_pose": command.vr_pose.reshape(-1).tolist(),
                "vr_relative": command.vr_relative.reshape(-1).tolist(),
                "world_target": command.world_target.reshape(-1).tolist(),
                "base_target_raw": command.base_target_raw.reshape(-1).tolist(),
                "base_target_safe": command.base_target_safe.reshape(-1).tolist(),
            },
            separators=(",", ":"),
        )
        + "\n"
    )
    record_file.flush()


def run_teleop(args: argparse.Namespace) -> None:
    """Run hand, arm, or combined teleoperation from one Quest receiver."""
    from input.quest3 import Quest3

    explicit_outputs = args.enable_hand or args.enable_arm
    enable_hand = args.enable_hand if explicit_outputs else True
    enable_arm = args.enable_arm if explicit_outputs else True

    input_device = Quest3(port=args.quest3_port, protocol=args.quest3_protocol)
    hand_output: Optional[InspireSerialOutput] = None
    hand_retargeter: Optional[Retargeter] = None
    joint_names: list[str] = []
    arm_driver: Optional[RokaeDriverBase] = None
    arm_controller: Optional[ArmTeleopController] = None
    robot_initial_pose: Optional[np.ndarray] = None
    record_file = None

    if enable_hand:
        config_path = args.hand_config
        if config_path is None:
            config_path = f"config/{args.optimizer}/quest3/quest3_inspire_hand.yaml"
        config_file = Path(__file__).parent / config_path
        hand_retargeter = Retargeter.from_yaml(str(config_file), args.hand)
        joint_names = hand_retargeter.optimizer.robot.dof_joint_names
        hand_output = InspireSerialOutput(
            port_name=args.inspire_port,
            baudrate=args.inspire_baudrate,
            hand_id=args.inspire_hand_id,
        )

    if enable_arm:
        arm_config_path = Path(args.arm_config)
        if not arm_config_path.is_absolute():
            arm_config_path = Path(__file__).parent / arm_config_path
        config = _load_yaml(arm_config_path)
        robot_config = config["robot"]
        teleop_config = config["teleoperation"]
        safety_config = config["safety"]

        base_config = robot_config["base_transform"]
        world_base_pose = _config_value(
            args.world_base_pose,
            base_config,
            "pose_xyzw",
        )
        T_world_base = _pose_xyz_quat_to_matrix(
            list(world_base_pose), "T_world_base pose"
        )
        axis_values = _config_value(
            args.quest_to_robot_rotation,
            teleop_config,
            "quest_to_robot_rotation",
        )
        quest_to_robot_rotation = np.asarray(axis_values, dtype=np.float64).reshape(
            3, 3
        )
        mapper = RelativePoseMapper(
            quest_to_robot_rotation=quest_to_robot_rotation,
            T_world_base=T_world_base,
            translation_scale=float(
                _config_value(
                    args.arm_translation_scale, teleop_config, "translation_scale"
                )
            ),
            rotation_scale=float(
                _config_value(args.arm_rotation_scale, teleop_config, "rotation_scale")
            ),
        )
        limiter = PoseSafetyLimiter(
            PoseSafetyConfig(
                max_translation_speed=float(
                    _config_value(
                        args.arm_max_translation_speed,
                        safety_config,
                        "max_translation_speed",
                    )
                ),
                max_angular_speed=float(
                    _config_value(
                        args.arm_max_angular_speed,
                        safety_config,
                        "max_angular_speed",
                    )
                ),
                max_translation_delta=float(
                    _config_value(
                        args.arm_max_translation_delta,
                        safety_config,
                        "max_translation_delta",
                    )
                ),
                max_rotation_delta=float(
                    _config_value(
                        args.arm_max_rotation_delta,
                        safety_config,
                        "max_rotation_delta",
                    )
                ),
                low_pass_alpha=float(
                    _config_value(
                        args.arm_low_pass_alpha, safety_config, "low_pass_alpha"
                    )
                ),
                workspace_min=tuple(
                    _config_value(
                        args.arm_workspace_min, safety_config, "workspace_min"
                    )
                ),
                workspace_max=tuple(
                    _config_value(
                        args.arm_workspace_max, safety_config, "workspace_max"
                    )
                ),
            )
        )
        command_timeout = float(
            _config_value(args.arm_tracking_timeout, safety_config, "command_timeout")
        )

        if args.mock_rokae:
            mock_pose = args.robot_initial_pose
            if mock_pose is None:
                mock_pose = config["mock"]["initial_pose_xyzw"]
            robot_initial_pose = _pose_xyz_quat_to_matrix(
                list(mock_pose), "mock ROKAE initial pose"
            )
            arm_driver = MockRokaeDriver(
                initial_pose=robot_initial_pose,
                control_hz=float(robot_config["control_hz"]),
                command_timeout=command_timeout,
            )
        else:
            arm_driver = RokaeXCoreDriver(
                robot_ip=args.rokae_robot_ip or robot_config["ip"],
                local_ip=args.rokae_local_ip or robot_config["local_ip"],
                robot_type=args.rokae_robot_type or robot_config["type"],
                rt_network_tolerance=int(
                    _config_value(
                        args.rokae_rt_network_tolerance,
                        robot_config,
                        "rt_network_tolerance",
                    )
                ),
                bridge_library=args.rokae_bridge,
            )

        arm_driver.connect()
        arm_driver.start(power_on=args.rokae_power_on)
        if robot_initial_pose is None:
            robot_initial_pose = arm_driver.get_tcp_pose()
        arm_controller = ArmTeleopController(
            mapper=mapper,
            limiter=limiter,
            command_timeout=command_timeout,
            # Mock is active by default. Real motion requires explicit consent.
            enabled=args.arm_start_enabled or args.mock_rokae,
        )
        if args.arm_record is not None:
            args.arm_record.parent.mkdir(parents=True, exist_ok=True)
            record_file = args.arm_record.open("w", encoding="utf-8")

    latest_qpos = (
        np.zeros(hand_retargeter.num_joints, dtype=np.float32)
        if hand_retargeter is not None
        else None
    )
    latest_arm_command: Optional[ArmCommand] = None
    qpos_ready = False
    hand_frame_count = 0
    arm_frame_count = 0
    input_thread_error: Optional[Exception] = None
    last_wrist_timestamp: Optional[float] = None
    state_lock = threading.Lock()
    stop_event = threading.Event()
    recenter_event = threading.Event()
    toggle_arm_event = threading.Event()

    def request_recenter(_signum=None, _frame=None) -> None:
        recenter_event.set()

    def request_arm_toggle(_signum=None, _frame=None) -> None:
        toggle_arm_event.set()

    if enable_arm and hasattr(signal, "SIGUSR1"):
        signal.signal(signal.SIGUSR1, request_recenter)
    if enable_arm and hasattr(signal, "SIGUSR2"):
        signal.signal(signal.SIGUSR2, request_arm_toggle)

    def input_thread_fn() -> None:
        nonlocal qpos_ready, hand_frame_count, arm_frame_count
        nonlocal input_thread_error, latest_arm_command, last_wrist_timestamp
        assert not enable_hand or (
            hand_retargeter is not None and latest_qpos is not None
        )
        assert not enable_arm or (
            arm_controller is not None
            and arm_driver is not None
            and robot_initial_pose is not None
        )
        while not stop_event.is_set():
            try:
                if enable_hand:
                    fingers_data = input_device.get_fingers_data()
                    fingers_pose = fingers_data[f"{args.hand}_fingers"]
                    if not np.allclose(fingers_pose, 0.0):
                        qpos = hand_retargeter.retarget(fingers_pose)
                        with state_lock:
                            latest_qpos[:] = qpos
                            qpos_ready = True
                        hand_frame_count += 1

                if not enable_arm:
                    if not enable_hand:
                        time.sleep(0.002)
                    continue

                wrist = input_device.get_wrist_pose(args.hand)
                if wrist is None:
                    time.sleep(0.002)
                    continue
                wrist_timestamp = float(wrist["timestamp"])
                if wrist_timestamp == last_wrist_timestamp:
                    time.sleep(0.001)
                    continue
                last_wrist_timestamp = wrist_timestamp
                T_vr = _wrist_dict_to_matrix(wrist)

                if latest_arm_command is None:
                    command = arm_controller.initialize(
                        T_vr, robot_initial_pose, wrist_timestamp
                    )
                    print(
                        "Arm reference initialized from current Quest and robot poses."
                    )
                else:
                    if recenter_event.is_set():
                        arm_controller.recenter(T_vr)
                        recenter_event.clear()
                        print("Arm reference re-centered without a target jump.")
                    command = arm_controller.update(T_vr, wrist_timestamp)

                arm_driver.set_target_pose(command.base_target_safe)
                with state_lock:
                    latest_arm_command = command
                arm_frame_count += 1
                if record_file is not None:
                    _record_command(record_file, command)
            except Exception as exc:
                input_thread_error = exc
                break

    input_thread = threading.Thread(
        target=input_thread_fn,
        name="quest-teleop-input",
        daemon=True,
    )

    print("Starting Quest 3 teleoperation")
    print(f"  Outputs: hand={enable_hand}, arm={enable_arm}")
    print(f"  Quest receiver: {args.quest3_protocol} 0.0.0.0:{args.quest3_port}")
    if enable_hand:
        print(
            f"  Inspire serial: {args.inspire_port} @ {args.inspire_baudrate} "
            f"(id={args.inspire_hand_id}, command=100 Hz)"
        )
    if enable_arm:
        backend = "mock" if args.mock_rokae else "xCoreSDK realtime"
        print(f"  ROKAE backend: {backend}")
        print(f"  Initial arm state: {arm_controller.status}")
        if hasattr(signal, "SIGUSR1"):
            print(f"  Recenter: kill -USR1 {os.getpid()}")
        if hasattr(signal, "SIGUSR2"):
            print(f"  Toggle arm enable: kill -USR2 {os.getpid()}")
    print("=" * 72)

    input_thread.start()
    control_dt = 0.01
    print_dt = 1.0 / args.arm_print_hz if args.arm_print_hz > 0.0 else float("inf")
    next_print = time.monotonic()
    start_time = time.monotonic()
    hand_command_count = 0
    previous_arm_status: Optional[str] = None

    try:
        while True:
            loop_start = time.monotonic()
            with state_lock:
                qpos_to_send = (
                    latest_qpos.copy()
                    if enable_hand and qpos_ready and latest_qpos is not None
                    else None
                )
                arm_command_to_print = latest_arm_command

            if qpos_to_send is not None:
                assert hand_output is not None
                hand_output.send(qpos_to_send, joint_names)
                hand_command_count += 1

            now = time.monotonic()
            if enable_arm:
                assert arm_controller is not None and arm_driver is not None
                if toggle_arm_event.is_set():
                    arm_controller.set_enabled(not arm_controller.enabled)
                    toggle_arm_event.clear()
                    print(f"Arm enabled: {arm_controller.enabled}")
                polled = arm_controller.poll(now)
                if polled is not None:
                    arm_driver.set_target_pose(polled.base_target_safe)
                    with state_lock:
                        latest_arm_command = polled
                    arm_command_to_print = polled
                    if polled.status != previous_arm_status:
                        print(f"Arm state: {polled.status}")
                        previous_arm_status = polled.status

                if (
                    args.arm_print_hz > 0.0
                    and arm_command_to_print is not None
                    and now >= next_print
                ):
                    target = arm_command_to_print.base_target_safe
                    xyz = target[:3, 3]
                    print(
                        f"ARM_TARGET status={arm_command_to_print.status} "
                        f"xyz=[{xyz[0]:+.5f}, {xyz[1]:+.5f}, {xyz[2]:+.5f}] "
                        f"base_T_tcp={_format_matrix(target)}"
                    )
                    next_print = now + print_dt

            if hand_command_count > 0 and hand_command_count % 500 == 0:
                elapsed = max(now - start_time, 1e-6)
                print(
                    f"Hand command: {hand_command_count / elapsed:.1f} Hz | "
                    f"Hand input: {hand_frame_count / elapsed:.1f} Hz | "
                    f"Wrist input: {arm_frame_count / elapsed:.1f} Hz"
                )

            if input_thread_error is not None and not input_thread.is_alive():
                raise RuntimeError(
                    "Quest input/retarget thread stopped"
                ) from input_thread_error

            sleep_time = control_dt - (time.monotonic() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nStopping controller...")
    finally:
        stop_event.set()
        input_thread.join(timeout=2.0)
        input_device.stop()
        if hand_output is not None:
            hand_output.close()
        if arm_driver is not None:
            try:
                arm_driver.hold()
                arm_driver.stop()
            finally:
                arm_driver.disconnect()
        if record_file is not None:
            record_file.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Quest 3 -> Inspire hand + ROKAE relative Cartesian teleoperation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--enable-hand",
        action="store_true",
        help="Enable Inspire output; if neither output flag is given, enable both",
    )
    parser.add_argument(
        "--enable-arm",
        action="store_true",
        help="Enable ROKAE output; if neither output flag is given, enable both",
    )
    parser.add_argument("--robot", choices=["rokae"], default="rokae")
    parser.add_argument("--hand", choices=["left", "right"], default="right")
    parser.add_argument(
        "--optimizer", choices=["adaptive", "vector"], default="adaptive"
    )
    parser.add_argument("--hand-config", default=None)
    parser.add_argument("--quest3-port", type=int, default=9000)
    parser.add_argument("--quest3-protocol", choices=["udp", "tcp"], default="udp")
    parser.add_argument("--inspire-port", default="/dev/ttyUSB0")
    parser.add_argument("--inspire-baudrate", type=int, default=115200)
    parser.add_argument("--inspire-hand-id", type=int, default=1)

    parser.add_argument("--arm-config", default="config/rokae_teleop.yaml")
    parser.add_argument("--mock-rokae", action="store_true")
    parser.add_argument("--arm-start-enabled", action="store_true")
    parser.add_argument("--rokae-robot-ip", default=None)
    parser.add_argument("--rokae-local-ip", default=None)
    parser.add_argument(
        "--rokae-robot-type",
        choices=["xmate-6", "xmate-er-pro-7", "standard-6"],
        default=None,
    )
    parser.add_argument("--rokae-bridge", type=Path, default=None)
    parser.add_argument("--rokae-rt-network-tolerance", type=int, default=None)
    parser.add_argument(
        "--rokae-power-on",
        action="store_true",
        help="Request motor power-on through xCoreSDK; ignored by mock",
    )
    parser.add_argument(
        "--robot-initial-pose",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        default=None,
        help="Mock-only base_T_tcp pose; real mode always reads the robot",
    )
    parser.add_argument(
        "--world-base-pose",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        default=None,
        help="T_world_base as XYZ + quaternion XYZW",
    )
    parser.add_argument(
        "--quest-to-robot-rotation",
        type=float,
        nargs=9,
        metavar=("R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22"),
        default=None,
    )
    parser.add_argument("--arm-translation-scale", type=float, default=None)
    parser.add_argument("--arm-rotation-scale", type=float, default=None)
    parser.add_argument("--arm-max-translation-speed", type=float, default=None)
    parser.add_argument("--arm-max-angular-speed", type=float, default=None)
    parser.add_argument("--arm-max-translation-delta", type=float, default=None)
    parser.add_argument("--arm-max-rotation-delta", type=float, default=None)
    parser.add_argument("--arm-low-pass-alpha", type=float, default=None)
    parser.add_argument("--arm-workspace-min", type=float, nargs=3, default=None)
    parser.add_argument("--arm-workspace-max", type=float, nargs=3, default=None)
    parser.add_argument("--arm-tracking-timeout", type=float, default=None)
    parser.add_argument("--arm-print-hz", type=float, default=10.0)
    parser.add_argument("--arm-record", type=Path, default=None, metavar="JSONL")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.arm_print_hz < 0.0:
        parser.error("--arm-print-hz cannot be negative")
    if args.rokae_rt_network_tolerance is not None and not (
        0 <= args.rokae_rt_network_tolerance <= 100
    ):
        parser.error("--rokae-rt-network-tolerance must be in [0, 100]")
    run_teleop(args)


if __name__ == "__main__":
    main()
