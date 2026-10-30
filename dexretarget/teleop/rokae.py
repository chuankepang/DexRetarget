"""Construct/start the existing ROKAE control route; no recording or camera imports."""
from __future__ import annotations
import os
import sys
import time
from pathlib import Path
from typing import Any, Optional
from dataclasses import dataclass
import numpy as np
from dexretarget.config import PROJECT_ROOT, load_yaml_mapping
from anydexretarget.teleop import (
    ArmTeleopController, PoseSafetyConfig, PoseSafetyLimiter,
    RelativePoseMapper, make_transform, matrix_to_rotation_vector, quaternion_xyzw_to_matrix,
)
from .devices.rokae import MockRokaeDriver, RokaeXCoreDriver
from rokae.control.configuration import configured_realtime

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
    return load_yaml_mapping(path)


def _config_value(argument, section: dict[str, Any], key: str):
    return section[key] if argument is None else argument


def _wait_for_fresh_wrist(
    input_device,
    hand: str,
    timeout: float,
    *,
    newer_than: Optional[float] = None,
    max_age: float = 0.20,
) -> dict:
    """Wait for a recent wrist, optionally requiring a post-barrier packet."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        wrist = input_device.get_wrist_pose(hand)
        if wrist is not None:
            timestamp = float(wrist["timestamp"])
            if (
                time.monotonic() - timestamp < max_age
                and (newer_than is None or timestamp > newer_than)
            ):
                return wrist
        if input_device.error is not None:
            raise RuntimeError("Quest receiver failed") from input_device.error
        time.sleep(0.01)
    raise TimeoutError(
        f"no fresh {hand} wrist pose received within {timeout:.1f}s"
        + (
            ""
            if newer_than is None
            else " after the ROKAE realtime-start barrier"
        )
    )


def _warn_if_non_realtime_kernel() -> None:
    """Warn when the host is not running a PREEMPT_RT kernel."""
    realtime_flag = Path("/sys/kernel/realtime")
    try:
        if realtime_flag.is_file() and realtime_flag.read_text().strip() == "1":
            return
    except OSError:
        pass
    if "PREEMPT_RT" in os.uname().version.upper():
        return
    print(
        "WARNING: this host is not running a PREEMPT_RT kernel. ROKAE requires "
        "a stable 1 ms realtime callback; validate with the 2 mm realtime test "
        "before enabling Quest teleoperation."
    )




@dataclass(frozen=True)
class ArmComponents:
    arm_driver: Any
    arm_execution: Any
    arm_controller: Any
    robot_initial_pose: Any
    latest_arm_command: Any
    arm_sequence: Any
    arm_control_mode: Any
    config: Any
    init_config: Any
    realtime_config: Any
    mapper: Any
    T_world_base: Any
    quest_to_robot_rotation: Any
    controller_spec: Any


def create_arm(args, home_config, input_device, hand_output=None):
    arm_model = None
    robot_initial_pose = None
    arm_sequence = 0
    if not args.mock_rokae:
        _warn_if_non_realtime_kernel()
    arm_config_path = Path(args.arm_config)
    if not arm_config_path.is_absolute():
        arm_config_path = PROJECT_ROOT / arm_config_path
    config = _load_yaml(arm_config_path)
    robot_config = config["robot"]
    teleop_config = config["teleoperation"]
    # 在连接/上电前验证恢复等待参数；此状态机不执行等待或设备操作。
    from .tracking import TrackingHold
    TrackingHold(teleop_config.get("tracking_resume_delay_s", 0.01))
    safety_config = config["safety"]
    init_config = dict(robot_config["initial_joint_move"])
    if home_config:
        init_config.update(
            enabled=home_config["on_start"], joint_position=home_config["arm_joints"],
            joint_speed=home_config["joint_speed"], timeout=home_config["timeout_s"],
            max_segment_delta=home_config["max_segment_delta"],
        )
    from rokae.control.controllers import resolve_controller

    controller_spec = resolve_controller(config, cli=args.arm_control)
    arm_control_mode = controller_spec.route
    if arm_control_mode not in {"pinocchio", "cartesian", "impedance"}:
        raise ValueError("arm control mode must be pinocchio, cartesian, or impedance")
    robot_type = args.rokae_robot_type or robot_config["type"]
    if arm_control_mode == "pinocchio" and robot_type != "xmate-er-pro-7":
        raise RuntimeError(
            "The Pinocchio joint route is implemented for xmate-er-pro-7. "
            "Use --arm-control cartesian for the other robot types."
        )
    if arm_control_mode == "pinocchio":
        from rokae.control.model import XmateErPro7

        arm_model = XmateErPro7()

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
    mapping_options = {
        "rotation_mapping": teleop_config.get("rotation_mapping", "spatial"),
        "wrist_to_tool_rotation": np.asarray(teleop_config.get("wrist_to_tool_rotation", np.eye(3)), dtype=float).reshape(3, 3),
    }
    mapper = RelativePoseMapper(
        **mapping_options,
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
    print(f"ROKAE rotation mapping={mapper.rotation_mapping}, scale={mapper.rotation_scale:g}; "
          f"wrist_to_tool={mapper.wrist_to_tool_rotation.tolist()}")
    print(
        "Configured T_world_base (world <- robot base):\n"
        + np.array2string(T_world_base, precision=7, suppress_small=False, sign="+")
    )
    if np.allclose(T_world_base[:3, :3], np.eye(3), atol=1e-9):
        print(
            "ROKAE mounting: UPRIGHT, base/world axes are aligned; "
            "world-base translation="
            f"{T_world_base[:3, 3].tolist()} m"
        )
    else:
        print("WARNING: non-identity world-from-base rotation is active")
    # ArmTeleopController owns reference/clutch state.  The actual limits
    # run in CartesianSetpointGenerator at the robot's fixed callback rate.
    limiter = PoseSafetyLimiter(
        PoseSafetyConfig(
            max_translation_speed=1e6,
            max_angular_speed=1e6,
            max_translation_delta=1e6,
            max_rotation_delta=1e6,
            low_pass_alpha=1.0,
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
    from rokae.control.controllers.spec import effective_cartesian_safety

    safety_config = effective_cartesian_safety(config, controller_spec)
    realtime_config = configured_realtime(args, robot_config, safety_config)

    if args.mock_rokae:
        if arm_control_mode == "pinocchio":
            initial_joints = np.asarray(
                init_config["joint_position"], dtype=np.float64
            )
            robot_initial_pose = arm_model.fk(initial_joints)
            arm_driver = MockRokaeDriver(
                initial_pose=robot_initial_pose,
                realtime_config=realtime_config,
                control_mode="pinocchio",
                initial_joints=initial_joints,
                kinematics=arm_model,
            )
        else:
            mock_pose = args.robot_initial_pose
            if mock_pose is None:
                mock_pose = config["mock"]["initial_pose_xyzw"]
            robot_initial_pose = _pose_xyz_quat_to_matrix(
                list(mock_pose), "mock ROKAE initial pose"
            )
            arm_driver = MockRokaeDriver(
                initial_pose=robot_initial_pose,
                realtime_config=realtime_config,
                control_mode=(
                    "impedance" if arm_control_mode == "impedance" else "cartesian"
                ),
            )
    else:
        arm_driver = RokaeXCoreDriver(
            robot_ip=args.rokae_robot_ip or robot_config["ip"],
            local_ip=args.rokae_local_ip or robot_config["local_ip"],
            robot_type=robot_type,
            rt_network_tolerance=int(
                _config_value(
                    args.rokae_rt_network_tolerance,
                    robot_config,
                    "rt_network_tolerance",
                )
            ),
            realtime_config=realtime_config,
            sdk_path=args.rokae_sdk_path,
            controller_rate_limit=bool(robot_config["controller_rate_limit"]),
            controller_filter_cutoff_hz=float(
                robot_config["controller_filter_cutoff_hz"]
            ),
            nrt_online_speed_scale=float(
                init_config.get(
                    "online_speed_scale",
                    config.get("non_realtime", {}).get("online_speed_scale", 1.0),
                )
            ),
        )

    try:
        if not args.mock_rokae:
            from rokae.control.controllers import apply_realtime_gains

            apply_realtime_gains(arm_driver, controller_spec)
        arm_driver.connect()
        controller_world_T_base = arm_driver.get_controller_base_frame()
        controller_base_angle = float(
            np.linalg.norm(
                matrix_to_rotation_vector(controller_world_T_base[:3, :3])
            )
        )
        print(
            "Controller baseFrame (controller world <- base):\n"
            + np.array2string(
                controller_world_T_base,
                precision=7,
                suppress_small=False,
                sign="+",
            )
        )
        if bool(robot_config.get("require_controller_base_rotation_identity", False)):
            tolerance = float(
                robot_config.get("controller_base_rotation_tolerance", 0.02)
            )
            if controller_base_angle > tolerance:
                raise RuntimeError(
                    "Controller baseFrame rotation is not identity for the current "
                    f"upright installation: angle={controller_base_angle:.6f} rad, "
                    f"limit={tolerance:.6f} rad. Correct the installation/base "
                    "orientation and load in Robot Assist before moving."
                )
        if not args.skip_init and (
            args.rokae_move_to_init or bool(init_config["enabled"])
        ):
            startup_targets = (home_config["joint_waypoints"] if home_config else []) + [init_config["joint_position"]]
            for target in startup_targets:
                arm_driver.move_to_init(
                    target, joint_speed=float(init_config["joint_speed"]),
                    timeout=float(init_config["timeout"]),
                    max_segment_delta=init_config.get("max_segment_delta"),
                    power_on=args.rokae_power_on,
                )
            if home_config:
                measured_home = arm_driver.get_joint_positions()
                if measured_home is None or not np.all(np.isfinite(measured_home)) or np.max(np.abs(np.asarray(measured_home) - home_config["arm_joints"])) > home_config["joint_tolerance_rad"]:
                    raise RuntimeError("Startup home feedback is outside configured tolerance")
            if home_config and home_config.get("hand_angles") is not None and hand_output:
                hand_output.send_angles(home_config["hand_angles"])
        from rokae.control.controllers import create_controller

        arm_execution = create_controller(
            controller_spec,
            arm_driver,
            kinematics=arm_model,
            initial_joints=(
                None
                if arm_model is None
                else np.asarray(init_config["joint_position"], dtype=np.float64)
            ),
            mock=bool(args.mock_rokae),
        )
        if robot_initial_pose is None:
            robot_initial_pose = arm_driver.get_tcp_pose()
        print(
            "Waiting for a fresh Quest wrist before starting ROKAE realtime mode..."
        )
        prestart_wrist = _wait_for_fresh_wrist(
            input_device,
            args.hand,
            args.quest_reference_timeout,
            max_age=realtime_config.max_source_age,
        )
        arm_controller = ArmTeleopController(
            mapper=mapper,
            limiter=limiter,
            command_timeout=realtime_config.hold_timeout,
            # Both mock and real start paused unless explicitly requested.
            enabled=args.arm_start_enabled,
            apply_safety=False,
        )
        arm_execution.start(power_on=args.rokae_power_on)
        # Mode switching and state-receiver startup can exceed
        # max_source_age.  Hold the measured startup pose, discard the
        # pre-start wrist, and establish the reference from the first
        # packet that arrived after realtime mode was fully active.
        arm_driver.hold()
        realtime_start_barrier = time.monotonic()
        print(
            "ROKAE realtime mode is HOLDING; waiting for a post-start "
            "fresh Quest wrist..."
        )
        wrist_reference = _wait_for_fresh_wrist(
            input_device,
            args.hand,
            args.quest_reference_timeout,
            newer_than=max(
                realtime_start_barrier,
                float(prestart_wrist["timestamp"]),
            ),
            max_age=realtime_config.max_source_age,
        )
        reference_pose = arm_execution.command_pose()
        latest_arm_command = arm_controller.initialize(
            _wrist_dict_to_matrix(wrist_reference),
            reference_pose,
            float(wrist_reference["timestamp"]),
        )
        if arm_controller.enabled:
            from rokae.control.command import EECommand

            initial_target_accepted = arm_execution.set_target(
                EECommand(
                    pose=latest_arm_command.base_target_raw,
                    timestamp=float(wrist_reference["timestamp"]),
                    sequence=arm_sequence,
                    source="quest",
                )
            )
            if not initial_target_accepted:
                raise RuntimeError(
                    "Post-start Quest target was rejected; diagnostics="
                    f"{arm_driver.diagnostics()}"
                )
            arm_sequence += 1
        else:
            print(
                "ROKAE realtime mode remains HOLDING; arm is disabled "
                "until keyboard e."
            )
    except BaseException:
        try:
            arm_driver.disconnect()
        except Exception as close_error:
            print(f"ROKAE setup cleanup failed: {close_error}", file=sys.stderr)
        raise

    return ArmComponents(arm_driver, arm_execution, arm_controller, robot_initial_pose, latest_arm_command, arm_sequence, arm_control_mode, config, init_config, realtime_config, mapper, T_world_base, quest_to_robot_rotation, controller_spec)
