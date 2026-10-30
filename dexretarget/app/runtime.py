"""Application composition root for existing Quest/Inspire/ROKAE workflows."""
from __future__ import annotations
import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

from dexretarget.lifecycle import close_resource, close_resources
from .debug import _format_matrix, _record_command, print_arm_status
from dexretarget.config import PROJECT_ROOT
from dexretarget.teleop.rokae import (_pose_xyz_quat_to_matrix, _wrist_dict_to_matrix, _load_yaml, _config_value, _wait_for_fresh_wrist, _warn_if_non_realtime_kernel)

from anydexretarget.teleop import (
    ArmCommand,
    ArmTeleopController,
    PoseSafetyConfig,
    PoseSafetyLimiter,
    RealtimeCartesianConfig,
    RelativePoseMapper,
    make_transform,
    matrix_to_rotation_vector,
    quaternion_xyzw_to_matrix,
)
from dexretarget.teleop.devices.inspire import InspireSerialOutput
from dexretarget.teleop.devices.rokae import (
    MockRokaeDriver,
    RokaeDriverBase,
    RokaeXCoreDriver,
)


def run_teleop(args: argparse.Namespace) -> None:
    """Also release devices when setup fails before the main control loop."""
    resources = {}
    try:
        _run_teleop(args, resources)
    finally:
        # Close the shared gate before joining device producers on setup errors.
        close_resource(resources, "session")
        close_resources(resources)


def _run_teleop(args: argparse.Namespace, resources) -> None:
    """Run hand, arm, or combined teleoperation from one Quest receiver."""
    from dexretarget.teleop.devices.quest3 import Quest3

    explicit_outputs = args.enable_hand or args.enable_arm
    enable_hand = args.enable_hand if explicit_outputs else True
    enable_arm = args.enable_arm if explicit_outputs else True

    profile = getattr(args, "task_profile", None)
    home_config = profile.get("home") if profile else None
    if home_config and not enable_arm:
        raise ValueError("Task home requires the arm output")
    if home_config and args.rokae_robot_type not in {None, "xmate-er-pro-7"}:
        raise ValueError("Task home profiles currently require the seven-axis xmate-er-pro-7")
    if args.capture and (not enable_hand or not enable_arm):
        raise ValueError("Dataset capture requires both arm and hand outputs")
    usb_serial = None
    use_relay = args.quest3_source == 'relay'
    if args.quest3_source == 'auto' and args.quest3_usb:
        from dexretarget.teleop.devices.quest_relay import default_socket_path
        use_relay = (args.quest3_relay_socket or default_socket_path()).exists()
    if use_relay:
        from dexretarget.teleop.devices.quest_relay import QuestRelayInput
        try:
            input_device = QuestRelayInput(args.quest3_relay_socket)
        except OSError as exc:
            config_option = f' --config {args.config}' if getattr(args, 'config', None) else ''
            raise RuntimeError('Quest 通信服务不可用；先在另一终端运行 '
                               f'python -m dexretarget.quest{config_option}') from exc
        print(f'Quest 输入：读取独立通信服务 {input_device.path}（退出遥操作不关闭 USB）。', flush=True)
    elif args.quest3_usb:
        from .quest_usb import prepare_usb
        usb_serial = prepare_usb(args.quest3_port)
        input_device = Quest3(host=getattr(args, 'quest3_host', '0.0.0.0'),
                              port=args.quest3_port, protocol=args.quest3_protocol)
    else:
        input_device = Quest3(host=getattr(args, 'quest3_host', '0.0.0.0'),
                              port=args.quest3_port, protocol=args.quest3_protocol)
    resources["Quest"] = input_device.stop
    args.quest3_effective_source = 'relay' if use_relay else 'direct'
    if use_relay or args.quest3_usb:
        from .quest_usb import open_hts, wait_for_usb_tracking
        # Bind/listen must succeed before HTS opens a client socket. No outputs
        # or collection workers exist during this unlimited initial wait.
        input_device.wait_until_ready()
        if not use_relay:
            print(f'Quest TCP 接收服务已就绪，端口 {args.quest3_port}。', flush=True)
            open_hts(usb_serial)
        wait_for_usb_tracking(input_device, args.hand, require_landmarks=enable_hand)
    collector = None
    capture_sources = None
    if args.capture:
        from dexretarget.dataset.recorder import Recorder

        collector = Recorder(args.collection_config, context={
            "simulated": bool(args.mock_rokae), "entrypoint": "python -m dexretarget",
            "hand_side": args.hand, "task_profile": profile,
            "teleop_args": {key: str(value) if isinstance(value, Path) else value
                            for key, value in vars(args).items()},
        })
        resources["dataset"] = collector.close

    hand_output: Optional[InspireSerialOutput] = None
    hand_retargeter: Optional[Retargeter] = None
    joint_names: list[str] = []
    arm_driver: Optional[RokaeDriverBase] = None
    arm_controller: Optional[ArmTeleopController] = None
    robot_initial_pose: Optional[np.ndarray] = None
    latest_arm_command: Optional[ArmCommand] = None
    arm_sequence = 0
    arm_control_mode = "cartesian"
    arm_model = None
    arm_execution = None
    controller_spec = None
    record_file = None

    if enable_hand:
        from anydexretarget import Retargeter
        config_path = args.hand_config
        if config_path is None:
            config_path = f"config/retargeting/{args.optimizer}/quest3/quest3_inspire_hand.yaml"
        config_file = PROJECT_ROOT / config_path
        hand_retargeter = Retargeter.from_yaml(str(config_file), args.hand)
        joint_names = hand_retargeter.optimizer.robot.dof_joint_names
        hand_output = InspireSerialOutput(
            port_name=args.inspire_port,
            baudrate=args.inspire_baudrate,
            hand_id=args.inspire_hand_id,
        )
        resources["hand serial"] = hand_output.close

    if enable_arm:
        from dexretarget.teleop.rokae import create_arm
        arm = create_arm(args, home_config, input_device, hand_output)
        resources["arm disconnect"] = arm.arm_driver.disconnect
        arm_driver = arm.arm_driver
        arm_execution = arm.arm_execution
        arm_controller = arm.arm_controller
        robot_initial_pose = arm.robot_initial_pose
        latest_arm_command = arm.latest_arm_command
        arm_sequence = arm.arm_sequence
        arm_control_mode = arm.arm_control_mode
        config = arm.config
        init_config = arm.init_config
        realtime_config = arm.realtime_config
        mapper = arm.mapper
        T_world_base = arm.T_world_base
        quest_to_robot_rotation = arm.quest_to_robot_rotation
        controller_spec = arm.controller_spec
        if args.arm_record is not None:
            args.arm_record.parent.mkdir(parents=True, exist_ok=True)
            record_file = args.arm_record.open("w", encoding="utf-8")
            resources["debug log"] = record_file.close

    stop_event = threading.Event()
    recenter_event = threading.Event()
    from .keyboard import Keyboard
    from .session import TeleopSession

    def set_following(enabled):
        if arm_controller:
            arm_controller.set_enabled(enabled)
            arm_driver.hold()
            if enabled:
                wrist = input_device.get_wrist_pose(args.hand)
                arm_controller.initialize(_wrist_dict_to_matrix(wrist), arm_execution.command_pose(), float(wrist["timestamp"]))
                arm_execution.reset_tracking()
                recenter_event.clear()
            else:
                recenter_event.set()

    def check_ready():
        tracking = input_device.get_hand_tracking_data(args.hand)
        now = time.monotonic()
        if enable_arm and (tracking["wrist"] is None or now - tracking["wrist"]["timestamp"] > .20):
            raise RuntimeError("Fresh Quest wrist required before enabling/recording")
        if enable_hand and (tracking["fingers"] is None or tracking["landmarks_timestamp"] is None or now - tracking["landmarks_timestamp"] > .20):
            raise RuntimeError("Fresh Quest landmarks required before enabling/recording")

    def do_recenter():
        recenter_event.set()

    from dexretarget.teleop.home import run_home
    def return_home(cancelled):
        run_home(home_config, arm_driver, arm_execution, session,
                 cancelled=cancelled, stop_event=stop_event, recenter_event=recenter_event,
                 hand_output=hand_output, power_on=args.rokae_power_on,
                 on_event=collector.event if collector else None)

    session = TeleopSession(
        collector=collector, task=args.task, set_following=set_following,
        ready=check_ready, home=return_home if home_config else None,
        cancel_home=(lambda: arm_driver.cancel_motion()) if arm_driver else (lambda: None),
        recenter=do_recenter, quit_event=stop_event, enabled=args.arm_start_enabled,
    )
    resources["session"] = session.close
    keyboard = Keyboard(stop_event)
    resources["keyboard"] = keyboard.close

    from dexretarget.teleop.loop import TeleopLoop
    from .capture import CaptureObserver
    observer = CaptureObserver(collector) if collector else None
    loop = TeleopLoop(
        input_device, args.hand, session, retargeter=hand_retargeter,
        arm_controller=arm_controller, execution=arm_execution,
        initial_command=latest_arm_command, sequence=arm_sequence,
        observer=observer, stop_event=stop_event, recenter_event=recenter_event,
        debug=(lambda command, sequence: _record_command(record_file, command, sequence, arm_driver)) if record_file else None,
    )

    print("Starting Quest 3 teleoperation")
    print(f"  Outputs: hand={enable_hand}, arm={enable_arm}")
    print(f"  Quest receiver: {'local relay' if use_relay else args.quest3_protocol + ' 0.0.0.0:' + str(args.quest3_port)}")
    if enable_hand:
        print(
            f"  Inspire serial: {args.inspire_port} @ {args.inspire_baudrate} "
            f"(id={args.inspire_hand_id}, command=100 Hz)"
        )
    if enable_arm:
        backend = "mock" if args.mock_rokae else "xCoreSDK-CPP 0.3.4/pybind11"
        print(f"  ROKAE backend: {backend}")
        print(f"  ROKAE control: {arm_control_mode}")
        print(f"  Initial arm state: {arm_controller.status}")
    print("=" * 72)

    loop.start()
    resources["teleoperation loop"] = loop.close
    control_dt = 0.01
    print_dt = 1.0 / args.arm_print_hz if args.arm_print_hz > 0.0 else float("inf")
    next_print = time.monotonic()
    start_time = time.monotonic()
    from dexretarget.teleop.tracking import fresh_timestamp
    from .tracking import TrackingRecovery
    tracking_config = config.get("teleoperation", {}) if enable_arm else {}
    recovery = TrackingRecovery(session, resume_delay=tracking_config.get("tracking_resume_delay_s", 0.01))
    hand_command_count = 0
    previous_arm_status: Optional[str] = None

    try:
        if collector is not None:
            # Source startup never waits for camera warmup in the control loop.
            collector.context["control_mode"] = arm_control_mode
            collector.context["robot_config"] = config
            collector.context["effective_init"] = init_config
            collector.context["effective_control"] = {
                "realtime": vars(realtime_config),
                "controller": {key: value.tolist() if isinstance(value, np.ndarray) else value
                               for key, value in vars(controller_spec).items()},
                "world_from_base": T_world_base.tolist(),
                "quest_to_robot_rotation": quest_to_robot_rotation.tolist(),
                "translation_scale": mapper.translation_scale,
                "rotation_scale": mapper.rotation_scale,
                "rotation_mapping": mapper.rotation_mapping,
                "wrist_to_tool_rotation": mapper.wrist_to_tool_rotation.tolist(),
                "hand_side": args.hand,
            }
            collector.context["hand_config_text"] = config_file.read_text()
            from .sources import CaptureSources
            capture_sources = CaptureSources(collector.config, collector, read_failures=collector.read_failures)
            resources["capture sources"] = capture_sources.close
            capture_sources.start(arm=arm_driver, hand=hand_output)
        keyboard.start()
        while not stop_event.is_set():
            loop_start = time.monotonic()
            for key in keyboard.drain():
                try:
                    session.handle(key)
                except (ValueError, RuntimeError) as exc:
                    print(f"Control: {exc}", flush=True)
            with session.lock:
                qpos_to_send, arm_command_to_print, last_landmarks_timestamp = loop.snapshot()

                now = time.monotonic()
                tracking = input_device.get_hand_tracking_data(args.hand)
                wrist = tracking['wrist']
                hand_fresh = not enable_hand or fresh_timestamp(last_landmarks_timestamp, now, .20)
                wrist_fresh = not enable_arm or (wrist is not None and fresh_timestamp(
                    wrist['timestamp'], now, arm_controller.command_timeout))
                def hold_tracking():
                    if arm_driver:
                        arm_driver.hold()
                    recenter_event.set()

                def resume_tracking():
                    if arm_controller:
                        anchored = arm_controller.initialize(_wrist_dict_to_matrix(wrist), arm_execution.command_pose(), float(wrist['timestamp']))
                        loop.last_wrist_timestamp = float(wrist['timestamp'])
                        loop.set_command(anchored)
                        arm_execution.reset_tracking()
                        recenter_event.clear()

                recovery.update(hand_fresh and wrist_fresh, now, hold=hold_tracking, resume=resume_tracking)
                if qpos_to_send is not None and session.enabled and not session.tracking_held:
                    assert hand_output is not None
                    hand_result = hand_output.send(recovery.hand.target(qpos_to_send, now), joint_names)
                    if observer:
                        observer.hand_target(hand_result, hand_command_count)
                    hand_command_count += 1

            now = time.monotonic()
            if enable_arm and session.operation != "home" and not session.fault:
                assert arm_controller is not None and arm_driver is not None
                polled = arm_controller.poll(now) if session.enabled else None
                if collector is not None:
                    collector.control(session.enabled and not session.tracking_held and arm_controller.enabled and arm_controller.status == "active")
                if polled is not None:
                    loop.set_command(polled)
                    arm_command_to_print = polled
                if arm_controller.status != previous_arm_status:
                    print(f"Arm state: {arm_controller.status} (following_reason={session.following_reason})")
                    previous_arm_status = arm_controller.status

                if (
                    args.arm_print_hz > 0.0
                    and arm_command_to_print is not None
                    and now >= next_print
                ):
                    print_arm_status(arm_command_to_print, arm_driver, arm_controller.status)
                    next_print = now + print_dt

                driver_diagnostics = arm_driver.diagnostics()
                callback_error = driver_diagnostics.get("callback_error")
                if callback_error is not None:
                    raise RuntimeError(
                        f"ROKAE realtime callback failed: {callback_error}"
                    )
                if driver_diagnostics["watchdog_state"] == "safe_stop":
                    raise RuntimeError(
                        "Quest wrist stream exceeded stop_timeout; ROKAE realtime "
                        "motion was finished. Restart after checking the link."
                    )

            if hand_command_count > 0 and hand_command_count % 500 == 0:
                elapsed = max(now - start_time, 1e-6)
                print(
                    f"Hand command: {hand_command_count / elapsed:.1f} Hz | "
                    f"Hand input: {loop.hand_frame_count / elapsed:.1f} Hz | "
                    f"Wrist input: {loop.arm_frame_count / elapsed:.1f} Hz"
                )

            if loop.error is not None:
                raise RuntimeError(
                    "Quest input/retarget thread stopped"
                ) from loop.error

            if capture_sources is not None:
                capture_sources.check_health()
            if collector is not None:
                collector.check_health()

            sleep_time = control_dt - (time.monotonic() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nStopping controller...")
    finally:
        stop_event.set()
        def cleanup(label, operation):
            return close_resource(resources, label, operation)
        cleanup("session", session.close)
        cleanup("keyboard", keyboard.close)
        cleanup("teleoperation loop", loop.close)
        cleanup("Quest", input_device.stop)
        if arm_driver is not None:
            cleanup("arm hold", arm_driver.hold)
        if collector is not None:
            cleanup("dataset", collector.close)
        if capture_sources is not None:
            cleanup("capture sources", capture_sources.close)
        if hand_output is not None:
            cleanup("hand serial", hand_output.close)
        if arm_driver is not None:
            cleanup("arm stop", arm_driver.stop)
            cleanup("arm disconnect", arm_driver.disconnect)
        if record_file is not None:
            cleanup("debug log", record_file.close)


def __getattr__(name):
    if name == "Retargeter":
        from anydexretarget import Retargeter
        return Retargeter
    raise AttributeError(name)
