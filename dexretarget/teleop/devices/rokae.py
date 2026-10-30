"""ROKAE drivers backed by the vendored xCoreSDK-CPP v0.3.4 pybind module."""

from __future__ import annotations

import sys
import importlib
from importlib.machinery import PathFinder
import platform
import sysconfig
import threading
import time
from collections import deque
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from anydexretarget.teleop.pose import validate_transform
from anydexretarget.teleop.realtime import (
    CartesianSetpointGenerator,
    RealtimeCartesianConfig,
)


from dexretarget.config import PROJECT_ROOT
VENDORED_CPP_MODULE_DIR = PROJECT_ROOT / "rokae" / "python"


def plan_init_joint_targets(
    current: Sequence[float],
    final_target: Sequence[float],
    max_segment_delta: float,
    staged_joint_indices: Sequence[int] = (6,),
) -> list[np.ndarray]:
    """Interpolate an arbitrary init displacement into small joint segments.

    Selected joints (J7 by default) are held at their measured angle while the
    arm shape moves, then are moved in a second coarse leg.  There is no limit
    on the *total* displacement; only each generated segment is bounded.
    """
    current_array = np.asarray(current, dtype=np.float64)
    final_array = np.asarray(final_target, dtype=np.float64)
    if current_array.ndim != 1 or final_array.shape != current_array.shape:
        raise ValueError("current and init joint targets must have the same 1-D shape")
    if not np.all(np.isfinite(current_array)) or not np.all(np.isfinite(final_array)):
        raise ValueError("init joint targets must be finite")
    segment_limit = float(max_segment_delta)
    if not np.isfinite(segment_limit) or segment_limit <= 0.0:
        raise ValueError("max_segment_delta must be positive")
    staged = tuple(int(index) for index in staged_joint_indices)
    if len(set(staged)) != len(staged) or any(
        index < 0 or index >= current_array.size for index in staged
    ):
        raise ValueError("staged_joint_indices contains an invalid/duplicate index")
    cursor = current_array.copy()
    planned: list[np.ndarray] = []

    coarse_targets: list[np.ndarray] = []
    if staged:
        arm_shape = final_array.copy()
        arm_shape[list(staged)] = current_array[list(staged)]
        if np.max(np.abs(arm_shape - cursor)) > 1e-6:
            coarse_targets.append(arm_shape)
    coarse_targets.append(final_array)

    for coarse_index, coarse_target in enumerate(coarse_targets, start=1):
        delta = float(np.max(np.abs(coarse_target - cursor)))
        if delta <= 1e-6:
            cursor = coarse_target
            continue
        step_count = max(1, int(np.ceil(delta / segment_limit)))
        start = cursor.copy()
        for step_index in range(1, step_count + 1):
            fraction = step_index / step_count
            planned.append(start + fraction * (coarse_target - start))
        cursor = coarse_target
    return planned


class RokaeDriverBase(ABC):
    """Common non-queued interface for mock and real ROKAE backends."""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def move_to_init(
        self,
        joint_position: Optional[Sequence[float]],
        *,
        joint_speed: float = 0.10,
        timeout: float = 60.0,
        max_joint_delta: Optional[float] = None,
        joint_waypoints: Optional[Sequence[Sequence[float]]] = None,
        max_segment_delta: Optional[float] = None,
        power_on: bool = True,
        cancel_event=None,
    ) -> None: ...

    @abstractmethod
    def start(self, power_on: bool = True) -> None: ...

    @abstractmethod
    def get_tcp_pose(self) -> np.ndarray: ...

    @abstractmethod
    def get_controller_base_frame(self) -> np.ndarray: ...

    @abstractmethod
    def get_joint_positions(self) -> Optional[np.ndarray]: ...

    @abstractmethod
    def get_command_pose(self) -> np.ndarray: ...

    @abstractmethod
    def set_target_pose(
        self,
        base_T_tcp_target: np.ndarray,
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
    ) -> bool: ...

    @abstractmethod
    def hold(self) -> None: ...

    @abstractmethod
    def diagnostics(self) -> dict[str, Any]: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    def cancel_motion(self) -> None:
        self.stop()


class MockRokaeDriver(RokaeDriverBase):
    """Hardware-free backend using the exact production setpoint generator."""

    def __init__(
        self,
        initial_pose: np.ndarray,
        control_hz: float = 1000.0,
        command_timeout: float = 0.20,
        realtime_config: Optional[RealtimeCartesianConfig] = None,
        control_mode: str = "cartesian",
        initial_joints: Optional[Sequence[float]] = None,
        kinematics: Any = None,
    ) -> None:
        pose = validate_transform(initial_pose, "mock initial pose")
        if realtime_config is None:
            realtime_config = RealtimeCartesianConfig(
                control_hz=control_hz,
                hold_timeout=command_timeout,
                stop_timeout=max(command_timeout * 3.0, command_timeout + 0.05),
                workspace_min=(-10.0, -10.0, -10.0),
                workspace_max=(10.0, 10.0, 10.0),
                translation_cutoff_hz=1000.0,
                rotation_cutoff_hz=1000.0,
                max_translation_speed=1000.0,
                max_angular_speed=1000.0,
                max_translation_acceleration=1e6,
                max_angular_acceleration=1e6,
                max_translation_jerk=1e9,
                max_angular_jerk=1e9,
            )
        self.realtime_config = realtime_config
        self._generator = CartesianSetpointGenerator(realtime_config)
        self._current_pose = pose.copy()
        self._connected = False
        self._running = False
        self._command_count = 0
        self._last_step_state = "not_started"
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._generator_lock = threading.RLock()
        self._cartesian_hold = threading.Event()
        if control_mode not in {"cartesian", "pinocchio", "impedance"}:
            raise ValueError("control_mode must be cartesian, pinocchio, or impedance")
        self.control_mode = control_mode
        self._kinematics = kinematics
        self._joint_stream = None
        self._command_joints: Optional[np.ndarray] = None
        self._stream_lock = threading.Lock()
        if control_mode == "pinocchio":
            if kinematics is None or initial_joints is None:
                raise ValueError("pinocchio mock requires kinematics and initial_joints")
            from rokae.control.smoothing import JointStream, JointStreamConfig

            joints = np.asarray(initial_joints, dtype=np.float64)
            self._command_joints = joints.copy()
            self._current_pose = kinematics.fk(joints)
            self._joint_stream = JointStream(
                JointStreamConfig.from_limits(kinematics.stream_limits)
            )

    @property
    def command_count(self) -> int:
        with self._lock:
            return self._command_count

    @property
    def timed_out(self) -> bool:
        return self._last_step_state in {"holding_stale", "safe_stop"}

    def connect(self) -> None:
        self._connected = True

    def move_to_init(self, joint_position: Optional[Sequence[float]], **kwargs: Any) -> None:
        if self._running:
            raise RuntimeError("stop realtime before mock home")
        cancelled = kwargs.get("cancel_event")
        if cancelled is not None and cancelled.is_set():
            raise RuntimeError("Home cancelled")
        if joint_position is not None:
            joints = np.asarray(joint_position, dtype=np.float64)
            if joints.shape != (7,) or not np.all(np.isfinite(joints)):
                raise ValueError("Mock home requires seven finite joint positions")
            with self._lock:
                self._command_joints = joints.copy()
                if self._kinematics is not None:
                    self._current_pose = self._kinematics.fk(joints)

    def cancel_motion(self) -> None:
        self.stop()

    def start(
        self,
        power_on: bool = True,
        control_mode: str = "cartesian",
        kinematics: Any = None,
    ) -> None:
        del power_on, control_mode, kinematics
        if not self._connected:
            raise RuntimeError("Mock ROKAE driver is not connected")
        if self._running:
            return
        now = time.monotonic()
        if self._joint_stream is not None and self._command_joints is not None:
            self._joint_stream.reset(self._command_joints, now)
        else:
            self._generator.reset(self._current_pose, now)
        self._stop_event.clear()
        self._cartesian_hold.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._control_loop, name="mock-rokae-rt", daemon=True
        )
        self._thread.start()

    def _control_loop(self) -> None:
        period = 1.0 / self.realtime_config.control_hz
        deadline = time.monotonic()
        while not self._stop_event.is_set():
            if self._joint_stream is not None and self._kinematics is not None:
                with self._stream_lock:
                    joints = self._joint_stream.step(time.monotonic())
                pose = self._kinematics.fk(joints)
                state = str(self._joint_stream.watchdog)
                with self._lock:
                    self._command_joints = joints
                    self._current_pose = pose
                    self._last_step_state = state
                    self._command_count += 1
            else:
                with self._generator_lock:
                    if self._cartesian_hold.is_set():
                        # HOLD is a latch, not a synthetic command consuming
                        # the caller's sequence number (including initial 0).
                        self._generator.reset(self._generator.pose, time.monotonic())
                    step = None if self._cartesian_hold.is_set() else self._generator.step()
                with self._lock:
                    if step is not None:
                        self._current_pose = step.pose
                    self._last_step_state = "holding" if step is None else step.state
                    self._command_count += 1
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0.0:
                self._stop_event.wait(delay)
            else:
                deadline = time.monotonic()

    def get_tcp_pose(self) -> np.ndarray:
        with self._lock:
            return self._current_pose.copy()

    def get_state(self) -> dict[str, Any]:
        """Synthetic telemetry, used only with explicitly marked mock recordings."""
        joints = self.get_joint_positions()
        return {"tcp_pose": self.get_tcp_pose(), "timestamp": time.monotonic(),
                "joint_position": np.full(7, np.nan) if joints is None else joints,
                "joint_velocity": np.full(7, np.nan)}

    def get_controller_base_frame(self) -> np.ndarray:
        return np.eye(4, dtype=np.float64)

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """Cartesian mock has no joint state. Pinocchio mock returns the stream."""
        with self._lock:
            if self._command_joints is None:
                return None
            return self._command_joints.copy()

    def get_command_pose(self) -> np.ndarray:
        if self._joint_stream is not None and self._kinematics is not None:
            with self._stream_lock:
                joints = self._joint_stream.position.copy()
            return self._kinematics.fk(joints)
        with self._generator_lock:
            return self._generator.pose

    def get_command_joints(self) -> np.ndarray:
        if self._joint_stream is None:
            raise RuntimeError("this ROKAE backend has no joint command")
        with self._stream_lock:
            return self._joint_stream.position.copy()

    def set_target_joints(
        self,
        joint_target: Sequence[float],
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
    ) -> bool:
        if self._joint_stream is None or not self._running:
            raise RuntimeError("Mock ROKAE joint stream is not running")
        now = time.monotonic()
        source = now if source_timestamp is None else float(source_timestamp)
        seq = self._command_count if sequence is None else int(sequence)
        with self._stream_lock:
            return self._joint_stream.publish(
                np.asarray(joint_target, dtype=np.float64),
                source,
                seq,
                now,
            )

    def set_target_pose(
        self,
        base_T_tcp_target: np.ndarray,
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
    ) -> bool:
        if self.control_mode == "pinocchio":
            raise RuntimeError("pinocchio mock does not accept Cartesian targets")
        if not self._running:
            raise RuntimeError("Mock ROKAE realtime loop is not running")
        with self._generator_lock:
            accepted = self._generator.publish_target(
                base_T_tcp_target,
                source_timestamp=source_timestamp,
                sequence=sequence,
            )
            if accepted:
                self._cartesian_hold.clear()
            return accepted

    def hold(self) -> None:
        if not self._running:
            return
        if self._joint_stream is not None:
            with self._stream_lock:
                self._joint_stream.hold()
            return
        with self._generator_lock:
            self._cartesian_hold.set()

    def diagnostics(self) -> dict[str, Any]:
        return {
            "backend": "mock",
            "running": self._running,
            "watchdog_state": self._last_step_state,
            "command_count": self.command_count,
            "buffer": self._generator.buffer.stats,
        }

    def stop(self) -> None:
        if not self._running:
            return
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._running = False

    def disconnect(self) -> None:
        self.stop()
        self._connected = False


class RokaeXCoreDriver(RokaeDriverBase):
    """Thin Python facade over the C++ v0.3.4 realtime driver.

    Quest processing calls :meth:`set_target_pose`, which only replaces one
    C++ latest-value slot.  The xCoreSDK callback, SE(3) filtering, motion
    limiting and watchdog all execute in C++ and never depend on the Python GIL.
    """

    _ROBOT_TYPES = {"xmate-6": 6, "xmate-er-pro-7": 7, "standard-6": 6}

    def __init__(
        self,
        robot_ip: str,
        local_ip: str,
        robot_type: str = "xmate-er-pro-7",
        rt_network_tolerance: int = 20,
        realtime_config: Optional[RealtimeCartesianConfig] = None,
        sdk_path: Optional[Path] = None,
        controller_rate_limit: bool = True,
        controller_filter_cutoff_hz: float = 50.0,
        nrt_online_speed_scale: float = 1.0,
    ) -> None:
        if robot_type not in self._ROBOT_TYPES:
            raise ValueError(f"unsupported ROKAE robot type: {robot_type}")
        if not 0 <= rt_network_tolerance <= 100:
            raise ValueError("rt_network_tolerance must be in [0, 100]")
        if controller_filter_cutoff_hz <= 0.0:
            raise ValueError("controller_filter_cutoff_hz must be positive")
        if not 0.01 <= nrt_online_speed_scale <= 1.0:
            raise ValueError("nrt_online_speed_scale must be in [0.01, 1.0]")
        self.robot_ip = str(robot_ip)
        self.local_ip = str(local_ip)
        self.robot_type = robot_type
        self.rt_network_tolerance = int(rt_network_tolerance)
        self.realtime_config = realtime_config or RealtimeCartesianConfig()
        self.module_path = Path(sdk_path or VENDORED_CPP_MODULE_DIR)
        self.controller_rate_limit = bool(controller_rate_limit)
        self.controller_filter_cutoff_hz = float(controller_filter_cutoff_hz)
        self.nrt_online_speed_scale = float(nrt_online_speed_scale)
        # Used only by start_realtime_cartesian_impedance. Position mode ignores them.
        self._impedance_health_history = deque(maxlen=8)
        self.apply_load_on_impedance_start = False
        self.expected_load_mass = 0.0
        self.expected_load_cog = (0.0, 0.0, 0.0)
        self.impedance_force_frame = "world"
        self.cartesian_stiffness = (300.0, 300.0, 300.0, 15.0, 15.0, 15.0)
        self.cartesian_desired_wrench = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.max_equilibrium_translation_error = 0.04
        self.max_equilibrium_rotation_error = 0.30
        self._binding: Any = None
        self._driver: Any = None
        self._connected = False
        self._started = False
        self._control_mode = "cartesian"
        self._kinematics: Any = None
        self._last_command_pose: Optional[np.ndarray] = None
        self._next_sequence = 0

    def _import_binding(self) -> Any:
        candidates = list(self.module_path.glob("_rokae_cpp*.so"))
        if not candidates:
            raise FileNotFoundError(
                f"ROKAE C++ module is not built under {self.module_path}; "
                "run: bash rokae/build.sh"
            )
        if PathFinder.find_spec("_rokae_cpp", [str(self.module_path.resolve())]) is None:
            raise RuntimeError(
                f"No compatible ROKAE extension for Python {sys.version_info.major}.{sys.version_info.minor} "
                f"on {platform.machine()} under {self.module_path}. "
                f"Expected _rokae_cpp{sysconfig.get_config_var('EXT_SUFFIX')}; "
                f"found: {', '.join(path.name for path in candidates)}. "
                "Activate dexteleop and run: bash rokae/build.sh"
            )
        # Import the extension under its package-qualified name.  Importing the
        # same pybind11 shared object once as ``rokae.python._rokae_cpp`` and a
        # second time as top-level ``_rokae_cpp`` registers every C++ type
        # twice and aborts with "type ... is already registered".
        package_dir = PROJECT_ROOT / "rokae" / "python"
        try:
            if self.module_path.resolve() == package_dir.resolve():
                return importlib.import_module("rokae.python._rokae_cpp")
            path_text = str(self.module_path.resolve())
            if path_text not in sys.path:
                sys.path.insert(0, path_text)
            return importlib.import_module("_rokae_cpp")
        except ImportError as exc:
            raise RuntimeError(
                f"Failed to load the ROKAE C++ bridge on {platform.machine()}: {exc}. "
                "Rebuild with the active Python and verify matching SDK runtime dependencies."
            ) from exc

    def _cpp_config(self, binding: Any) -> Any:
        cfg = binding.RealtimeConfig()
        cfg.nrt_online_speed_scale = self.nrt_online_speed_scale
        for name in (
            "translation_cutoff_hz",
            "rotation_cutoff_hz",
            "response_hz",
            "translation_deadband",
            "rotation_deadband",
            "max_translation_speed",
            "max_angular_speed",
            "max_translation_acceleration",
            "max_angular_acceleration",
            "max_translation_jerk",
            "max_angular_jerk",
            "max_target_translation_delta",
            "max_target_rotation_delta",
            "workspace_min",
            "workspace_max",
            "hold_timeout",
            "stop_timeout",
            "max_source_age",
            "future_tolerance",
        ):
            setattr(cfg, name, getattr(self.realtime_config, name))
        cfg.controller_rate_limit = self.controller_rate_limit
        cfg.controller_filter_cutoff_hz = self.controller_filter_cutoff_hz
        cfg.rt_network_tolerance = self.rt_network_tolerance
        cfg.apply_load_on_impedance_start = self.apply_load_on_impedance_start
        cfg.expected_load_mass = self.expected_load_mass
        cfg.expected_load_cog = self.expected_load_cog
        cfg.impedance_force_frame = self.impedance_force_frame
        cfg.cartesian_stiffness = tuple(self.cartesian_stiffness)
        cfg.cartesian_desired_wrench = tuple(self.cartesian_desired_wrench)
        cfg.max_equilibrium_translation_error = float(self.max_equilibrium_translation_error)
        cfg.max_equilibrium_rotation_error = float(self.max_equilibrium_rotation_error)
        cfg.validate()
        return cfg

    def connect(self) -> None:
        if self._connected:
            return
        binding = self._import_binding()
        driver = binding.RokaeDriver(
            self.robot_ip,
            self.local_ip,
            self.robot_type,
            self._cpp_config(binding),
        )
        try:
            driver.connect()
        except Exception:
            try:
                driver.disconnect()
            except Exception:
                pass
            raise
        self._binding = binding
        self._driver = driver
        self._connected = True

    def get_tcp_pose(self) -> np.ndarray:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        return validate_transform(
            np.asarray(self._driver.get_tcp_pose(), dtype=np.float64),
            "xCoreSDK-CPP TCP pose",
        )

    def calculate_fk(self, joint_position: Sequence[float]) -> np.ndarray:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        return validate_transform(
            np.asarray(
                self._driver.calculate_fk(list(joint_position)), dtype=np.float64
            ),
            "xCoreSDK controller FK",
        )

    def calculate_fk_in_base(self, joint_position: Sequence[float]) -> np.ndarray:
        """Official FK in the RT base frame; does not command any motion."""
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        if not hasattr(self._driver, "calculate_fk_in_base"):
            raise RuntimeError("rebuild the ROKAE bridge: bash rokae/build.sh")
        return validate_transform(
            np.asarray(self._driver.calculate_fk_in_base(list(joint_position)), dtype=np.float64),
            "xCoreSDK base-relative FK",
        )

    def get_controller_base_frame(self) -> np.ndarray:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        return validate_transform(
            np.asarray(self._driver.get_base_frame(), dtype=np.float64),
            "xCoreSDK controller baseFrame",
        )

    def get_joint_positions(self) -> Optional[np.ndarray]:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        positions = np.asarray(self._driver.get_joint_positions(), dtype=np.float64)
        dof = self._ROBOT_TYPES[self.robot_type]
        if positions.shape != (dof,) or not np.all(np.isfinite(positions)):
            raise RuntimeError("xCoreSDK returned invalid joint positions")
        return positions.copy()

    def get_state(self) -> dict[str, Any]:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        state = dict(self._driver.get_state())
        state["tcp_pose"] = validate_transform(
            np.asarray(state["tcp_pose"], dtype=np.float64), "ROKAE state TCP"
        )
        return state

    def move_to_init(
        self,
        joint_position: Optional[Sequence[float]],
        *,
        joint_speed: float = 0.10,
        timeout: float = 60.0,
        max_joint_delta: Optional[float] = None,
        joint_waypoints: Optional[Sequence[Sequence[float]]] = None,
        max_segment_delta: Optional[float] = None,
        power_on: bool = True,
        cancel_event=None,
    ) -> None:
        if joint_position is None:
            return
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        if self._started:
            raise RuntimeError("move_to_init must run before realtime control")
        dof = self._ROBOT_TYPES[self.robot_type]
        target = np.asarray(joint_position, dtype=np.float64)
        if target.shape != (dof,) or not np.all(np.isfinite(target)):
            raise ValueError(f"init joint_position must contain {dof} finite radians")
        if not 0.0 < joint_speed <= 1.0:
            raise ValueError("init joint_speed must be in (0, 1]")
        del max_joint_delta, joint_waypoints  # legacy API; total travel is unrestricted
        segment_limit = 0.20 if max_segment_delta is None else float(max_segment_delta)
        speed = int(np.clip(round(joint_speed * 1000.0), 5, 1000))
        current = self.get_joint_positions()
        if current is None:
            raise RuntimeError("ROKAE did not return current joints for init planning")
        limits_enabled, soft_limits = self.get_soft_limits()
        if not limits_enabled:
            raise RuntimeError(
                "controller soft limits are disabled; restore them before init"
            )
        margins = np.minimum(
            current - soft_limits[:, 0], soft_limits[:, 1] - current
        )
        if np.any(margins <= 0.15):
            axes = ", ".join(
                f"J{int(index)+1}({margins[index]:+.4f} rad)"
                for index in np.flatnonzero(margins <= 0.15)
            )
            raise RuntimeError(
                "init is blocked because joint(s) are at/near controller soft "
                f"limits: {axes}. Run `python rokae/examples/05_move_to_init.py "
                "--execute --recover-joint-limits` first; teleoperation never "
                "disables soft limits automatically"
            )
        planned = plan_init_joint_targets(current, target, segment_limit)
        if not planned:
            print("[ROKAE] already at configured init; no MoveAbsJ required")
            return
        print(f"[ROKAE] init speed: SDK={speed}, online_scale={self.nrt_online_speed_scale:g}")
        for index, segment_target in enumerate(planned, start=1):
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("Home cancelled")
            print(
                f"[ROKAE] init segment {index}/{len(planned)} target="
                + np.array2string(segment_target, precision=6, separator=",")
            )
            self._driver.move_to_init(
                segment_target.tolist(), speed, float(timeout),
                # The controller declares a segment reached within 0.01 rad;
                # leave deterministic guard slack for that residual when the
                # next interpolated target is sent.
                segment_limit + 0.02, bool(power_on)
            )

    def cancel_motion(self) -> None:
        """Request the existing SDK stop for an operator-aborted home motion."""
        if self._driver is not None:
            self._driver.stop()
            self._started = False

    def get_soft_limits(self) -> tuple[bool, np.ndarray]:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        enabled, limits = self._driver.get_soft_limits()
        values = np.asarray(limits, dtype=np.float64)
        dof = self._ROBOT_TYPES[self.robot_type]
        if values.shape != (dof, 2) or not np.all(np.isfinite(values)):
            raise RuntimeError("xCoreSDK returned invalid soft limits")
        return bool(enabled), values

    def recover_disabled_soft_limits(
        self, expected_joints: Sequence[float], expected_limits: Sequence[Sequence[float]],
        *, clearance_degrees: float = 0.5,
        request_power_on: bool = False,
        jog_rate: float = 0.05,
    ) -> None:
        """Explicit maintenance Jog only; never starts teleoperation or init."""
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        self._driver.recover_disabled_soft_limits(
            list(expected_joints), [list(pair) for pair in expected_limits],
            float(clearance_degrees), bool(request_power_on), float(jog_rate),
        )

    def get_controller_logs(self, count: int = 10, *, faults_only: bool = False) -> list[dict[str, Any]]:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        rows = (self._driver.get_controller_logs(count, True) if faults_only
                else self._driver.get_controller_logs(count))
        return [dict(row) for row in rows]

    def recover_from_joint_limits(
        self,
        target: Sequence[float],
        *,
        speed: int = 20,
        timeout: float = 60.0,
        max_joint_delta: float = 0.5,
    ) -> None:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        self._driver.recover_from_joint_limits(
            list(target), int(speed), float(timeout), float(max_joint_delta)
        )

    def move_joint(
        self,
        joint_position: Sequence[float],
        *,
        speed: int = 300,
        timeout: float = 60.0,
        max_joint_delta: float = 0.25,
        power_on: bool = True,
    ) -> None:
        if self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        self._driver.move_joint(
            list(joint_position), speed, timeout, max_joint_delta, power_on
        )

    def move_cartesian(
        self,
        base_T_tcp_target: np.ndarray,
        *,
        speed: int = 20,
        timeout: float = 60.0,
        max_translation_delta: float = 0.05,
        max_rotation_delta: float = 0.20,
        power_on: bool = True,
    ) -> None:
        if self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        target = validate_transform(base_T_tcp_target, "ROKAE Cartesian target")
        self._driver.move_cartesian(
            target,
            speed,
            timeout,
            max_translation_delta,
            max_rotation_delta,
            power_on,
        )

    def preview_nrt_cartesian_target(
        self, base_T_tcp_target: np.ndarray
    ) -> np.ndarray:
        """Validate/convert base_T_tcp to the active NRT Toolset frame."""
        if self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        target = validate_transform(base_T_tcp_target, "ROKAE Cartesian preview")
        return validate_transform(
            np.asarray(
                self._driver.preview_nrt_cartesian_target(target),
                dtype=np.float64,
            ),
            "ROKAE NRT ref_T_end target",
        )

    def start(
        self,
        power_on: bool = True,
        control_mode: str = "cartesian",
        kinematics: Any = None,
    ) -> None:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        if self._started:
            return
        if control_mode not in {"cartesian", "pinocchio", "impedance"}:
            raise ValueError("control_mode must be cartesian, pinocchio, or impedance")
        if control_mode == "impedance" and self.robot_type == "standard-6":
            raise ValueError(
                "xCoreSDK 0.3.4 Cartesian impedance is only available on cobot models"
            )
        if control_mode == "pinocchio":
            if kinematics is None:
                raise ValueError("pinocchio realtime requires a host kinematic model")
            from rokae.control.smoothing import JointStreamConfig

            enabled, soft = self.get_soft_limits()
            if not enabled:
                raise RuntimeError("controller soft limits must be enabled before Pinocchio control")
            kinematics.intersect_position_limits(soft)
            joints_before = self.get_joint_positions()
            kinematics.require_within_limits(joints_before)
            # All queries precede RT/power-on. FK samples are virtual joint
            # configurations, not movement waypoints. Never fit away a frame error.
            report = kinematics.validate_controller_fk(self.calculate_fk_in_base, joints_before)
            joints_after = self.get_joint_positions()
            if np.max(np.abs(np.asarray(joints_after) - joints_before)) > 0.002:
                raise RuntimeError("robot moved during model validation; retry while stationary")
            kinematics.require_controller_match(joints_after, self.get_tcp_pose())
            self.kinematics_validation = report
            print(f"Pinocchio preflight: {len(report['samples'])} official FK samples; "
                  f"max mismatch={report['max_translation_m']*1000:.2f} mm / "
                  f"{np.rad2deg(report['max_rotation_rad']):.3f} deg; effective limits shared by IK/stream.")
            limits = JointStreamConfig.from_limits(kinematics.stream_limits)
            cfg = self._binding.JointRealtimeConfig()
            cfg.max_velocity = limits.max_velocity.tolist()
            cfg.max_acceleration = limits.max_acceleration.tolist()
            cfg.max_jerk = limits.max_jerk.tolist()
            cfg.position_min = limits.position_min.tolist()
            cfg.position_max = limits.position_max.tolist()
            cfg.reference_cutoff_hz = limits.reference_cutoff_hz
            cfg.response_hz = limits.response_hz
            cfg.deadband = limits.deadband
            cfg.max_target_delta = limits.max_target_delta
            cfg.hold_timeout = limits.hold_timeout
            cfg.stop_timeout = limits.stop_timeout
            cfg.max_source_age = limits.max_source_age
            cfg.future_tolerance = limits.future_tolerance
            cfg.validate()
            self._driver.start_realtime_joint(cfg, bool(power_on))
            self._kinematics = kinematics
        elif control_mode == "impedance":
            self._impedance_health_history.clear()
            self._driver.start_realtime_cartesian_impedance(bool(power_on))
        else:
            self._driver.start_realtime_cartesian(bool(power_on))
        self._control_mode = control_mode
        self._started = True

    def get_command_joints(self) -> np.ndarray:
        if self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        joints = np.asarray(self._driver.get_command_joints(), dtype=np.float64)
        if joints.ndim != 1 or not np.all(np.isfinite(joints)):
            raise RuntimeError("xCoreSDK returned invalid command joints")
        return joints

    def set_target_joints(
        self,
        joint_target: Sequence[float],
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
    ) -> bool:
        if not self._started or self._control_mode != "pinocchio":
            raise RuntimeError("ROKAE realtime joint control is not started")
        assert self._driver is not None
        target = np.asarray(joint_target, dtype=np.float64)
        source = time.monotonic() if source_timestamp is None else float(source_timestamp)
        seq = self._next_sequence if sequence is None else int(sequence)
        accepted = bool(self._driver.set_target_joints(target.tolist(), source, seq))
        if accepted:
            self._next_sequence = max(self._next_sequence, seq + 1)
        return accepted

    def get_command_pose(self) -> np.ndarray:
        if self._driver is None:
            if self._last_command_pose is None:
                raise RuntimeError("ROKAE command pose is unavailable")
            return self._last_command_pose.copy()
        if self._control_mode == "pinocchio":
            if self._kinematics is None:
                raise RuntimeError("pinocchio command pose requires a kinematic model")
            return validate_transform(
                self._kinematics.fk(self.get_command_joints()),
                "ROKAE Pinocchio command pose",
            )
        return validate_transform(
            np.asarray(self._driver.get_command_pose(), dtype=np.float64),
            "ROKAE C++ command pose",
        )

    def set_target_pose(
        self,
        base_T_tcp_target: np.ndarray,
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
    ) -> bool:
        if not self._started:
            raise RuntimeError("ROKAE realtime Cartesian control is not started")
        assert self._driver is not None
        target = validate_transform(base_T_tcp_target, "ROKAE latest target")
        source = time.monotonic() if source_timestamp is None else float(source_timestamp)
        seq = self._next_sequence if sequence is None else int(sequence)
        accepted = bool(self._driver.set_target_pose(target, source, seq))
        if accepted:
            self._last_command_pose = target.copy()
            self._next_sequence = max(self._next_sequence, seq + 1)
        return accepted

    def hold(self) -> None:
        if self._started and self._driver is not None:
            self._driver.hold()

    def diagnostics(self) -> dict[str, Any]:
        if self._driver is None:
            return {"backend": "xCoreSDK-CPP-0.3.4/pybind11", "running": False}
        # Local realtime_running is a start/stop flag, not controller health.
        # Query at 2 Hz outside the native 1 kHz callback, including while paused.
        now = time.monotonic()
        if (self._started and self._control_mode == "impedance"
                and now >= getattr(self, "_next_impedance_health_check", 0.0)):
            self._next_impedance_health_check = now + 0.5
            state = dict(self._driver.get_state())
            # Reuse the existing 2 Hz health query: bounded history, no extra
            # controller RPC or I/O in the 1 kHz callback. Not a peak recorder.
            self._impedance_health_history.append({
                "monotonic_s": now,
                "power": state.get("power_state"),
                "q_rad": state.get("joint_position"),
                "dq_rad_s": state.get("joint_velocity"),
                "tcp_measured": (np.asarray(state["tcp_pose"]).tolist()
                                 if "tcp_pose" in state else None),
                # Last accepted Python target, not the final filtered RT command.
                "last_python_target": (self._last_command_pose.tolist()
                                       if self._last_command_pose is not None else None),
            })
            if state.get("power_state") != "on" or state.get("operate_mode") != "automatic":
                try:
                    logs = self.get_controller_logs()
                except Exception as exc:
                    logs = f"controller log query failed: {exc}"
                raise RuntimeError(
                    f"ROKAE impedance controller is no longer powered in automatic mode: "
                    f"power={state.get('power_state')}, mode={state.get('operate_mode')}; "
                    f"recent 2 Hz state samples (may miss transient peaks): "
                    f"{list(self._impedance_health_history)}; "
                    f"recent logs (may include earlier events): {logs}"
                )
        result = dict(self._driver.diagnostics())
        result["backend"] = "xCoreSDK-CPP-0.3.4/pybind11"
        result["buffer"] = {
            key: result[key]
            for key in (
                "accepted",
                "rejected_sequence",
                "rejected_source_time",
                "rejected_source_age",
                "rejected_target_delta",
                "latest_sequence",
            )
        }
        return result

    def stop(self) -> None:
        if self._driver is None or not self._started:
            self._started = False
            return
        self._driver.stop_realtime()
        self._started = False

    def disconnect(self) -> None:
        driver = self._driver
        if driver is None:
            return
        try:
            if self._started:
                self.stop()
            driver.disconnect()
        finally:
            self._driver = None
            self._binding = None
            self._connected = False
            self._started = False


__all__ = [
    "MockRokaeDriver",
    "RokaeDriverBase",
    "RokaeXCoreDriver",
    "VENDORED_CPP_MODULE_DIR",
]
