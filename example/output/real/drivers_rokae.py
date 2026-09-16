"""ROKAE drivers backed by the vendored xCoreSDK-CPP v0.3.4 pybind module."""

from __future__ import annotations

import sys
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from anydexretarget.teleop.pose import validate_transform
from anydexretarget.teleop.realtime import (
    CartesianSetpointGenerator,
    RealtimeCartesianConfig,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
VENDORED_CPP_MODULE_DIR = PROJECT_ROOT / "rokae" / "python"


def plan_init_joint_targets(
    current: Sequence[float],
    final_target: Sequence[float],
    joint_waypoints: Optional[Sequence[Sequence[float]]],
    max_joint_delta: Optional[float],
    max_segment_delta: Optional[float] = None,
) -> list[np.ndarray]:
    """Plan a guarded, finely segmented init path in joint space."""
    current_array = np.asarray(current, dtype=np.float64)
    final_array = np.asarray(final_target, dtype=np.float64)
    if current_array.ndim != 1 or final_array.shape != current_array.shape:
        raise ValueError("current and init joint targets must have the same 1-D shape")
    if not np.all(np.isfinite(current_array)) or not np.all(np.isfinite(final_array)):
        raise ValueError("init joint targets must be finite")
    limit = 1e9 if max_joint_delta is None else float(max_joint_delta)
    if not np.isfinite(limit) or limit <= 0.0:
        raise ValueError("max_joint_delta must be positive")

    segment_limit = limit if max_segment_delta is None else float(max_segment_delta)
    if not np.isfinite(segment_limit) or segment_limit <= 0.0:
        raise ValueError("max_segment_delta must be positive")
    if segment_limit > limit:
        raise ValueError("max_segment_delta cannot exceed max_joint_delta")

    waypoints = [] if joint_waypoints is None else list(joint_waypoints)
    cursor = current_array.copy()
    planned: list[np.ndarray] = []

    # A waypoint path is only needed when direct motion exceeds the hard
    # one-shot guard.  If it is needed, preserve every configured waypoint;
    # silently skipping a later waypoint can recreate a large final segment.
    direct_delta = float(np.max(np.abs(final_array - cursor)))
    coarse_targets: list[np.ndarray] = []
    if direct_delta > limit:
        if not waypoints:
            raise RuntimeError(
                f"init path needs a waypoint: direct delta {direct_delta:.6f} rad "
                f"exceeds max_joint_delta {limit:.6f} rad"
            )
        for waypoint in waypoints:
            waypoint_array = np.asarray(waypoint, dtype=np.float64)
            if waypoint_array.shape != cursor.shape or not np.all(
                np.isfinite(waypoint_array)
            ):
                raise ValueError("each init joint waypoint must match the robot DoF")
            coarse_targets.append(waypoint_array)
    coarse_targets.append(final_array)

    for coarse_index, coarse_target in enumerate(coarse_targets, start=1):
        delta = float(np.max(np.abs(coarse_target - cursor)))
        if delta > limit:
            raise RuntimeError(
                f"init coarse segment {coarse_index} delta {delta:.6f} rad exceeds "
                f"max_joint_delta {limit:.6f} rad"
            )
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


class MockRokaeDriver(RokaeDriverBase):
    """Hardware-free backend using the exact production setpoint generator."""

    def __init__(
        self,
        initial_pose: np.ndarray,
        control_hz: float = 1000.0,
        command_timeout: float = 0.20,
        realtime_config: Optional[RealtimeCartesianConfig] = None,
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

    @property
    def command_count(self) -> int:
        with self._lock:
            return self._command_count

    @property
    def timed_out(self) -> bool:
        return self._last_step_state in {"holding_stale", "safe_stop"}

    def connect(self) -> None:
        self._connected = True

    def move_to_init(self, joint_position: Optional[Sequence[float]], **_: Any) -> None:
        del joint_position

    def start(self, power_on: bool = True) -> None:
        del power_on
        if not self._connected:
            raise RuntimeError("Mock ROKAE driver is not connected")
        if self._running:
            return
        now = time.monotonic()
        self._generator.reset(self._current_pose, now)
        self._stop_event.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._control_loop, name="mock-rokae-rt", daemon=True
        )
        self._thread.start()

    def _control_loop(self) -> None:
        period = 1.0 / self.realtime_config.control_hz
        deadline = time.monotonic()
        while not self._stop_event.is_set():
            step = self._generator.step()
            with self._lock:
                self._current_pose = step.pose
                self._last_step_state = step.state
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

    def get_controller_base_frame(self) -> np.ndarray:
        return np.eye(4, dtype=np.float64)

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """Mock has no kinematic model or synthetic joint state."""
        return None

    def get_command_pose(self) -> np.ndarray:
        return self._generator.pose

    def set_target_pose(
        self,
        base_T_tcp_target: np.ndarray,
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
    ) -> bool:
        if not self._running:
            raise RuntimeError("Mock ROKAE realtime loop is not running")
        return self._generator.publish_target(
            base_T_tcp_target,
            source_timestamp=source_timestamp,
            sequence=sequence,
        )

    def hold(self) -> None:
        if self._running:
            self._generator.publish_target(self._generator.pose)

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
        self._binding: Any = None
        self._driver: Any = None
        self._connected = False
        self._started = False
        self._last_command_pose: Optional[np.ndarray] = None
        self._next_sequence = 0

    def _import_binding(self) -> Any:
        candidates = list(self.module_path.glob("_rokae_cpp*.so"))
        if not candidates:
            raise FileNotFoundError(
                f"ROKAE C++ module is not built under {self.module_path}; "
                "run: bash rokae/build.sh"
            )
        path_text = str(self.module_path.resolve())
        if path_text not in sys.path:
            sys.path.insert(0, path_text)
        try:
            import _rokae_cpp  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError(
                "failed to load the ROKAE C++ bridge; rebuild with the active "
                "Python and verify the vendored aarch64 runtime"
            ) from exc
        return _rokae_cpp

    def _cpp_config(self, binding: Any) -> Any:
        cfg = binding.RealtimeConfig()
        cfg.nrt_online_speed_scale = self.nrt_online_speed_scale
        for name in (
            "translation_cutoff_hz",
            "rotation_cutoff_hz",
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
        limit = 1e9 if max_joint_delta is None else float(max_joint_delta)
        speed = int(np.clip(round(joint_speed * 1000.0), 5, 1000))
        current = self.get_joint_positions()
        if current is None:
            raise RuntimeError("ROKAE did not return current joints for init planning")
        planned = plan_init_joint_targets(
            current, target, joint_waypoints, limit, max_segment_delta
        )
        if not planned:
            print("[ROKAE] already at configured init; no MoveAbsJ required")
            return
        for index, segment_target in enumerate(planned, start=1):
            print(
                f"[ROKAE] init segment {index}/{len(planned)} target="
                + np.array2string(segment_target, precision=6, separator=",")
            )
            self._driver.move_to_init(
                segment_target.tolist(), speed, float(timeout), limit, bool(power_on)
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

    def start(self, power_on: bool = True) -> None:
        if not self._connected or self._driver is None:
            raise RuntimeError("ROKAE driver is not connected")
        if self._started:
            return
        self._driver.start_realtime_cartesian(bool(power_on))
        self._started = True

    def get_command_pose(self) -> np.ndarray:
        if self._driver is None:
            if self._last_command_pose is None:
                raise RuntimeError("ROKAE command pose is unavailable")
            return self._last_command_pose.copy()
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
