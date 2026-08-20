"""Relative Cartesian mapping and safety limiting for VR teleoperation."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .pose import (
    interpolate_rotation,
    invert_transform,
    make_transform,
    matrix_to_rotation_vector,
    rotation_vector_to_matrix,
    validate_rotation,
    validate_transform,
)


@dataclass(frozen=True)
class PoseMappingResult:
    """Intermediate transforms produced by reference-based pose mapping."""

    vr_relative: np.ndarray
    mapped_relative: np.ndarray
    world_target: np.ndarray
    base_target: np.ndarray


class RelativePoseMapper:
    """Map Quest motion relative to a reference into a robot TCP target.

    ``quest_to_robot_rotation`` maps axes from the Quest reference frame into
    the robot-reference incremental frame. ``T_world_base`` is the robot base
    pose in the shared world frame and supports upright, inverted, or tilted
    mounting without scattered axis sign changes.
    """

    def __init__(
        self,
        quest_to_robot_rotation: np.ndarray,
        T_world_base: np.ndarray,
        translation_scale: float = 1.0,
        rotation_scale: float = 1.0,
    ) -> None:
        if not np.isfinite(translation_scale) or translation_scale <= 0.0:
            raise ValueError("translation_scale must be positive")
        if not np.isfinite(rotation_scale) or rotation_scale < 0.0:
            raise ValueError("rotation_scale cannot be negative")
        self.quest_to_robot_rotation = validate_rotation(
            quest_to_robot_rotation, "quest_to_robot_rotation"
        )
        self.T_world_base = validate_transform(T_world_base, "T_world_base")
        self.T_base_world = invert_transform(self.T_world_base)
        self.translation_scale = float(translation_scale)
        self.rotation_scale = float(rotation_scale)
        self._T_vr_ref: Optional[np.ndarray] = None
        self._T_world_tcp_ref: Optional[np.ndarray] = None
        self._last_result: Optional[PoseMappingResult] = None

    @property
    def referenced(self) -> bool:
        return self._T_vr_ref is not None

    @property
    def last_result(self) -> Optional[PoseMappingResult]:
        return self._copy_result(self._last_result)

    @staticmethod
    def _copy_result(
        result: Optional[PoseMappingResult],
    ) -> Optional[PoseMappingResult]:
        if result is None:
            return None
        return PoseMappingResult(
            result.vr_relative.copy(),
            result.mapped_relative.copy(),
            result.world_target.copy(),
            result.base_target.copy(),
        )

    def set_reference(self, T_vr_ref: np.ndarray, T_base_tcp_ref: np.ndarray) -> None:
        """Set simultaneous Quest and robot references without commanding motion."""
        self._T_vr_ref = validate_transform(T_vr_ref, "T_vr_ref")
        base_ref = validate_transform(T_base_tcp_ref, "T_base_tcp_ref")
        self._T_world_tcp_ref = self.T_world_base @ base_ref
        identity = np.eye(4, dtype=np.float64)
        self._last_result = PoseMappingResult(
            identity.copy(), identity.copy(), self._T_world_tcp_ref.copy(), base_ref
        )

    def recenter(self, T_vr_current: np.ndarray, T_base_tcp_hold: np.ndarray) -> None:
        """Use the current VR pose as a new reference while preserving target."""
        self.set_reference(T_vr_current, T_base_tcp_hold)

    def update(self, T_vr_current: np.ndarray) -> PoseMappingResult:
        if self._T_vr_ref is None or self._T_world_tcp_ref is None:
            raise RuntimeError("RelativePoseMapper reference has not been initialized")
        current = validate_transform(T_vr_current, "T_vr_current")

        # Required body-relative convention: inv(T_vr_ref) @ T_vr_current.
        vr_relative = invert_transform(self._T_vr_ref) @ current
        mapped_relative = np.eye(4, dtype=np.float64)
        mapped_relative[:3, 3] = (
            self.translation_scale * self.quest_to_robot_rotation @ vr_relative[:3, 3]
        )
        mapped_rotation = (
            self.quest_to_robot_rotation
            @ vr_relative[:3, :3]
            @ self.quest_to_robot_rotation.T
        )
        mapped_rotvec = matrix_to_rotation_vector(mapped_rotation)
        mapped_relative[:3, :3] = rotation_vector_to_matrix(
            self.rotation_scale * mapped_rotvec
        )

        world_target = self._T_world_tcp_ref @ mapped_relative
        base_target = self.T_base_world @ world_target
        result = PoseMappingResult(
            vr_relative=vr_relative,
            mapped_relative=mapped_relative,
            world_target=world_target,
            base_target=base_target,
        )
        self._last_result = result
        return self._copy_result(result)  # type: ignore[return-value]


@dataclass(frozen=True)
class PoseSafetyConfig:
    """Software limits applied before a target reaches the robot driver."""

    max_translation_speed: float = 0.20
    max_angular_speed: float = 0.80
    max_translation_delta: float = 0.005
    max_rotation_delta: float = 0.02
    low_pass_alpha: float = 0.25
    workspace_min: tuple[float, float, float] = (-1.2, -1.2, -0.1)
    workspace_max: tuple[float, float, float] = (1.2, 1.2, 1.5)

    def __post_init__(self) -> None:
        positive = {
            "max_translation_speed": self.max_translation_speed,
            "max_angular_speed": self.max_angular_speed,
            "max_translation_delta": self.max_translation_delta,
            "max_rotation_delta": self.max_rotation_delta,
        }
        for name, value in positive.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if not 0.0 < self.low_pass_alpha <= 1.0:
            raise ValueError("low_pass_alpha must be in (0, 1]")
        lower = np.asarray(self.workspace_min, dtype=np.float64)
        upper = np.asarray(self.workspace_max, dtype=np.float64)
        if lower.shape != (3,) or upper.shape != (3,):
            raise ValueError("workspace bounds must each contain three values")
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
            raise ValueError("workspace bounds must be finite")
        if np.any(lower >= upper):
            raise ValueError("workspace_min must be smaller than workspace_max")


class PoseSafetyLimiter:
    """Low-pass, workspace, Cartesian speed, and per-frame pose limiter."""

    def __init__(self, config: PoseSafetyConfig) -> None:
        self.config = config
        self._last_output: Optional[np.ndarray] = None
        self._last_time: Optional[float] = None

    @property
    def last_output(self) -> Optional[np.ndarray]:
        return None if self._last_output is None else self._last_output.copy()

    def reset(self, pose: np.ndarray, timestamp: float) -> np.ndarray:
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        initial = validate_transform(pose, "safety reset pose")
        lower = np.asarray(self.config.workspace_min)
        upper = np.asarray(self.config.workspace_max)
        if np.any(initial[:3, 3] < lower) or np.any(initial[:3, 3] > upper):
            raise ValueError(
                "Current robot TCP is outside the configured base-frame workspace; "
                "refusing to create a target that could jump to a boundary"
            )
        self._last_output = initial
        self._last_time = float(timestamp)
        return self._last_output.copy()

    def apply(self, target: np.ndarray, timestamp: float) -> np.ndarray:
        raw = validate_transform(target, "raw target")
        if self._last_output is None or self._last_time is None:
            return self.reset(raw, timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")

        dt = max(float(timestamp) - self._last_time, 1e-6)
        previous = self._last_output
        alpha = self.config.low_pass_alpha

        filtered_position = previous[:3, 3] + alpha * (raw[:3, 3] - previous[:3, 3])
        filtered_rotation = interpolate_rotation(previous[:3, :3], raw[:3, :3], alpha)

        translation_step = filtered_position - previous[:3, 3]
        translation_norm = float(np.linalg.norm(translation_step))
        max_translation_step = min(
            self.config.max_translation_delta,
            self.config.max_translation_speed * dt,
        )
        if translation_norm > max_translation_step:
            translation_step *= max_translation_step / translation_norm

        rotation_step = matrix_to_rotation_vector(
            previous[:3, :3].T @ filtered_rotation
        )
        rotation_norm = float(np.linalg.norm(rotation_step))
        max_rotation_step = min(
            self.config.max_rotation_delta,
            self.config.max_angular_speed * dt,
        )
        if rotation_norm > max_rotation_step:
            rotation_step *= max_rotation_step / rotation_norm

        output = make_transform(
            translation=previous[:3, 3] + translation_step,
            rotation=previous[:3, :3] @ rotation_vector_to_matrix(rotation_step),
        )
        output[:3, 3] = np.clip(
            output[:3, 3],
            np.asarray(self.config.workspace_min),
            np.asarray(self.config.workspace_max),
        )
        self._last_output = output
        self._last_time = float(timestamp)
        return output.copy()


@dataclass(frozen=True)
class ArmCommand:
    """One complete arm mapping result sent to a latest-value driver buffer."""

    timestamp: float
    status: str
    vr_pose: np.ndarray
    vr_relative: np.ndarray
    world_target: np.ndarray
    base_target_raw: np.ndarray
    base_target_safe: np.ndarray


class ArmTeleopController:
    """Thread-safe reference mapping, safety limiting, clutch, and watchdog."""

    def __init__(
        self,
        mapper: RelativePoseMapper,
        limiter: PoseSafetyLimiter,
        command_timeout: float = 0.20,
        enabled: bool = True,
    ) -> None:
        if not np.isfinite(command_timeout) or command_timeout <= 0.0:
            raise ValueError("command_timeout must be positive")
        self.mapper = mapper
        self.limiter = limiter
        self.command_timeout = float(command_timeout)
        self._enabled = bool(enabled)
        self._status = "waiting_reference"
        self._last_input_time: Optional[float] = None
        self._last_vr_pose: Optional[np.ndarray] = None
        self._last_command: Optional[ArmCommand] = None
        self._needs_recenter = False
        self._lock = threading.RLock()

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @property
    def status(self) -> str:
        with self._lock:
            return self._status

    @property
    def last_command(self) -> Optional[ArmCommand]:
        with self._lock:
            command = self._last_command
            if command is None:
                return None
            return ArmCommand(
                command.timestamp,
                command.status,
                command.vr_pose.copy(),
                command.vr_relative.copy(),
                command.world_target.copy(),
                command.base_target_raw.copy(),
                command.base_target_safe.copy(),
            )

    def initialize(
        self,
        T_vr_ref: np.ndarray,
        T_base_tcp_ref: np.ndarray,
        timestamp: float,
    ) -> ArmCommand:
        with self._lock:
            vr_ref = validate_transform(T_vr_ref, "T_vr_ref")
            base_ref = validate_transform(T_base_tcp_ref, "T_base_tcp_ref")
            self.mapper.set_reference(vr_ref, base_ref)
            self.limiter.reset(base_ref, timestamp)
            mapping = self.mapper.last_result
            assert mapping is not None
            self._last_input_time = float(timestamp)
            self._last_vr_pose = vr_ref
            self._needs_recenter = False
            self._status = "active" if self._enabled else "hold_disabled"
            self._last_command = ArmCommand(
                timestamp=float(timestamp),
                status=self._status,
                vr_pose=vr_ref,
                vr_relative=mapping.vr_relative,
                world_target=mapping.world_target,
                base_target_raw=base_ref,
                base_target_safe=base_ref,
            )
            return self.last_command  # type: ignore[return-value]

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            requested = bool(enabled)
            if requested == self._enabled:
                return
            self._enabled = requested
            self._needs_recenter = True
            self._status = "waiting_recenter" if requested else "hold_disabled"

    def recenter(self, T_vr_current: Optional[np.ndarray] = None) -> bool:
        with self._lock:
            if self._last_command is None:
                return False
            vr_pose = self._last_vr_pose if T_vr_current is None else T_vr_current
            if vr_pose is None:
                return False
            hold = self._last_command.base_target_safe
            self.mapper.recenter(vr_pose, hold)
            timestamp = self._last_input_time
            assert timestamp is not None
            self.limiter.reset(hold, timestamp)
            self._needs_recenter = False
            self._status = "active" if self._enabled else "hold_disabled"
            return True

    def update(self, T_vr_current: np.ndarray, timestamp: float) -> ArmCommand:
        with self._lock:
            if self._last_command is None:
                raise RuntimeError("ArmTeleopController must be initialized first")
            vr_pose = validate_transform(T_vr_current, "T_vr_current")
            if self._needs_recenter:
                self.mapper.recenter(vr_pose, self._last_command.base_target_safe)
                self.limiter.reset(self._last_command.base_target_safe, timestamp)
                self._needs_recenter = False

            self._last_input_time = float(timestamp)
            self._last_vr_pose = vr_pose
            if not self._enabled:
                self._status = "hold_disabled"
                self._last_command = self._hold_command(timestamp, vr_pose)
                return self.last_command  # type: ignore[return-value]

            mapping = self.mapper.update(vr_pose)
            safe_target = self.limiter.apply(mapping.base_target, timestamp)
            self._status = "active"
            self._last_command = ArmCommand(
                timestamp=float(timestamp),
                status=self._status,
                vr_pose=vr_pose,
                vr_relative=mapping.vr_relative,
                world_target=mapping.world_target,
                base_target_raw=mapping.base_target,
                base_target_safe=safe_target,
            )
            return self.last_command  # type: ignore[return-value]

    def poll(self, timestamp: float) -> Optional[ArmCommand]:
        """Apply the watchdog even when no new Quest packet arrives."""
        with self._lock:
            if self._last_command is None or self._last_input_time is None:
                return None
            if float(timestamp) - self._last_input_time <= self.command_timeout:
                return self.last_command
            if self._status != "hold_timeout":
                self._status = "hold_timeout"
                self._needs_recenter = True
                self._last_command = self._hold_command(
                    timestamp,
                    self._last_vr_pose if self._last_vr_pose is not None else np.eye(4),
                )
            return self.last_command

    def _hold_command(self, timestamp: float, vr_pose: np.ndarray) -> ArmCommand:
        assert self._last_command is not None
        previous = self._last_command
        return ArmCommand(
            timestamp=float(timestamp),
            status=self._status,
            vr_pose=validate_transform(vr_pose, "hold VR pose"),
            vr_relative=previous.vr_relative.copy(),
            world_target=previous.world_target.copy(),
            base_target_raw=previous.base_target_raw.copy(),
            base_target_safe=previous.base_target_safe.copy(),
        )
