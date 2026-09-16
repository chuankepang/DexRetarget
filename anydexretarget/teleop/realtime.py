"""Fixed-rate Cartesian setpoint generation for realtime teleoperation.

The producer (Quest/network thread) only replaces one latest sample.  The
consumer (robot control callback) runs at its own fixed rate and turns that
sample into a smooth, bounded SE(3) command.  No input packet is ever queued
for later replay.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .pose import (
    interpolate_rotation,
    make_transform,
    matrix_to_rotation_vector,
    rotation_vector_to_matrix,
    validate_transform,
)


def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm < 1e-15:
        return vector
    return vector * (maximum / norm)


@dataclass(frozen=True)
class CartesianTargetSample:
    """One producer sample with ordering and freshness metadata."""

    pose: np.ndarray
    sequence: int
    source_timestamp: float
    receive_timestamp: float


@dataclass(frozen=True)
class LatestTargetStats:
    accepted: int
    rejected_sequence: int
    rejected_source_time: int
    rejected_source_age: int
    latest_sequence: Optional[int]


class LatestTargetBuffer:
    """Thread-safe latest-value slot; deliberately not a FIFO."""

    def __init__(
        self,
        max_source_age: float = 0.20,
        future_tolerance: float = 0.05,
    ) -> None:
        if max_source_age <= 0.0 or future_tolerance < 0.0:
            raise ValueError("invalid latest-target timing limits")
        self.max_source_age = float(max_source_age)
        self.future_tolerance = float(future_tolerance)
        self._sample: Optional[CartesianTargetSample] = None
        self._next_sequence = 0
        self._accepted = 0
        self._rejected_sequence = 0
        self._rejected_source_time = 0
        self._rejected_source_age = 0
        self._lock = threading.Lock()

    def publish(
        self,
        pose: np.ndarray,
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
        receive_timestamp: Optional[float] = None,
    ) -> bool:
        """Replace the slot, rejecting late, duplicate, and stale samples."""
        target = validate_transform(pose, "Cartesian latest target")
        received = (
            time.monotonic() if receive_timestamp is None else float(receive_timestamp)
        )
        source = received if source_timestamp is None else float(source_timestamp)
        if not math.isfinite(source) or not math.isfinite(received):
            raise ValueError("target timestamps must be finite")
        age = received - source
        with self._lock:
            if age > self.max_source_age or age < -self.future_tolerance:
                self._rejected_source_age += 1
                return False
            seq = self._next_sequence if sequence is None else int(sequence)
            if self._sample is not None:
                if seq <= self._sample.sequence:
                    self._rejected_sequence += 1
                    return False
                if source < self._sample.source_timestamp:
                    self._rejected_source_time += 1
                    return False
            self._sample = CartesianTargetSample(target, seq, source, received)
            self._next_sequence = max(self._next_sequence, seq + 1)
            self._accepted += 1
            return True

    def snapshot(self) -> Optional[CartesianTargetSample]:
        with self._lock:
            sample = self._sample
            if sample is None:
                return None
            return CartesianTargetSample(
                sample.pose.copy(),
                sample.sequence,
                sample.source_timestamp,
                sample.receive_timestamp,
            )

    @property
    def stats(self) -> LatestTargetStats:
        with self._lock:
            return LatestTargetStats(
                accepted=self._accepted,
                rejected_sequence=self._rejected_sequence,
                rejected_source_time=self._rejected_source_time,
                rejected_source_age=self._rejected_source_age,
                latest_sequence=None if self._sample is None else self._sample.sequence,
            )


@dataclass(frozen=True)
class RealtimeCartesianConfig:
    """Limits evaluated at the robot callback rate, not the Quest rate."""

    control_hz: float = 1000.0
    translation_cutoff_hz: float = 3.0
    rotation_cutoff_hz: float = 3.0
    translation_deadband: float = 0.002
    rotation_deadband: float = 0.015
    max_translation_speed: float = 0.10
    max_angular_speed: float = 0.35
    max_translation_acceleration: float = 0.25
    max_angular_acceleration: float = 1.0
    max_translation_jerk: float = 1.5
    max_angular_jerk: float = 6.0
    max_target_translation_delta: float = 0.25
    max_target_rotation_delta: float = 1.0
    workspace_min: tuple[float, float, float] = (-1.2, -1.2, -1.2)
    workspace_max: tuple[float, float, float] = (1.2, 1.2, 1.2)
    hold_timeout: float = 0.15
    stop_timeout: float = 0.60
    max_source_age: float = 0.20
    future_tolerance: float = 0.05

    def __post_init__(self) -> None:
        positive = (
            "control_hz",
            "translation_cutoff_hz",
            "rotation_cutoff_hz",
            "max_translation_speed",
            "max_angular_speed",
            "max_translation_acceleration",
            "max_angular_acceleration",
            "max_translation_jerk",
            "max_angular_jerk",
            "max_target_translation_delta",
            "max_target_rotation_delta",
            "hold_timeout",
            "stop_timeout",
            "max_source_age",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.stop_timeout <= self.hold_timeout:
            raise ValueError("stop_timeout must be greater than hold_timeout")
        for name in (
            "translation_deadband",
            "rotation_deadband",
            "future_tolerance",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        lower = np.asarray(self.workspace_min, dtype=np.float64)
        upper = np.asarray(self.workspace_max, dtype=np.float64)
        if (
            lower.shape != (3,)
            or upper.shape != (3,)
            or not np.all(np.isfinite(lower))
            or not np.all(np.isfinite(upper))
            or np.any(lower >= upper)
        ):
            raise ValueError("workspace bounds must be ordered 3-vectors")


@dataclass(frozen=True)
class RealtimeStep:
    pose: np.ndarray
    state: str
    sequence: Optional[int]
    input_age: float
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    angular_acceleration: np.ndarray


class CartesianSetpointGenerator:
    """SE(3) low-pass filter and jerk-limited fixed-rate interpolator."""

    def __init__(self, config: RealtimeCartesianConfig) -> None:
        self.config = config
        self.buffer = LatestTargetBuffer(
            max_source_age=config.max_source_age,
            future_tolerance=config.future_tolerance,
        )
        self._pose: Optional[np.ndarray] = None
        self._filtered: Optional[np.ndarray] = None
        self._last_step_time: Optional[float] = None
        self._linear_velocity = np.zeros(3)
        self._angular_velocity = np.zeros(3)
        self._linear_acceleration = np.zeros(3)
        self._angular_acceleration = np.zeros(3)
        self._state = "not_initialized"

    @property
    def state(self) -> str:
        return self._state

    @property
    def pose(self) -> np.ndarray:
        if self._pose is None:
            raise RuntimeError("Cartesian setpoint generator is not initialized")
        return self._pose.copy()

    def reset(self, pose: np.ndarray, timestamp: Optional[float] = None) -> None:
        initial = validate_transform(pose, "realtime initial TCP")
        lower = np.asarray(self.config.workspace_min)
        upper = np.asarray(self.config.workspace_max)
        if np.any(initial[:3, 3] < lower) or np.any(initial[:3, 3] > upper):
            raise ValueError(
                "initial TCP lies outside the configured base-frame workspace"
            )
        self._pose = initial
        self._filtered = initial.copy()
        reset_time = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(reset_time):
            raise ValueError("realtime reset timestamp must be finite")
        self._last_step_time = reset_time
        self._linear_velocity.fill(0.0)
        self._angular_velocity.fill(0.0)
        self._linear_acceleration.fill(0.0)
        self._angular_acceleration.fill(0.0)
        self._state = "holding_no_input"

    def publish_target(
        self,
        pose: np.ndarray,
        *,
        source_timestamp: Optional[float] = None,
        sequence: Optional[int] = None,
        receive_timestamp: Optional[float] = None,
    ) -> bool:
        return self.buffer.publish(
            pose,
            source_timestamp=source_timestamp,
            sequence=sequence,
            receive_timestamp=receive_timestamp,
        )

    @staticmethod
    def _alpha(cutoff_hz: float, dt: float) -> float:
        return 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)

    def _bounded_vector_state(
        self,
        error: np.ndarray,
        velocity: np.ndarray,
        acceleration: np.ndarray,
        dt: float,
        max_velocity: float,
        max_acceleration: float,
        max_jerk: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Overdamped position servo.  The previous implementation always
        # accelerated toward error/dt and did not plan braking, so a fixed
        # target produced a sustained limit cycle.  A damping ratio > 1 gives
        # monotonic settling while retaining explicit velocity/accel/jerk
        # bounds for moving Quest targets.
        damping_ratio = 1.25
        natural_frequency = min(
            2.0 * max_acceleration / max_velocity,
            0.5 / dt,
        )
        desired_acceleration = _limit_norm(
            natural_frequency * natural_frequency * error
            - 2.0 * damping_ratio * natural_frequency * velocity,
            max_acceleration,
        )
        jerk = _limit_norm((desired_acceleration - acceleration) / dt, max_jerk)
        new_acceleration = _limit_norm(acceleration + jerk * dt, max_acceleration)
        new_velocity = _limit_norm(velocity + new_acceleration * dt, max_velocity)
        step = new_velocity * dt
        return step, new_velocity, new_acceleration

    def step(self, timestamp: Optional[float] = None) -> RealtimeStep:
        if self._pose is None or self._filtered is None or self._last_step_time is None:
            raise RuntimeError("Cartesian setpoint generator is not initialized")
        now = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(now):
            raise ValueError("realtime step timestamp must be finite")
        nominal_dt = 1.0 / self.config.control_hz
        dt = float(
            np.clip(now - self._last_step_time, nominal_dt * 0.25, nominal_dt * 4.0)
        )
        self._last_step_time = now

        sample = self.buffer.snapshot()
        input_age = (
            math.inf if sample is None else max(0.0, now - sample.receive_timestamp)
        )
        if sample is None:
            state = "holding_no_input"
            desired = self._pose.copy()
        elif input_age >= self.config.stop_timeout:
            state = "safe_stop"
            desired = self._pose.copy()
        elif input_age >= self.config.hold_timeout:
            state = "holding_stale"
            desired = self._pose.copy()
        else:
            state = "tracking"
            desired = sample.pose.copy()

        lower = np.asarray(self.config.workspace_min)
        upper = np.asarray(self.config.workspace_max)
        desired[:3, 3] = np.clip(desired[:3, 3], lower, upper)

        position_error = desired[:3, 3] - self._filtered[:3, 3]
        if np.linalg.norm(position_error) >= self.config.translation_deadband:
            alpha = self._alpha(self.config.translation_cutoff_hz, dt)
            self._filtered[:3, 3] += alpha * position_error

        rotation_error = matrix_to_rotation_vector(
            self._filtered[:3, :3].T @ desired[:3, :3]
        )
        if np.linalg.norm(rotation_error) >= self.config.rotation_deadband:
            self._filtered[:3, :3] = interpolate_rotation(
                self._filtered[:3, :3],
                desired[:3, :3],
                self._alpha(self.config.rotation_cutoff_hz, dt),
            )

        linear_error = self._filtered[:3, 3] - self._pose[:3, 3]
        linear_step, self._linear_velocity, self._linear_acceleration = (
            self._bounded_vector_state(
                linear_error,
                self._linear_velocity,
                self._linear_acceleration,
                dt,
                self.config.max_translation_speed,
                self.config.max_translation_acceleration,
                self.config.max_translation_jerk,
            )
        )
        angular_error = matrix_to_rotation_vector(
            self._pose[:3, :3].T @ self._filtered[:3, :3]
        )
        angular_step, self._angular_velocity, self._angular_acceleration = (
            self._bounded_vector_state(
                angular_error,
                self._angular_velocity,
                self._angular_acceleration,
                dt,
                self.config.max_angular_speed,
                self.config.max_angular_acceleration,
                self.config.max_angular_jerk,
            )
        )

        self._pose = make_transform(
            self._pose[:3, 3] + linear_step,
            self._pose[:3, :3] @ rotation_vector_to_matrix(angular_step),
        )
        self._pose[:3, 3] = np.clip(self._pose[:3, 3], lower, upper)
        self._state = state
        return RealtimeStep(
            pose=self._pose.copy(),
            state=state,
            sequence=None if sample is None else sample.sequence,
            input_age=input_age,
            linear_velocity=self._linear_velocity.copy(),
            angular_velocity=self._angular_velocity.copy(),
            linear_acceleration=self._linear_acceleration.copy(),
            angular_acceleration=self._angular_acceleration.copy(),
        )


__all__ = [
    "CartesianSetpointGenerator",
    "CartesianTargetSample",
    "LatestTargetBuffer",
    "LatestTargetStats",
    "RealtimeCartesianConfig",
    "RealtimeStep",
]
