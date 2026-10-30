"""1 kHz joint reference filter and jerk-limited servo.

Positive response_hz selects a third-order reference tracker with acceleration
feedback and bounded bandwidth; zero preserves the legacy second-order tracker.
Both use a low-passed reference and velocity/acceleration/jerk limits. This is
reference generation, not the robot's physical impedance or damping controller.
The recurrence matches ``JointRealtimeCore`` for offline verification.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .model import StreamLimits

_ZETA = 1.25


@dataclass
class JointStreamConfig:
    max_velocity: np.ndarray
    max_acceleration: np.ndarray
    max_jerk: np.ndarray
    position_min: np.ndarray
    position_max: np.ndarray
    reference_cutoff_hz: float = 15.0
    deadband: float = 1e-4
    max_target_delta: float = 0.35
    hold_timeout: float = 0.15
    stop_timeout: float = 0.60
    max_source_age: float = 0.20
    future_tolerance: float = 0.05
    response_hz: float = 0.0

    @classmethod
    def from_limits(cls, limits: StreamLimits) -> "JointStreamConfig":
        return cls(
            max_velocity=np.asarray(limits.max_velocity, dtype=np.float64).copy(),
            max_acceleration=np.asarray(limits.max_acceleration, dtype=np.float64).copy(),
            max_jerk=np.asarray(limits.max_jerk, dtype=np.float64).copy(),
            position_min=np.asarray(limits.position_min, dtype=np.float64).copy(),
            position_max=np.asarray(limits.position_max, dtype=np.float64).copy(),
            reference_cutoff_hz=float(limits.reference_cutoff_hz),
            deadband=float(limits.deadband),
            max_target_delta=float(limits.max_target_delta),
            hold_timeout=float(limits.hold_timeout),
            stop_timeout=float(limits.stop_timeout),
            max_source_age=float(limits.max_source_age),
            future_tolerance=float(limits.future_tolerance),
            response_hz=float(limits.response_hz),
        )

    def validate(self) -> None:
        if not math.isfinite(self.response_hz) or not 0 <= self.response_hz <= 5:
            raise ValueError("response_hz must be finite and in [0, 5]")
        vectors = (
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.position_min,
            self.position_max,
        )
        dof = int(self.max_velocity.size)
        if dof < 1 or any(np.asarray(value).shape != (dof,) for value in vectors):
            raise ValueError("joint stream vectors must share one positive length")
        if np.any(self.max_velocity <= 0) or np.any(self.max_acceleration <= 0):
            raise ValueError("joint velocity and acceleration limits must be positive")
        if np.any(self.max_jerk <= 0):
            raise ValueError("joint jerk limits must be positive")
        if np.any(self.position_min >= self.position_max):
            raise ValueError("joint position limits must be ordered")
        if self.reference_cutoff_hz <= 0 or self.max_target_delta <= 0:
            raise ValueError("cutoff and max_target_delta must be positive")
        if self.deadband < 0 or self.future_tolerance < 0:
            raise ValueError("deadband and future tolerance must be non-negative")
        if self.stop_timeout <= self.hold_timeout or self.max_source_age <= 0:
            raise ValueError("joint stream timeouts are inconsistent")
        finite = np.concatenate(vectors)
        if not np.all(np.isfinite(finite)):
            raise ValueError("joint stream limits must be finite")

    @property
    def dof(self) -> int:
        return int(self.max_velocity.size)


def _alpha(cutoff_hz: float, dt: float) -> float:
    return 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)


class JointStream:
    """Latest-target joint servo. ``step`` is the 1 ms callback."""

    def __init__(self, config: JointStreamConfig) -> None:
        config.validate()
        self.config = config
        self._latest: np.ndarray | None = None
        self._latest_sequence = -1
        self._latest_source = 0.0
        self._latest_receive = 0.0
        self._has_sample = False
        self.position = np.zeros(config.dof, dtype=np.float64)
        self.filtered = np.zeros(config.dof, dtype=np.float64)
        self.velocity = np.zeros(config.dof, dtype=np.float64)
        self.acceleration = np.zeros(config.dof, dtype=np.float64)
        self._last_time = 0.0
        self._initialized = False
        self._hold = False
        self._stop = False
        self.should_finish = False
        self.watchdog = "not_started"
        self.accepted = 0
        self.rejected_sequence = 0
        self.rejected_source_time = 0
        self.rejected_source_age = 0
        self.rejected_target_delta = 0
        self.callback_count = 0

    def reset(self, joints: np.ndarray, timestamp: float) -> None:
        q = self._finite(joints, "initial joints")
        if np.any(q < self.config.position_min) or np.any(q > self.config.position_max):
            raise ValueError("initial joints are outside the stream limits")
        self.position = q.copy()
        self.filtered = q.copy()
        self.velocity.fill(0.0)
        self.acceleration.fill(0.0)
        self._last_time = float(timestamp)
        self._latest = q.copy()
        self._latest_sequence = -1
        self._latest_source = float(timestamp)
        self._latest_receive = float(timestamp)
        self._has_sample = True
        self._initialized = True
        self._hold = False
        self._stop = False
        self.should_finish = False
        self.watchdog = "holding_no_input"

    def publish(
        self,
        joints: np.ndarray,
        source_timestamp: float,
        sequence: int,
        receive_timestamp: float,
    ) -> bool:
        if not self._initialized or self._latest is None:
            raise RuntimeError("joint stream is not initialized")
        target = self._finite(joints, "joint target")
        source_age = float(receive_timestamp) - float(source_timestamp)
        if (
            source_age > self.config.max_source_age
            or source_age < -self.config.future_tolerance
        ):
            self.rejected_source_age += 1
            return False
        if np.max(np.abs(target - self.position)) > self.config.max_target_delta:
            self.rejected_target_delta += 1
            return False
        if sequence <= self._latest_sequence:
            self.rejected_sequence += 1
            return False
        if source_timestamp < self._latest_source:
            self.rejected_source_time += 1
            return False
        self._latest = target
        self._latest_sequence = int(sequence)
        self._latest_source = float(source_timestamp)
        self._latest_receive = float(receive_timestamp)
        self._has_sample = True
        self._hold = False
        self.accepted += 1
        return True

    def hold(self) -> None:
        self._hold = True

    def request_stop(self) -> None:
        self._stop = True

    def step(self, timestamp: float) -> np.ndarray:
        if not self._initialized:
            raise RuntimeError("joint stream is not initialized")
        dt = float(np.clip(timestamp - self._last_time, 0.00025, 0.004))
        self._last_time = float(timestamp)
        age = (
            max(0.0, timestamp - self._latest_receive)
            if self._has_sample
            else math.inf
        )
        desired = self.position.copy()
        if self._stop or (not self._hold and age >= self.config.stop_timeout):
            self.watchdog = "safe_stop"
            self.should_finish = True
        elif self._hold:
            self.watchdog = "holding_stale"
        elif not self._has_sample or self._latest is None:
            self.watchdog = "holding_no_input"
        elif age >= self.config.hold_timeout:
            self.watchdog = "holding_stale"
        else:
            self.watchdog = "tracking"
            desired = self._latest

        blend = _alpha(self.config.reference_cutoff_hz, dt)
        filter_error = desired - self.filtered
        moving = np.abs(filter_error) >= self.config.deadband
        self.filtered = np.where(
            moving, self.filtered + blend * filter_error, self.filtered
        )

        error = self.filtered - self.position
        error = np.where(np.abs(error) < self.config.deadband, 0.0, error)
        natural = np.minimum(
            (2.0 * math.pi * self.config.response_hz if self.config.response_hz > 0
             else 2.0 * self.config.max_acceleration / self.config.max_velocity),
            0.5 / dt,
        )
        if self.config.response_hz > 0:
            # Third-order reference tracking includes acceleration feedback.
            # Reduce bandwidth for large errors/state derivatives before jerk
            # saturation; merely increasing a second-order gain can oscillate.
            natural = np.minimum.reduce([
                np.full(self.config.dof, natural),
                np.cbrt(2*self.config.max_jerk / np.maximum(np.abs(error), 1e-12)),
                np.sqrt(2*self.config.max_jerk / np.maximum(3*np.abs(self.velocity), 1e-12)),
                2*self.config.max_jerk / np.maximum(3*np.abs(self.acceleration), 1e-12)])
            jerk = np.clip(natural**3*error - 3*natural**2*self.velocity
                           - 3*natural*self.acceleration,
                           -self.config.max_jerk, self.config.max_jerk)
        else:
            desired_acceleration = np.clip(
                natural * natural * error - 2.0 * _ZETA * natural * self.velocity,
                -self.config.max_acceleration, self.config.max_acceleration)
            jerk = np.clip((desired_acceleration - self.acceleration) / dt,
                           -self.config.max_jerk, self.config.max_jerk)
        self.acceleration = np.clip(
            self.acceleration + jerk * dt,
            -self.config.max_acceleration,
            self.config.max_acceleration,
        )
        self.velocity = np.clip(
            self.velocity + self.acceleration * dt,
            -self.config.max_velocity,
            self.config.max_velocity,
        )
        self.position = self.position + self.velocity * dt
        low = self.position < self.config.position_min
        high = self.position > self.config.position_max
        self.position = np.clip(
            self.position, self.config.position_min, self.config.position_max
        )
        self.velocity = np.where(low | high, 0.0, self.velocity)
        self.acceleration = np.where(low | high, 0.0, self.acceleration)
        self.callback_count += 1
        return self.position.copy()

    def diagnostics(self, timestamp: float) -> dict[str, float | int | str]:
        age = (
            max(0.0, float(timestamp) - self._latest_receive)
            if self._has_sample
            else math.inf
        )
        return {
            "watchdog_state": self.watchdog,
            "callback_count": self.callback_count,
            "accepted": self.accepted,
            "rejected_sequence": self.rejected_sequence,
            "rejected_source_time": self.rejected_source_time,
            "rejected_source_age": self.rejected_source_age,
            "rejected_target_delta": self.rejected_target_delta,
            "latest_sequence": self._latest_sequence,
            "latest_command_age": age,
        }

    def _finite(self, joints: np.ndarray, name: str) -> np.ndarray:
        q = np.asarray(joints, dtype=np.float64).reshape(-1)
        if q.shape != (self.config.dof,) or not np.all(np.isfinite(q)):
            raise ValueError(f"{name} must contain {self.config.dof} finite radians")
        return q
