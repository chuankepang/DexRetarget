"""Reference conditioning shared by position and impedance commands.

Low-pass smoothing of a live teleop stream still lives in the C++ realtime
core. This module is the separate Python stage for rate limits, workspace
clipping, and equilibrium limiting. Equilibrium limiting is not filtering.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rokae.control.reference_limit import (
    limit_equilibrium,
    rotation_from_vector,
    rotation_vector,
)


@dataclass
class RateLimit:
    max_translation_speed: float = 0.15
    max_angular_speed: float = 0.50
    max_translation_acceleration: float | None = 0.40
    max_angular_acceleration: float | None = 1.50
    max_dt: float = 0.05


class PoseRateLimiter:
    """Caps how fast a pose reference may change. A stale gap does not jump."""

    def __init__(self, limits: RateLimit | None = None) -> None:
        self.limits = limits or RateLimit()
        self._pose: np.ndarray | None = None
        self._linear_velocity = np.zeros(3, dtype=np.float64)
        self._angular_velocity = np.zeros(3, dtype=np.float64)
        self._timestamp: float | None = None

    def reset(self, pose: np.ndarray, timestamp: float) -> None:
        self._pose = np.asarray(pose, dtype=np.float64).copy()
        self._timestamp = float(timestamp)
        self._linear_velocity[:] = 0.0
        self._angular_velocity[:] = 0.0

    def apply(self, pose: np.ndarray, timestamp: float) -> np.ndarray:
        target = np.asarray(pose, dtype=np.float64)
        if self._pose is None or self._timestamp is None:
            self.reset(target, timestamp)
            return self._pose.copy()
        dt = float(timestamp - self._timestamp)
        if not np.isfinite(dt) or dt <= 0.0:
            return self._pose.copy()
        dt = min(dt, self.limits.max_dt)
        delta = target[:3, 3] - self._pose[:3, 3]
        distance = float(np.linalg.norm(delta))
        max_step = self.limits.max_translation_speed * dt
        if self.limits.max_translation_acceleration is not None:
            max_step = min(
                max_step,
                float(np.linalg.norm(self._linear_velocity)) * dt
                + 0.5 * self.limits.max_translation_acceleration * dt * dt
                + 1e-9,
            )
            # Also cap the speed change directly so a single step cannot exceed a*dt.
            allowed_speed = (
                float(np.linalg.norm(self._linear_velocity))
                + self.limits.max_translation_acceleration * dt
            )
            max_step = min(max_step, allowed_speed * dt)
        if distance > max_step > 0.0:
            delta = delta * (max_step / distance)
        elif max_step <= 0.0:
            delta = np.zeros(3, dtype=np.float64)
        relative = self._pose[:3, :3].T @ target[:3, :3]
        vector = rotation_vector(relative)
        angle = float(np.linalg.norm(vector))
        max_angle = self.limits.max_angular_speed * dt
        if self.limits.max_angular_acceleration is not None:
            allowed_rate = (
                float(np.linalg.norm(self._angular_velocity))
                + self.limits.max_angular_acceleration * dt
            )
            max_angle = min(max_angle, allowed_rate * dt)
        if angle > max_angle > 0.0:
            vector = vector * (max_angle / angle)
        elif max_angle <= 0.0:
            vector = np.zeros(3, dtype=np.float64)
        updated = np.eye(4, dtype=np.float64)
        updated[:3, :3] = self._pose[:3, :3] @ rotation_from_vector(vector)
        updated[:3, 3] = self._pose[:3, 3] + delta
        self._linear_velocity = delta / dt
        self._angular_velocity = vector / dt
        self._pose = updated
        self._timestamp = float(timestamp)
        return updated.copy()


def clip_workspace(
    pose: np.ndarray,
    minimum: np.ndarray,
    maximum: np.ndarray,
) -> np.ndarray:
    limited = np.array(pose, dtype=np.float64, copy=True)
    low = np.asarray(minimum, dtype=np.float64)
    high = np.asarray(maximum, dtype=np.float64)
    if low.shape != (3,) or high.shape != (3,):
        raise ValueError("workspace bounds must have length 3")
    if np.any(low >= high):
        raise ValueError("workspace minimum must be below maximum")
    limited[:3, 3] = np.clip(limited[:3, 3], low, high)
    return limited


class ReferenceProcessor:
    """Frame-agnostic conditioner. Callers map Quest/tool frames first."""

    def __init__(
        self,
        *,
        rate: RateLimit | None = None,
        workspace_min: np.ndarray | None = None,
        workspace_max: np.ndarray | None = None,
        max_equilibrium_translation: float | None = None,
        max_equilibrium_rotation: float | None = None,
    ) -> None:
        self.rate = PoseRateLimiter(rate)
        self.workspace_min = None if workspace_min is None else np.asarray(workspace_min, dtype=np.float64)
        self.workspace_max = None if workspace_max is None else np.asarray(workspace_max, dtype=np.float64)
        self.max_equilibrium_translation = max_equilibrium_translation
        self.max_equilibrium_rotation = max_equilibrium_rotation

    def reset(self, pose: np.ndarray, timestamp: float) -> None:
        self.rate.reset(pose, timestamp)

    def process(
        self,
        pose: np.ndarray,
        timestamp: float,
        measured: np.ndarray | None = None,
    ) -> np.ndarray:
        conditioned = self.rate.apply(pose, timestamp)
        if self.workspace_min is not None and self.workspace_max is not None:
            conditioned = clip_workspace(
                conditioned, self.workspace_min, self.workspace_max
            )
        if (
            measured is not None
            and self.max_equilibrium_translation is not None
            and self.max_equilibrium_rotation is not None
        ):
            conditioned = limit_equilibrium(
                conditioned,
                measured,
                self.max_equilibrium_translation,
                self.max_equilibrium_rotation,
            )
        return conditioned
