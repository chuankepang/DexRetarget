"""Point-to-point interpolators for offline joint and Cartesian moves.

The joint law is the quintic used by libfranka's motion generator, synchronized
on the slowest joint. Cartesian translation is linear in that same scalar, and
orientation is SLERP, which is the UR/Franka waypoint blend reduced to one
segment.
"""

from __future__ import annotations

import math

import numpy as np


def quintic(fraction: float) -> float:
    """Minimum-jerk scalar on [0, 1] with zero velocity and acceleration at both ends."""
    clipped = float(np.clip(fraction, 0.0, 1.0))
    return clipped ** 3 * (10.0 + clipped * (-15.0 + 6.0 * clipped))


def quintic_duration(
    delta: np.ndarray,
    max_velocity: np.ndarray,
    max_acceleration: np.ndarray,
) -> float:
    """Duration that respects per-joint velocity and acceleration bounds.

    A quintic reaches peak velocity ``1.875 * distance / T`` and peak
    acceleration ``5.773 * distance / T^2``.
    """
    distance = np.abs(np.asarray(delta, dtype=np.float64))
    velocity = np.asarray(max_velocity, dtype=np.float64)
    acceleration = np.asarray(max_acceleration, dtype=np.float64)
    if distance.shape != velocity.shape or distance.shape != acceleration.shape:
        raise ValueError("quintic bounds must match the displacement")
    if np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError("quintic bounds must be positive")
    times = np.zeros(distance.shape, dtype=np.float64)
    moving = distance > 1e-12
    times = np.where(moving, 1.875 * distance / velocity, times)
    accel_time = np.sqrt(np.maximum(5.773 * distance / acceleration, 0.0))
    times = np.maximum(times, accel_time)
    duration = float(np.max(times)) if times.size else 0.0
    return max(duration, 0.0)


def sample_joint_quintic(
    start: np.ndarray,
    goal: np.ndarray,
    max_velocity: np.ndarray,
    max_acceleration: np.ndarray,
    sample_hz: float = 1000.0,
) -> tuple[np.ndarray, float]:
    """Return ``(samples, duration)``. ``samples[0]`` is the start pose."""
    start = np.asarray(start, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    duration = quintic_duration(goal - start, max_velocity, max_acceleration)
    if duration <= 1e-9:
        return start.reshape(1, -1).copy(), 0.0
    count = max(2, int(math.ceil(duration * sample_hz)) + 1)
    times = np.linspace(0.0, duration, count)
    fractions = np.array([quintic(time / duration) for time in times])
    samples = start + fractions[:, None] * (goal - start)
    samples[-1] = goal
    return samples, duration


def slerp(start: np.ndarray, goal: np.ndarray, fraction: float) -> np.ndarray:
    """Spherical linear interpolation of two rotation matrices."""
    relative = start.T @ goal
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return start.copy()
    skew = relative - relative.T
    axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=np.float64)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12:
        return start.copy()
    axis /= axis_norm
    theta = angle * float(np.clip(fraction, 0.0, 1.0))
    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    rotation = (
        math.cos(theta) * np.eye(3)
        + math.sin(theta) * cross
        + (1.0 - math.cos(theta)) * np.outer(axis, axis)
    )
    return start @ rotation


def sample_cartesian(
    start: np.ndarray,
    goal: np.ndarray,
    duration: float,
    sample_hz: float = 1000.0,
) -> np.ndarray:
    """Quintic-timed Cartesian samples, shape ``(N, 4, 4)``."""
    if duration < 0.0 or not np.isfinite(duration):
        raise ValueError("Cartesian duration must be finite and non-negative")
    start = np.asarray(start, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    if duration <= 1e-9:
        return start.reshape(1, 4, 4).copy()
    count = max(2, int(math.ceil(duration * sample_hz)) + 1)
    samples = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], count, axis=0)
    for index, time in enumerate(np.linspace(0.0, duration, count)):
        fraction = quintic(time / duration)
        samples[index, :3, :3] = slerp(start[:3, :3], goal[:3, :3], fraction)
        samples[index, :3, 3] = start[:3, 3] + fraction * (goal[:3, 3] - start[:3, 3])
    samples[-1] = goal
    return samples
