"""Equilibrium reference limiting.

This is not a low-pass filter. It caps the pose error between a desired
equilibrium and the measured TCP. Translation is a Euclidean ball. Rotation
is the short SO(3) arc, so quaternion sign and 2*pi wrap do not create a
long spin.
"""

from __future__ import annotations

import numpy as np


def _as_pose(pose: np.ndarray, name: str) -> np.ndarray:
    matrix = np.asarray(pose, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 pose")
    if np.linalg.norm(matrix[3] - np.array([0.0, 0.0, 0.0, 1.0])) > 1e-6:
        raise ValueError(f"{name} has an invalid bottom row")
    rotation = matrix[:3, :3]
    if abs(np.linalg.det(rotation) - 1.0) > 2e-3:
        raise ValueError(f"{name} rotation is not SO(3)")
    return matrix


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """SO(3) logarithm. Angle is in [0, pi]."""
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-8:
        return np.zeros(3, dtype=np.float64)
    skew = rotation - rotation.T
    axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-8:
        # 180 deg: the skew vanishes. Recover an axis from the diagonal.
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        axis = np.zeros(3, dtype=np.float64)
        axis[index] = 1.0
        axis += rotation[:, index]
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-8:
            axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            axis /= axis_norm
        return axis * angle
    return axis * (angle / norm)


def rotation_from_vector(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3, dtype=np.float64)
    axis = np.asarray(vector, dtype=np.float64) / angle
    skew = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def quaternion_xyzw_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """XYZW quaternion to a rotation. q and -q are the same pose."""
    raw = np.asarray(quaternion, dtype=np.float64)
    if raw.shape != (4,) or not np.all(np.isfinite(raw)):
        raise ValueError("quaternion must be four finite XYZW values")
    norm = float(np.linalg.norm(raw))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = raw / norm
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def limit_equilibrium(
    desired: np.ndarray,
    measured: np.ndarray,
    max_translation: float,
    max_rotation: float,
) -> np.ndarray:
    """Return desired clipped toward measured."""
    if not np.isfinite(max_translation) or max_translation <= 0.0:
        raise ValueError("max_translation must be finite and > 0")
    if not np.isfinite(max_rotation) or not 0.0 < max_rotation <= np.pi:
        raise ValueError("max_rotation must be in (0, pi]")
    goal = _as_pose(desired, "desired pose")
    current = _as_pose(measured, "measured pose")
    delta = goal[:3, 3] - current[:3, 3]
    distance = float(np.linalg.norm(delta))
    if distance > max_translation:
        delta = delta * (max_translation / distance)
    relative = current[:3, :3].T @ goal[:3, :3]
    vector = rotation_vector(relative)
    angle = float(np.linalg.norm(vector))
    if angle > max_rotation:
        vector = vector * (max_rotation / angle)
    limited = np.eye(4, dtype=np.float64)
    limited[:3, :3] = current[:3, :3] @ rotation_from_vector(vector)
    limited[:3, 3] = current[:3, 3] + delta
    return limited
