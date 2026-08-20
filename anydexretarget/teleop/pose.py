"""Small, dependency-free SE(3) helpers for Cartesian teleoperation."""

from __future__ import annotations

import numpy as np


def validate_rotation(rotation: np.ndarray, name: str = "rotation") -> np.ndarray:
    """Validate and copy a proper 3x3 rotation matrix."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6):
        raise ValueError(f"{name} must be orthonormal")
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        raise ValueError(f"{name} must have determinant +1")
    return matrix.copy()


def validate_transform(transform: np.ndarray, name: str = "transform") -> np.ndarray:
    """Validate and copy a finite rigid 4x4 transform."""
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    validate_rotation(matrix[:3, :3], f"{name} rotation")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} last row must be [0, 0, 0, 1]")
    return matrix.copy()


def quaternion_xyzw_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert an ``[x, y, z, w]`` quaternion to a rotation matrix."""
    quat = np.asarray(quaternion, dtype=np.float64)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        raise ValueError("quaternion must contain four finite values [x, y, z, w]")
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = quat / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to a normalized ``[x, y, z, w]`` quaternion."""
    matrix = validate_rotation(rotation)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        quat = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[1, 0] - matrix[0, 1]) / s,
                0.25 * s,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            quat = np.array(
                [
                    0.25 * s,
                    (matrix[0, 1] + matrix[1, 0]) / s,
                    (matrix[0, 2] + matrix[2, 0]) / s,
                    (matrix[2, 1] - matrix[1, 2]) / s,
                ]
            )
        elif index == 1:
            s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            quat = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / s,
                    0.25 * s,
                    (matrix[1, 2] + matrix[2, 1]) / s,
                    (matrix[0, 2] - matrix[2, 0]) / s,
                ]
            )
        else:
            s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            quat = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / s,
                    (matrix[1, 2] + matrix[2, 1]) / s,
                    0.25 * s,
                    (matrix[1, 0] - matrix[0, 1]) / s,
                ]
            )
    quat /= np.linalg.norm(quat)
    if quat[3] < 0.0:
        quat = -quat
    return quat


def rotation_vector_to_matrix(rotation_vector: np.ndarray) -> np.ndarray:
    """SO(3) exponential map from an axis-angle vector."""
    vector = np.asarray(rotation_vector, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("rotation_vector must be a finite 3-vector")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def matrix_to_rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """SO(3) logarithm map returning the shortest axis-angle vector."""
    matrix = validate_rotation(rotation)
    quaternion = matrix_to_quaternion_xyzw(matrix)
    xyz = quaternion[:3]
    xyz_norm = float(np.linalg.norm(xyz))
    if xyz_norm < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * np.arctan2(xyz_norm, quaternion[3])
    if angle > np.pi:
        angle -= 2.0 * np.pi
    return xyz * (angle / xyz_norm)


def make_transform(
    translation: np.ndarray | None = None,
    rotation: np.ndarray | None = None,
) -> np.ndarray:
    """Create a rigid transform from optional translation and rotation."""
    result = np.eye(4, dtype=np.float64)
    if translation is not None:
        position = np.asarray(translation, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("translation must be a finite 3-vector")
        result[:3, 3] = position
    if rotation is not None:
        result[:3, :3] = validate_rotation(rotation)
    return result


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid transform without a generic matrix inverse."""
    matrix = validate_transform(transform)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return result


def interpolate_rotation(
    start: np.ndarray, end: np.ndarray, fraction: float
) -> np.ndarray:
    """Geodesically interpolate two rotation matrices."""
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction must be in [0, 1]")
    start_matrix = validate_rotation(start, "start rotation")
    end_matrix = validate_rotation(end, "end rotation")
    delta = matrix_to_rotation_vector(start_matrix.T @ end_matrix)
    return start_matrix @ rotation_vector_to_matrix(fraction * delta)


def pose_distance(start: np.ndarray, end: np.ndarray) -> tuple[float, float]:
    """Return translation and angular distances between two transforms."""
    first = validate_transform(start, "start transform")
    second = validate_transform(end, "end transform")
    translation = float(np.linalg.norm(second[:3, 3] - first[:3, 3]))
    angle = float(
        np.linalg.norm(matrix_to_rotation_vector(first[:3, :3].T @ second[:3, :3]))
    )
    return translation, angle
