"""Shared dataset/deployment encoding; never sends motion commands."""

import numpy as np

from anydexretarget.geometry import matrix_to_rotation_vector, rotation_vector_to_matrix


def pose6(pose):
    pose = np.asarray(pose, dtype=np.float64)
    if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
        raise ValueError("Invalid TCP transform")
    rotation = pose[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-4):
        raise ValueError("TCP rotation is not SO(3)")
    if not np.allclose(pose[3], [0, 0, 0, 1]):
        raise ValueError("Invalid homogeneous transform last row")
    return np.r_[pose[:3, 3], matrix_to_rotation_vector(rotation)].astype("f4")


def encode_state(tcp, native_hand_angles):
    angles = np.asarray(native_hand_angles, dtype=np.float32)
    if angles.shape != (6,) or not np.all(np.isfinite(angles)) or np.any((angles < 0) | (angles > 1000)):
        raise ValueError("Expected six actual Inspire angle registers in 0..1000")
    return np.r_[pose6(tcp), angles / 1000].astype("f4")


def decode_action(action):
    action = np.asarray(action, dtype=np.float64)
    if action.shape != (12,) or not np.all(np.isfinite(action)):
        raise ValueError("Expected one finite 12-dimensional action")
    tcp = np.eye(4)
    tcp[:3, :3] = rotation_vector_to_matrix(action[3:6])
    tcp[:3, 3] = action[:3]
    angles = np.rint(np.clip(action[6:], 0, 1) * 1000).astype(np.int16)
    return tcp, angles
