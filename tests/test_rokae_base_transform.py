"""Numerical tests for world-to-ROKAE-base mounting transforms."""

import unittest

import numpy as np

from anydexretarget.teleop import RelativePoseMapper, invert_transform, make_transform
from anydexretarget.teleop.pose import rotation_vector_to_matrix


class BaseTransformTest(unittest.TestCase):
    def _run_mount_test(self, T_world_base: np.ndarray) -> None:
        T_base_tcp_ref = make_transform(
            [0.40, -0.15, 0.30],
            rotation_vector_to_matrix(np.array([0.1, -0.2, 0.05])),
        )
        T_vr_ref = make_transform([1.0, 1.5, 0.8])
        T_vr_current = T_vr_ref @ make_transform(
            [0.10, -0.04, 0.06],
            rotation_vector_to_matrix(np.array([0.2, 0.1, -0.15])),
        )
        mapper = RelativePoseMapper(np.eye(3), T_world_base)
        mapper.set_reference(T_vr_ref, T_base_tcp_ref)
        result = mapper.update(T_vr_current)

        expected_world = (
            T_world_base @ T_base_tcp_ref @ (invert_transform(T_vr_ref) @ T_vr_current)
        )
        expected_base = invert_transform(T_world_base) @ expected_world
        np.testing.assert_allclose(result.world_target, expected_world, atol=1e-10)
        np.testing.assert_allclose(result.base_target, expected_base, atol=1e-10)

    def test_d_upright_base(self) -> None:
        self._run_mount_test(np.eye(4))

    def test_e_upside_down_base(self) -> None:
        T_world_base = make_transform(
            [0.2, -0.1, 1.0], rotation_vector_to_matrix(np.array([np.pi, 0.0, 0.0]))
        )
        self._run_mount_test(T_world_base)

    def test_f_tilted_base_yaw_30_pitch_20(self) -> None:
        yaw = rotation_vector_to_matrix(np.array([0.0, 0.0, np.deg2rad(30.0)]))
        pitch = rotation_vector_to_matrix(np.array([0.0, np.deg2rad(20.0), 0.0]))
        T_world_base = make_transform([0.3, -0.2, 0.6], yaw @ pitch)
        self._run_mount_test(T_world_base)


if __name__ == "__main__":
    unittest.main()
