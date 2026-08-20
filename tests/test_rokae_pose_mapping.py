"""Numerical tests for Quest-reference to ROKAE Cartesian mapping."""

import unittest

import numpy as np

from anydexretarget.teleop import RelativePoseMapper, make_transform
from anydexretarget.teleop.pose import (
    matrix_to_rotation_vector,
    rotation_vector_to_matrix,
)


class RelativePoseMappingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.robot_ref = make_transform([0.45, -0.10, 0.35])
        self.vr_ref = make_transform([1.0, 2.0, 3.0])

    def mapper(self, axis_map=None, translation_scale=1.0, rotation_scale=1.0):
        mapper = RelativePoseMapper(
            np.eye(3) if axis_map is None else axis_map,
            np.eye(4),
            translation_scale,
            rotation_scale,
        )
        mapper.set_reference(self.vr_ref, self.robot_ref)
        return mapper

    def test_a_zero_motion_returns_robot_reference(self) -> None:
        result = self.mapper().update(self.vr_ref)
        np.testing.assert_allclose(result.base_target, self.robot_ref, atol=1e-12)

    def test_b_xyz_translation_axis_mapping_and_scale(self) -> None:
        # Quest X->robot Y, Quest Y->robot Z, Quest Z->robot X.
        axis_map = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        mapper = self.mapper(axis_map=axis_map, translation_scale=0.5)
        expected_deltas = (
            np.array([0.0, 0.05, 0.0]),
            np.array([0.0, 0.0, 0.05]),
            np.array([0.05, 0.0, 0.0]),
        )
        for axis, expected in enumerate(expected_deltas):
            vr = self.vr_ref.copy()
            vr[axis, 3] += 0.10
            result = mapper.update(vr)
            np.testing.assert_allclose(
                result.base_target[:3, 3] - self.robot_ref[:3, 3],
                expected,
                atol=1e-12,
            )

    def test_c_xyz_rotations_use_so3_not_euler_subtraction(self) -> None:
        mapper = self.mapper(rotation_scale=0.5)
        for axis in np.eye(3):
            vr = self.vr_ref.copy()
            vr[:3, :3] = rotation_vector_to_matrix(axis * np.deg2rad(30.0))
            result = mapper.update(vr)
            output_rotation = matrix_to_rotation_vector(
                self.robot_ref[:3, :3].T @ result.base_target[:3, :3]
            )
            np.testing.assert_allclose(
                output_rotation, axis * np.deg2rad(15.0), atol=1e-10
            )

    def test_reference_relative_rotation_with_nonidentity_reference(self) -> None:
        reference_rotation = rotation_vector_to_matrix(np.array([0.2, -0.1, 0.3]))
        vr_ref = make_transform([0.2, 0.3, 0.4], reference_rotation)
        local_delta = rotation_vector_to_matrix(np.array([0.0, 0.25, 0.0]))
        vr_current = vr_ref @ make_transform(rotation=local_delta)
        mapper = self.mapper()
        mapper.set_reference(vr_ref, self.robot_ref)
        result = mapper.update(vr_current)
        np.testing.assert_allclose(result.base_target[:3, :3], local_delta, atol=1e-10)


if __name__ == "__main__":
    unittest.main()
