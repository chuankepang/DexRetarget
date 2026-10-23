"""Regression tests for the upright ROKAE mounting and guarded init path."""

import unittest
from pathlib import Path

import numpy as np
import yaml

from dexretarget.teleop.devices.rokae import plan_init_joint_targets


ROOT = Path(__file__).resolve().parents[1]


def _er3_pro_fk(joints: np.ndarray) -> np.ndarray:
    alpha = np.array(
        [-np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, 0.0]
    )
    d = np.array([0.3415, 0.0, 0.394, 0.0, 0.366, 0.0, 0.2503])
    transform = np.eye(4)
    for theta, twist, offset in zip(joints, alpha, d):
        c, s = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(twist), np.sin(twist)
        transform = transform @ np.array(
            [
                [c, -s * ca, s * sa, 0.0],
                [s, c * ca, -c * sa, 0.0],
                [0.0, sa, ca, offset],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
    return transform


class UprightInitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        with (ROOT / "config/robots/rokae.yaml").open(encoding="utf-8") as stream:
            cls.config = yaml.safe_load(stream)

    def test_world_base_rotation_is_identity(self) -> None:
        pose = self.config["robot"]["base_transform"]["pose_xyzw"]
        np.testing.assert_allclose(pose[3:], [0, 0, 0, 1], atol=1e-12)

    def test_init_aligns_j7_wrist_axis_with_world_x(self) -> None:
        init = self.config["robot"]["initial_joint_move"]
        joints = np.asarray(init["joint_position"], dtype=np.float64)
        transform = _er3_pro_fk(joints)
        tcp_forward = transform[:3, 2]
        desired = np.array([1.0, 0.0, 0.0])
        self.assertGreater(float(tcp_forward @ desired), 0.999)
        self.assertGreater(abs(float(transform[:3, 0] @ np.array([0.0, 1.0, 0.0]))), 0.999)
        self.assertGreaterEqual(abs(float(joints[1])), 0.35)
        self.assertGreaterEqual(abs(float(joints[3])), 0.35)
        self.assertGreaterEqual(abs(float(joints[5])), 0.35)

    def test_old_palm_up_pose_uses_staged_interpolated_path(self) -> None:
        init = self.config["robot"]["initial_joint_move"]
        current = np.array([0.0, -0.249, 0.0, 1.644, 0.0, 0.949, -1.507])
        planned = plan_init_joint_targets(
            current,
            init["joint_position"],
            init["max_segment_delta"],
        )
        self.assertGreater(len(planned), 2)
        # J7 is held while J1-J6 reach the desired arm shape.
        first_j7_change = next(
            index for index, target in enumerate(planned)
            if abs(float(target[6] - current[6])) > 1e-12
        )
        np.testing.assert_allclose(
            planned[first_j7_change - 1][:6], init["joint_position"][:6], atol=1e-12
        )
        cursor = current
        for target in planned:
            self.assertLessEqual(
                float(np.max(np.abs(target - cursor))),
                float(init["max_segment_delta"]) + 1e-12,
            )
            cursor = target
        np.testing.assert_allclose(cursor, init["joint_position"], atol=1e-12)

    def test_live_path_is_subdivided_into_small_init_steps(self) -> None:
        init = self.config["robot"]["initial_joint_move"]
        current = np.array([0.0, -0.249, 0.0, 1.644, 0.0, 0.949, -1.507])
        planned = plan_init_joint_targets(
            current,
            init["joint_position"],
            init["max_segment_delta"],
        )
        self.assertGreater(len(planned), 2)
        cursor = current
        for target in planned:
            self.assertLessEqual(
                float(np.max(np.abs(target - cursor))),
                float(init["max_segment_delta"]) + 1e-12,
            )
            cursor = target
        np.testing.assert_allclose(cursor, init["joint_position"], atol=1e-12)

    def test_init_nrt_speed_uses_bounded_faster_profile(self) -> None:
        # The operator explicitly requested faster init/home motion. The old
        # <100 / <=0.20 commissioning profile is no longer the requirement.
        init = self.config["robot"]["initial_joint_move"]
        speed = round(float(init["joint_speed"]) * 1000.0)
        self.assertGreaterEqual(speed, 5)
        self.assertLessEqual(speed, 500)
        self.assertGreaterEqual(float(init["online_speed_scale"]), 0.01)
        self.assertLessEqual(float(init["online_speed_scale"]), 0.50)

    def test_already_at_init_does_not_visit_waypoint(self) -> None:
        init = self.config["robot"]["initial_joint_move"]
        planned = plan_init_joint_targets(
            init["joint_position"],
            init["joint_position"],
            init["max_segment_delta"],
        )
        self.assertEqual(planned, [])


if __name__ == "__main__":
    unittest.main()
