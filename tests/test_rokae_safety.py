"""Numerical tests for Cartesian safety, watchdog, and mock latest-value IO."""

import time
import unittest

import numpy as np

from anydexretarget.teleop import (
    ArmTeleopController,
    PoseSafetyConfig,
    PoseSafetyLimiter,
    RelativePoseMapper,
    make_transform,
)
from anydexretarget.teleop.pose import (
    matrix_to_rotation_vector,
    rotation_vector_to_matrix,
)
from example.output.real.drivers_rokae import MockRokaeDriver


def safety_config(**overrides) -> PoseSafetyConfig:
    values = dict(
        max_translation_speed=0.10,
        max_angular_speed=0.50,
        max_translation_delta=0.01,
        max_rotation_delta=0.02,
        low_pass_alpha=1.0,
        workspace_min=(-1.0, -1.0, -1.0),
        workspace_max=(1.0, 1.0, 1.0),
    )
    values.update(overrides)
    return PoseSafetyConfig(**values)


class PoseSafetyTest(unittest.TestCase):
    def test_g_translation_jump_is_rate_limited(self) -> None:
        limiter = PoseSafetyLimiter(safety_config())
        limiter.reset(np.eye(4), 0.0)
        output = limiter.apply(make_transform([0.5, 0.0, 0.0]), 0.01)
        self.assertAlmostEqual(output[0, 3], 0.001, places=12)

    def test_h_180_degree_jump_is_angularly_limited(self) -> None:
        limiter = PoseSafetyLimiter(safety_config())
        limiter.reset(np.eye(4), 0.0)
        target = make_transform(
            rotation=rotation_vector_to_matrix(np.array([0.0, 0.0, np.pi]))
        )
        output = limiter.apply(target, 0.01)
        angle = np.linalg.norm(matrix_to_rotation_vector(output[:3, :3]))
        self.assertAlmostEqual(angle, 0.005, places=12)

    def test_i_low_pass_reduces_noisy_position_variance(self) -> None:
        limiter = PoseSafetyLimiter(
            safety_config(
                max_translation_speed=100.0,
                max_translation_delta=10.0,
                max_angular_speed=100.0,
                max_rotation_delta=10.0,
                low_pass_alpha=0.15,
            )
        )
        limiter.reset(np.eye(4), 0.0)
        raw = np.array([0.03, -0.03] * 50)
        filtered = []
        for index, value in enumerate(raw, start=1):
            output = limiter.apply(make_transform([value, 0.0, 0.0]), index * 0.01)
            filtered.append(output[0, 3])
        self.assertLess(np.var(filtered[20:]), np.var(raw[20:]) * 0.05)

    def test_workspace_clips_base_frame_position(self) -> None:
        limiter = PoseSafetyLimiter(
            safety_config(
                max_translation_speed=100.0,
                max_translation_delta=10.0,
                workspace_min=(-0.1, -0.2, 0.0),
                workspace_max=(0.1, 0.2, 0.4),
            )
        )
        limiter.reset(make_transform([0.0, 0.0, 0.2]), 0.0)
        output = limiter.apply(make_transform([1.0, -1.0, 2.0]), 1.0)
        np.testing.assert_allclose(output[:3, 3], [0.1, -0.2, 0.4])

    def test_initial_pose_outside_workspace_is_rejected(self) -> None:
        limiter = PoseSafetyLimiter(
            safety_config(
                workspace_min=(-0.1, -0.1, 0.0),
                workspace_max=(0.1, 0.1, 0.4),
            )
        )
        with self.assertRaisesRegex(ValueError, "outside the configured"):
            limiter.reset(make_transform([0.2, 0.0, 0.2]), 0.0)

    def test_j_timeout_holds_and_resume_recenters(self) -> None:
        mapper = RelativePoseMapper(np.eye(3), np.eye(4))
        limiter = PoseSafetyLimiter(
            safety_config(max_translation_speed=10.0, max_translation_delta=1.0)
        )
        controller = ArmTeleopController(mapper, limiter, command_timeout=0.1)
        controller.initialize(np.eye(4), make_transform([0.4, 0.0, 0.3]), 0.0)
        moving = controller.update(make_transform([0.1, 0.0, 0.0]), 0.02)
        held = controller.poll(0.20)
        assert held is not None
        self.assertEqual(held.status, "hold_timeout")
        np.testing.assert_allclose(held.base_target_safe, moving.base_target_safe)

        resumed = controller.update(make_transform([0.5, 0.0, 0.0]), 0.21)
        self.assertEqual(resumed.status, "active")
        np.testing.assert_allclose(resumed.base_target_safe, held.base_target_safe)

    def test_enable_disable_holds_and_recenters(self) -> None:
        controller = ArmTeleopController(
            RelativePoseMapper(np.eye(3), np.eye(4)),
            PoseSafetyLimiter(
                safety_config(max_translation_speed=10.0, max_translation_delta=1.0)
            ),
            enabled=True,
        )
        controller.initialize(np.eye(4), np.eye(4), 0.0)
        moved = controller.update(make_transform([0.1, 0.0, 0.0]), 0.02)
        controller.set_enabled(False)
        disabled = controller.update(make_transform([0.3, 0.0, 0.0]), 0.04)
        np.testing.assert_allclose(disabled.base_target_safe, moved.base_target_safe)
        controller.set_enabled(True)
        enabled = controller.update(make_transform([0.6, 0.0, 0.0]), 0.06)
        np.testing.assert_allclose(enabled.base_target_safe, moved.base_target_safe)

    def test_mock_driver_latest_value_and_timeout_hold(self) -> None:
        driver = MockRokaeDriver(np.eye(4), control_hz=500.0, command_timeout=0.02)
        driver.connect()
        driver.start()
        try:
            first = make_transform([0.1, 0.0, 0.0])
            final = make_transform([0.3, 0.0, 0.0])
            driver.set_target_pose(first)
            driver.set_target_pose(final)
            time.sleep(0.01)
            np.testing.assert_allclose(driver.get_tcp_pose(), final)
            time.sleep(0.03)
            self.assertTrue(driver.timed_out)
            np.testing.assert_allclose(driver.get_tcp_pose(), final)
        finally:
            driver.disconnect()


if __name__ == "__main__":
    unittest.main()
