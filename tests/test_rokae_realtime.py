"""Deterministic tests for the decoupled ROKAE realtime command path."""

import math
import unittest

import numpy as np

from anydexretarget.teleop import (
    CartesianSetpointGenerator,
    LatestTargetBuffer,
    RealtimeCartesianConfig,
    make_transform,
    rotation_vector_to_matrix,
)
from anydexretarget.teleop.pose import matrix_to_rotation_vector
from rokae.python import _rokae_cpp


def config(control_hz: float) -> RealtimeCartesianConfig:
    return RealtimeCartesianConfig(
        control_hz=control_hz,
        translation_cutoff_hz=7.0,
        rotation_cutoff_hz=7.0,
        translation_deadband=0.0,
        rotation_deadband=0.0,
        max_translation_speed=0.30,
        max_angular_speed=0.80,
        max_translation_acceleration=1.0,
        max_angular_acceleration=3.0,
        max_translation_jerk=12.0,
        max_angular_jerk=30.0,
        workspace_min=(-1.0, -1.0, -1.0),
        workspace_max=(1.0, 1.0, 1.0),
        hold_timeout=0.12,
        stop_timeout=0.40,
        max_source_age=0.20,
    )


class LatestValueBufferTest(unittest.TestCase):
    def test_latest_wins_without_fifo_backlog(self) -> None:
        buffer = LatestTargetBuffer(max_source_age=1.0)
        for sequence in range(1000):
            self.assertTrue(
                buffer.publish(
                    make_transform([sequence / 1000.0, 0.0, 0.0]),
                    sequence=sequence,
                    source_timestamp=sequence / 1000.0,
                    receive_timestamp=sequence / 1000.0,
                )
            )
        sample = buffer.snapshot()
        assert sample is not None
        self.assertEqual(sample.sequence, 999)
        self.assertAlmostEqual(sample.pose[0, 3], 0.999)
        self.assertFalse(
            buffer.publish(
                np.eye(4), sequence=998, source_timestamp=1.0, receive_timestamp=1.0
            )
        )
        self.assertEqual(buffer.stats.rejected_sequence, 1)

    def test_stale_delayed_and_out_of_order_samples_are_rejected(self) -> None:
        buffer = LatestTargetBuffer(max_source_age=0.10)
        self.assertFalse(
            buffer.publish(
                np.eye(4), sequence=0, source_timestamp=0.0, receive_timestamp=0.2
            )
        )
        self.assertTrue(
            buffer.publish(
                np.eye(4), sequence=2, source_timestamp=0.20, receive_timestamp=0.25
            )
        )
        self.assertFalse(
            buffer.publish(
                np.eye(4), sequence=3, source_timestamp=0.19, receive_timestamp=0.26
            )
        )
        self.assertEqual(buffer.stats.rejected_source_age, 1)
        self.assertEqual(buffer.stats.rejected_source_time, 1)


class RealtimeRateMatrixTest(unittest.TestCase):
    def _run_rate_pair(self, quest_hz: float, robot_hz: float) -> None:
        cfg = config(robot_hz)
        generator = CartesianSetpointGenerator(cfg)
        generator.reset(np.eye(4), timestamp=0.0)
        dt = 1.0 / robot_hz
        next_quest = 0.0
        sequence = 0
        positions = []
        linear_velocities = []
        linear_accelerations = []
        angular_velocities = []
        angular_accelerations = []
        for index in range(int(robot_hz * 2.0)):
            now = index * dt
            if now + 1e-12 >= next_quest:
                # A bounded but deliberately faster human-side trajectory.
                pose = make_transform(
                    [0.20 * math.sin(2.0 * now), 0.10 * math.sin(1.3 * now), 0.0],
                    rotation_vector_to_matrix(
                        np.array([0.0, 0.0, 0.6 * math.sin(1.7 * now)])
                    ),
                )
                generator.publish_target(
                    pose,
                    sequence=sequence,
                    source_timestamp=now,
                    receive_timestamp=now,
                )
                sequence += 1
                next_quest += 1.0 / quest_hz
            step = generator.step(now)
            positions.append(step.pose[:3, 3])
            linear_velocities.append(step.linear_velocity)
            linear_accelerations.append(step.linear_acceleration)
            angular_velocities.append(step.angular_velocity)
            angular_accelerations.append(step.angular_acceleration)
            self.assertTrue(np.all(np.isfinite(step.pose)))
            np.testing.assert_allclose(step.pose[3], [0.0, 0.0, 0.0, 1.0])

        linear_v = np.linalg.norm(linear_velocities, axis=1)
        linear_a = np.linalg.norm(linear_accelerations, axis=1)
        angular_v = np.linalg.norm(angular_velocities, axis=1)
        angular_a = np.linalg.norm(angular_accelerations, axis=1)
        self.assertLessEqual(float(np.max(linear_v)), cfg.max_translation_speed + 1e-9)
        self.assertLessEqual(
            float(np.max(linear_a)), cfg.max_translation_acceleration + 1e-9
        )
        self.assertLessEqual(float(np.max(angular_v)), cfg.max_angular_speed + 1e-9)
        self.assertLessEqual(
            float(np.max(angular_a)), cfg.max_angular_acceleration + 1e-9
        )
        linear_jerk = np.linalg.norm(np.diff(linear_accelerations, axis=0) / dt, axis=1)
        angular_jerk = np.linalg.norm(
            np.diff(angular_accelerations, axis=0) / dt, axis=1
        )
        self.assertLessEqual(
            float(np.max(linear_jerk)), cfg.max_translation_jerk + 1e-6
        )
        self.assertLessEqual(float(np.max(angular_jerk)), cfg.max_angular_jerk + 1e-6)
        step_distance = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        self.assertLessEqual(
            float(np.max(step_distance)), cfg.max_translation_speed * dt + 1e-9
        )

    def test_quest_and_robot_frequency_matrix(self) -> None:
        for quest_hz in (60.0, 72.0, 90.0):
            for robot_hz in (250.0, 500.0, 1000.0):
                with self.subTest(quest_hz=quest_hz, robot_hz=robot_hz):
                    self._run_rate_pair(quest_hz, robot_hz)

    def test_fixed_target_settles_without_limit_cycle(self) -> None:
        cfg = config(1000.0)
        generator = CartesianSetpointGenerator(cfg)
        generator.reset(np.eye(4), timestamp=0.0)
        target = make_transform([0.05, 0.0, 0.0])
        positions = []
        sequence = 0
        for index in range(1, 3001):
            now = index / 1000.0
            if index % 14 == 0:
                self.assertTrue(
                    generator.publish_target(
                        target,
                        sequence=sequence,
                        source_timestamp=now,
                        receive_timestamp=now,
                    )
                )
                sequence += 1
            positions.append(generator.step(now).pose[0, 3])
        self.assertLessEqual(max(positions), 0.0501)
        self.assertAlmostEqual(positions[-1], 0.05, delta=1e-4)

    def test_init_to_teleop_transition_starts_without_pose_jump(self) -> None:
        cfg = config(500.0)
        initial = make_transform([0.25, -0.10, 0.30])
        generator = CartesianSetpointGenerator(cfg)
        generator.reset(initial, timestamp=1.0)
        before_input = generator.step(1.002)
        self.assertEqual(before_input.state, "holding_no_input")
        np.testing.assert_allclose(before_input.pose, initial, atol=1e-12)
        self.assertTrue(
            generator.publish_target(
                initial,
                sequence=0,
                source_timestamp=1.002,
                receive_timestamp=1.002,
            )
        )
        first_tracking = generator.step(1.004)
        self.assertEqual(first_tracking.state, "tracking")
        np.testing.assert_allclose(first_tracking.pose, initial, atol=1e-12)

    def test_jitter_loss_delay_and_watchdog_transitions(self) -> None:
        cfg = config(500.0)
        generator = CartesianSetpointGenerator(cfg)
        generator.reset(np.eye(4), timestamp=0.0)
        rng = np.random.default_rng(7)
        arrivals = []
        for sequence in range(60):
            source = sequence / 60.0
            if sequence % 7 == 0:  # deterministic packet loss
                continue
            delay = max(0.0, 0.025 + rng.normal(0.0, 0.015))
            arrivals.append((source + delay, sequence, source))
        arrivals.sort()
        pending = 0
        states = []
        dt = 1.0 / cfg.control_hz
        for index in range(int(1.8 * cfg.control_hz)):
            now = index * dt
            while pending < len(arrivals) and arrivals[pending][0] <= now:
                arrival, sequence, source = arrivals[pending]
                generator.publish_target(
                    make_transform([0.15 * source, 0.0, 0.0]),
                    sequence=sequence,
                    source_timestamp=source,
                    receive_timestamp=arrival,
                )
                pending += 1
            states.append(generator.step(now).state)
        self.assertIn("tracking", states)
        self.assertIn("holding_stale", states)
        self.assertEqual(states[-1], "safe_stop")


class CppRealtimeCoreTest(unittest.TestCase):
    @staticmethod
    def cpp_config():
        cfg = _rokae_cpp.RealtimeConfig()
        cfg.translation_cutoff_hz = 8.0
        cfg.rotation_cutoff_hz = 8.0
        cfg.translation_deadband = 0.0
        cfg.rotation_deadband = 0.0
        cfg.max_translation_speed = 0.15
        cfg.max_angular_speed = 0.60
        cfg.max_translation_acceleration = 0.50
        cfg.max_angular_acceleration = 2.0
        cfg.max_translation_jerk = 4.0
        cfg.max_angular_jerk = 15.0
        cfg.max_target_translation_delta = 1.0
        cfg.max_target_rotation_delta = 2.0
        cfg.workspace_min = (-1.0, -1.0, -1.0)
        cfg.workspace_max = (1.0, 1.0, 1.0)
        cfg.hold_timeout = 0.15
        cfg.stop_timeout = 0.60
        cfg.max_source_age = 0.20
        return cfg

    def test_cpp_latest_value_ordering_and_watchdog(self) -> None:
        core = _rokae_cpp.RealtimeCore(self.cpp_config())
        core.reset(np.eye(4), 0.0)
        first = make_transform([0.01, 0.0, 0.0])
        latest = make_transform([0.02, 0.0, 0.0])
        self.assertTrue(core.publish(first, 0.0, 0, 0.0))
        self.assertTrue(core.publish(latest, 0.001, 1, 0.001))
        self.assertFalse(core.publish(first, 0.002, 0, 0.002))
        core.step(0.002)
        diag = core.diagnostics(0.002)
        self.assertEqual(diag["latest_sequence"], 1)
        self.assertEqual(diag["rejected_sequence"], 1)
        self.assertEqual(diag["watchdog_state"], "tracking")
        self.assertIsNone(diag["callback_error"])
        core.step(0.20)
        self.assertEqual(core.diagnostics(0.20)["watchdog_state"], "holding_stale")
        core.step(0.70)
        self.assertTrue(core.should_finish)
        self.assertEqual(core.diagnostics(0.70)["watchdog_state"], "safe_stop")

    def test_cpp_init_to_first_target_has_no_jump(self) -> None:
        cfg = self.cpp_config()
        cfg.max_target_translation_delta = 1e-6
        cfg.max_target_rotation_delta = 1e-6
        core = _rokae_cpp.RealtimeCore(cfg)
        initial = make_transform(
            [0.25, -0.10, 0.30],
            rotation_vector_to_matrix(np.array([0.1, -0.2, 0.05])),
        )
        core.reset(initial, 1.0)
        np.testing.assert_allclose(core.step(1.001), initial, atol=1e-12)
        core.hold()
        self.assertTrue(core.publish(initial, 1.001, 0, 1.001))
        np.testing.assert_allclose(core.step(1.002), initial, atol=1e-12)
        self.assertEqual(core.diagnostics(1.002)["watchdog_state"], "tracking")
        self.assertEqual(core.diagnostics(1.002)["rejected_target_delta"], 0)

    def test_cpp_speed_acceleration_and_rotation_are_bounded(self) -> None:
        cfg = self.cpp_config()
        core = _rokae_cpp.RealtimeCore(cfg)
        core.reset(np.eye(4), 0.0)
        target = make_transform(
            [0.5, 0.0, 0.0],
            rotation_vector_to_matrix(np.array([0.0, 0.0, 1.0])),
        )
        self.assertTrue(core.publish(target, 0.0, 0, 0.0))
        positions = []
        angles = []
        for index in range(1, 101):
            pose = core.step(index * 0.001)
            positions.append(pose[:3, 3].copy())
            angles.append(
                np.linalg.norm(matrix_to_rotation_vector(pose[:3, :3]))
            )
        velocities = np.diff(positions, axis=0) / 0.001
        speeds = np.linalg.norm(velocities, axis=1)
        accelerations = np.linalg.norm(np.diff(velocities, axis=0) / 0.001, axis=1)
        angular_speeds = np.abs(np.diff(angles)) / 0.001
        self.assertLessEqual(float(np.max(speeds)), cfg.max_translation_speed + 1e-8)
        self.assertLessEqual(
            float(np.max(accelerations)), cfg.max_translation_acceleration + 1e-6
        )
        self.assertLessEqual(
            float(np.max(angular_speeds)), cfg.max_angular_speed + 1e-6
        )

    def test_cpp_rejects_stale_source_and_hold_is_persistent(self) -> None:
        core = _rokae_cpp.RealtimeCore(self.cpp_config())
        core.reset(np.eye(4), 1.0)
        self.assertFalse(core.publish(np.eye(4), 1.0, 0, 1.3))
        self.assertEqual(core.diagnostics(1.3)["rejected_source_age"], 1)
        core.hold()
        core.step(3.0)
        self.assertFalse(core.should_finish)
        self.assertEqual(core.diagnostics(3.0)["watchdog_state"], "holding_stale")

    def test_cpp_rejects_unsafe_target_jump(self) -> None:
        cfg = self.cpp_config()
        cfg.max_target_translation_delta = 0.02
        cfg.max_target_rotation_delta = 0.10
        core = _rokae_cpp.RealtimeCore(cfg)
        core.reset(np.eye(4), 0.0)
        self.assertFalse(core.publish(make_transform([0.03, 0.0, 0.0]), 0.0, 0, 0.0))
        self.assertEqual(core.diagnostics(0.0)["rejected_target_delta"], 1)

    def test_cpp_fixed_target_settles_without_limit_cycle(self) -> None:
        core = _rokae_cpp.RealtimeCore(self.cpp_config())
        core.reset(np.eye(4), 0.0)
        target = make_transform([0.05, 0.0, 0.0])
        positions = []
        sequence = 0
        for index in range(1, 3001):
            now = index / 1000.0
            if index % 14 == 0:
                self.assertTrue(core.publish(target, now, sequence, now))
                sequence += 1
            positions.append(core.step(now)[0, 3])
        self.assertLessEqual(max(positions), 0.0501)
        self.assertAlmostEqual(positions[-1], 0.05, delta=1e-4)


if __name__ == "__main__":
    unittest.main()
