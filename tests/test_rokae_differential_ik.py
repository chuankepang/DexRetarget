"""Simulation tests for host differential IK and the 1 kHz joint stream."""

from __future__ import annotations

import unittest

import numpy as np

from rokae.control.differential_ik import DifferentialIK
from rokae.control.interpolate import sample_cartesian, sample_joint_quintic
from rokae.control.model import XmateErPro7, rotation_angle
from rokae.control.smoothing import JointStream, JointStreamConfig
from native_support import HAS_NATIVE

# Missing matching bindings must not prevent independent Python tests collecting.
if HAS_NATIVE:
    from rokae.python import _rokae_cpp
else:
    _rokae_cpp = None


def cpp_config(limits: JointStreamConfig):
    cfg = _rokae_cpp.JointRealtimeConfig()
    cfg.max_velocity = limits.max_velocity.tolist()
    cfg.max_acceleration = limits.max_acceleration.tolist()
    cfg.max_jerk = limits.max_jerk.tolist()
    cfg.position_min = limits.position_min.tolist()
    cfg.position_max = limits.position_max.tolist()
    cfg.reference_cutoff_hz = limits.reference_cutoff_hz
    cfg.deadband = limits.deadband
    cfg.max_target_delta = limits.max_target_delta
    cfg.hold_timeout = limits.hold_timeout
    cfg.stop_timeout = limits.stop_timeout
    cfg.max_source_age = limits.max_source_age
    cfg.future_tolerance = limits.future_tolerance
    cfg.validate()
    return cfg


class ModelTest(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = XmateErPro7()

    def test_init_pose_matches_commissioning_numbers(self) -> None:
        pose = self.robot.fk(self.robot.neutral)
        np.testing.assert_allclose(
            pose[:3, 3], [0.681311, 0.0, 0.496221], atol=1e-4
        )
        np.testing.assert_allclose(pose[:3, 2], [1.0, 0.0, 0.0], atol=1e-6)
        tool_x = pose[:3, 0]
        self.assertGreater(abs(tool_x[1]), 0.99)

    def test_jacobian_matches_a_finite_difference(self) -> None:
        q = self.robot.neutral.copy()
        jacobian = self.robot.jacobian(q)
        delta = 1e-6
        for index in range(7):
            step = np.zeros(7)
            step[index] = delta
            error = self.robot.local_error(q, self.robot.fk(q + step))
            np.testing.assert_allclose(error / delta, jacobian[:, index], atol=1e-4)

    def test_straight_pose_is_more_singular_than_init(self) -> None:
        straight = self.robot.smallest_singular_value(np.zeros(7))
        bent = self.robot.smallest_singular_value(self.robot.neutral)
        self.assertLess(straight, 0.05)
        self.assertGreater(bent, straight * 5.0)


class DifferentialIKTest(unittest.TestCase):
    @unittest.skipIf(_rokae_cpp is None, "Native ROKAE binding is not built; not validated")
    def test_tracks_a_translation_without_joint_jumps(self) -> None:
        robot = XmateErPro7()
        start = robot.neutral.copy()
        goal = robot.fk(start)
        goal[:3, 3] += np.array([0.02, -0.01, 0.015])
        ik = DifferentialIK(robot, start)
        limits = JointStreamConfig.from_limits(robot.stream_limits)
        core = _rokae_cpp.JointRealtimeCore(cpp_config(limits))
        core.reset(start.tolist(), 0.0)
        previous = start.copy()
        peak = 0.0
        for index in range(1, 2501):
            timestamp = index * 0.001
            if index % 20 == 0:
                fraction = min(1.0, timestamp / 1.6)
                target = robot.fk(start)
                target[:3, 3] = start_pose_translation(robot, start) + (
                    goal[:3, 3] - start_pose_translation(robot, start)
                ) * fraction
                target[:3, :3] = goal[:3, :3]
                joints = ik.step(target, 0.02)
                if not core.publish(joints.tolist(), timestamp, index, timestamp):
                    ik.reset(np.asarray(core.command_joints(), dtype=np.float64))
            command = np.asarray(core.step(timestamp), dtype=np.float64)
            peak = max(peak, float(np.max(np.abs(command - previous))))
            previous = command
            self.assertTrue(np.all(command >= robot.position_min - 1e-9))
            self.assertTrue(np.all(command <= robot.position_max + 1e-9))
        reached = robot.fk(previous)
        translation = float(np.linalg.norm(reached[:3, 3] - goal[:3, 3]))
        rotation = rotation_angle(reached[:3, :3].T @ goal[:3, :3])
        self.assertLess(translation, 0.004)
        self.assertLess(rotation, np.deg2rad(2.0))
        self.assertLess(peak, float(np.max(robot.max_velocity)) * 0.001 * 1.05)
        self.assertGreater(self.robot_sigma(robot, previous), 0.02)

    def robot_sigma(self, robot: XmateErPro7, joints: np.ndarray) -> float:
        return robot.smallest_singular_value(joints)

    def test_nullspace_holds_a_solved_pose(self) -> None:
        robot = XmateErPro7()
        ik = DifferentialIK(robot, robot.neutral)
        pose = robot.fk(robot.neutral)
        for _ in range(20):
            ik.step(pose, 0.02)
        reached = robot.fk(ik.q)
        self.assertLess(float(np.linalg.norm(reached[:3, 3] - pose[:3, 3])), 0.002)


def start_pose_translation(robot: XmateErPro7, joints: np.ndarray) -> np.ndarray:
    return robot.fk(joints)[:3, 3]


@unittest.skipIf(_rokae_cpp is None, "Native ROKAE binding is not built; not validated")
class JointStreamAgreementTest(unittest.TestCase):
    def test_python_and_cpp_streams_match(self) -> None:
        robot = XmateErPro7()
        limits = JointStreamConfig.from_limits(robot.stream_limits)
        python_stream = JointStream(limits)
        cpp_stream = _rokae_cpp.JointRealtimeCore(cpp_config(limits))
        start = robot.neutral.copy()
        goal = start.copy()
        goal[1] += 0.08
        python_stream.reset(start, 0.0)
        cpp_stream.reset(start.tolist(), 0.0)
        sequence = 1
        self.assertTrue(python_stream.publish(goal, 0.0, sequence, 0.0))
        self.assertTrue(cpp_stream.publish(goal.tolist(), 0.0, sequence, 0.0))
        for index in range(1, 1501):
            timestamp = index * 0.001
            if index % 50 == 0:
                sequence += 1
                self.assertTrue(
                    python_stream.publish(goal, timestamp, sequence, timestamp)
                )
                self.assertTrue(
                    cpp_stream.publish(goal.tolist(), timestamp, sequence, timestamp)
                )
            python_command = python_stream.step(timestamp)
            cpp_command = np.asarray(cpp_stream.step(timestamp), dtype=np.float64)
            np.testing.assert_allclose(python_command, cpp_command, atol=1e-8)
        self.assertFalse(python_stream.publish(goal + 2.0, 2.0, sequence + 1, 2.0))
        self.assertFalse(cpp_stream.publish((goal + 2.0).tolist(), 2.0, sequence + 1, 2.0))
        self.assertEqual(python_stream.diagnostics(1.5)["watchdog_state"], "tracking")
        self.assertEqual(
            cpp_stream.diagnostics(1.5)["watchdog_state"],
            python_stream.diagnostics(1.5)["watchdog_state"],
        )

    def test_stale_target_holds_then_stops(self) -> None:
        robot = XmateErPro7()
        limits = JointStreamConfig.from_limits(robot.stream_limits)
        core = _rokae_cpp.JointRealtimeCore(cpp_config(limits))
        start = robot.neutral
        core.reset(start.tolist(), 0.0)
        self.assertTrue(core.publish(start.tolist(), 0.0, 1, 0.0))
        core.step(0.20)
        self.assertEqual(core.diagnostics(0.20)["watchdog_state"], "holding_stale")
        core.step(0.70)
        self.assertEqual(core.diagnostics(0.70)["watchdog_state"], "safe_stop")
        self.assertTrue(core.should_finish)


class InterpolatorTest(unittest.TestCase):
    def test_quintic_endpoints_and_cartesian_slerp(self) -> None:
        robot = XmateErPro7()
        start = robot.neutral
        goal = start.copy()
        goal[3] -= 0.1
        samples, duration = sample_joint_quintic(
            start, goal, robot.max_velocity, robot.max_acceleration
        )
        self.assertGreater(duration, 0.0)
        np.testing.assert_allclose(samples[0], start)
        np.testing.assert_allclose(samples[-1], goal)
        pose_0 = robot.fk(start)
        pose_1 = pose_0.copy()
        pose_1[:3, 3] += np.array([0.01, 0.02, -0.01])
        pose_1[:3, :3] = pose_0[:3, :3] @ np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        )
        cartesian = sample_cartesian(pose_0, pose_1, 1.0, sample_hz=100.0)
        np.testing.assert_allclose(cartesian[0], pose_0, atol=1e-8)
        np.testing.assert_allclose(cartesian[-1], pose_1, atol=1e-8)
        self.assertAlmostEqual(float(np.linalg.det(cartesian[len(cartesian) // 2, :3, :3])), 1.0)


if __name__ == "__main__":
    unittest.main()
