"""Offline tests for the position/impedance controller split."""

from __future__ import annotations

import threading
from native_support import HAS_NATIVE
import unittest

import numpy as np

from rokae.control.command import EECommand
from rokae.control.controllers import (
    CompliantPlant,
    ImpedanceController,
    PositionController,
    create_controller,
    resolve_controller,
)
from rokae.control.reference import PoseRateLimiter, RateLimit
from rokae.control.reference_limit import (
    limit_equilibrium,
    quaternion_xyzw_to_matrix,
    rotation_vector,
)


def _pose(xyz=(0.4, 0.0, 0.3), rotation=None) -> np.ndarray:
    pose = np.eye(4, dtype=np.float64)
    if rotation is not None:
        pose[:3, :3] = rotation
    pose[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return pose


class _FakeDriver:
    def __init__(self) -> None:
        self.mode = None
        self.pose = _pose()
        self.targets = []
        self.stopped = False

    def start(self, power_on: bool = True, control_mode: str = "cartesian", kinematics=None) -> None:
        del power_on, kinematics
        self.mode = control_mode

    def set_target_pose(self, pose, *, source_timestamp=None, sequence=None) -> bool:
        del source_timestamp, sequence
        self.targets.append(np.array(pose, dtype=np.float64, copy=True))
        return True

    def set_target_joints(self, joints, *, source_timestamp=None, sequence=None) -> bool:
        del joints, source_timestamp, sequence
        return True

    def get_command_pose(self):
        return self.pose.copy()

    def get_tcp_pose(self):
        return self.pose.copy()

    def get_joint_positions(self):
        return None

    def get_command_joints(self):
        return np.zeros(7)

    def hold(self) -> None:
        return None

    def stop(self) -> None:
        self.stopped = True
        self.mode = None

    def move_cartesian(self, pose, *, speed, timeout, max_translation_delta, max_rotation_delta, power_on) -> None:
        del speed, timeout, max_translation_delta, max_rotation_delta, power_on
        self.targets.append(np.array(pose, dtype=np.float64, copy=True))


class ControllerFactoryTest(unittest.TestCase):
    def test_position_and_impedance_selection(self) -> None:
        position = resolve_controller({"control": {"mode": "pinocchio"}})
        self.assertEqual(position.kind, "position")
        self.assertEqual(position.position_backend, "pinocchio")
        self.assertIsInstance(
            create_controller(position, _FakeDriver()), PositionController
        )

        impedance = resolve_controller(
            {"control": {"mode": "pinocchio", "controller": "impedance"}}
        )
        self.assertEqual(impedance.kind, "impedance")
        self.assertIsInstance(
            create_controller(impedance, _FakeDriver()), ImpedanceController
        )

    def test_precedence_and_invalid(self) -> None:
        config = {
            "control": {"mode": "cartesian", "controller": "position"},
            "tasks": {"cup": {"controller": "impedance"}},
        }
        self.assertEqual(resolve_controller(config, task="cup").kind, "impedance")
        self.assertEqual(
            resolve_controller(config, cli="pinocchio", task="cup").route, "pinocchio"
        )
        with self.assertRaises(ValueError):
            resolve_controller({"control": {"controller": "torque"}})
        with self.assertRaises(ValueError):
            resolve_controller(
                {"control": {"controller": "impedance", "impedance": {"backend": "torque"}}}
            )


class ReferenceLimitTest(unittest.TestCase):
    def test_translation_clip_and_zero(self) -> None:
        measured = _pose()
        same = limit_equilibrium(measured, measured, 0.04, 0.3)
        self.assertLess(np.linalg.norm(same - measured), 1e-9)
        far = _pose(xyz=(1.4, 0.2, 0.3))
        limited = limit_equilibrium(far, measured, 0.04, 0.3)
        delta = limited[:3, 3] - measured[:3, 3]
        self.assertAlmostEqual(float(np.linalg.norm(delta)), 0.04, places=6)
        direction = far[:3, 3] - measured[:3, 3]
        self.assertGreater(float(delta @ direction), 0.0)

    def test_rotation_clip_and_quaternion_sign(self) -> None:
        measured = _pose()
        angle = 1.2
        axis = np.array([0.0, 0.0, 1.0])
        skew = np.array(
            [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
        )
        rotation = np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)
        desired = _pose(rotation=rotation)
        limited = limit_equilibrium(desired, measured, 0.05, 0.30)
        turned = float(np.linalg.norm(rotation_vector(limited[:3, :3])))
        self.assertAlmostEqual(turned, 0.30, places=5)
        quat = np.array([0.0, 0.2, 0.0, 0.8])
        self.assertLess(
            np.linalg.norm(quaternion_xyzw_to_matrix(quat) - quaternion_xyzw_to_matrix(-quat)),
            1e-9,
        )

    def test_rejects_non_finite(self) -> None:
        measured = _pose()
        bad = measured.copy()
        bad[0, 3] = np.nan
        with self.assertRaises(ValueError):
            limit_equilibrium(bad, measured, 0.04, 0.3)
        bad[0, 3] = np.inf
        with self.assertRaises(ValueError):
            limit_equilibrium(bad, measured, 0.04, 0.3)


class RateLimitTest(unittest.TestCase):
    def test_no_jump_and_speed_bound(self) -> None:
        limiter = PoseRateLimiter(
            RateLimit(
                max_translation_speed=0.15,
                max_angular_speed=0.5,
                max_translation_acceleration=0.40,
                max_angular_acceleration=1.5,
                max_dt=0.05,
            )
        )
        start = _pose()
        limiter.reset(start, 0.0)
        far = _pose(xyz=(2.0, 0.0, 0.3))
        stepped = limiter.apply(far, 0.01)
        moved = float(np.linalg.norm(stepped[:3, 3] - start[:3, 3]))
        self.assertLessEqual(moved, 0.15 * 0.01 + 1e-9)
        self.assertGreater(moved, 0.0)


class LifecycleTest(unittest.TestCase):
    def test_impedance_transitions_and_limit(self) -> None:
        spec = resolve_controller(
            {
                "control": {
                    "controller": "impedance",
                    "impedance": {"reference_limit": {"translation": 0.04, "rotation": 0.3}},
                }
            }
        )
        driver = _FakeDriver()
        controller = create_controller(spec, driver)
        with self.assertRaises(RuntimeError):
            controller.set_target(EECommand(pose=_pose(), timestamp=1.0, sequence=1))
        controller.start()
        self.assertEqual(controller.state, "running")
        self.assertEqual(driver.mode, "impedance")
        controller.start()
        far = _pose(xyz=(1.0, 0.0, 0.3))
        accepted = controller.set_target(
            EECommand(pose=far, timestamp=1.0, sequence=1, source="test")
        )
        self.assertTrue(accepted)
        error = driver.targets[-1][:3, 3] - driver.pose[:3, 3]
        self.assertAlmostEqual(float(np.linalg.norm(error)), 0.04, places=6)
        controller.stop()
        self.assertEqual(controller.state, "stopped")
        self.assertTrue(driver.stopped)
        with self.assertRaises(RuntimeError):
            controller.set_target(EECommand(pose=_pose(), timestamp=2.0, sequence=2))

    def test_threads(self) -> None:
        spec = resolve_controller({"control": {"controller": "impedance"}})
        driver = _FakeDriver()
        controller = create_controller(spec, driver)
        controller.start()
        errors = []

        def publish(offset: int) -> None:
            try:
                for index in range(40):
                    pose = _pose(xyz=(0.4 + 0.2 * ((index + offset) % 3), 0.0, 0.3))
                    controller.set_target(
                        EECommand(
                            pose=pose,
                            timestamp=float(index) * 0.01,
                            sequence=offset * 100 + index,
                            source="thread",
                        )
                    )
            except Exception as exc:  # noqa: BLE001 - the test records any race
                errors.append(exc)

        threads = [threading.Thread(target=publish, args=(item,)) for item in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])
        self.assertTrue(driver.targets)
        self.assertTrue(all(np.all(np.isfinite(pose)) for pose in driver.targets))


class PlantTest(unittest.TestCase):
    def test_spring_sign(self) -> None:
        pose = _pose()
        plant = CompliantPlant(
            pose,
            stiffness=np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0]),
            damping=np.zeros(6),
            mass=1.0,
        )
        goal = _pose(xyz=(0.41, 0.0, 0.3))
        plant.set_equilibrium(goal)
        wrench = plant.step(0.01)
        self.assertGreater(wrench[0], 0.0)
        self.assertGreater(plant.pose[0, 3], pose[0, 3])


@unittest.skipUnless(HAS_NATIVE, "Native ROKAE binding is not built; not validated")
class SharedRealtimeCoreTest(unittest.TestCase):
    def test_watchdog_and_cpp_limiter(self) -> None:
        from rokae.python import _rokae_cpp

        config = _rokae_cpp.RealtimeConfig()
        config.hold_timeout = 0.05
        config.stop_timeout = 0.10
        config.max_source_age = 0.20
        config.validate()
        core = _rokae_cpp.RealtimeCore(config)
        start = _pose()
        core.reset(start, 0.0)
        self.assertTrue(core.publish(start, 0.0, 1, 0.0))
        core.step(0.20)
        self.assertTrue(core.should_finish)

        measured = _pose()
        desired = _pose(xyz=(1.0, 0.0, 0.3))
        cpp = np.asarray(
            _rokae_cpp.limit_equilibrium_pose(desired, measured, 0.04, 0.30),
            dtype=np.float64,
        )
        python = limit_equilibrium(desired, measured, 0.04, 0.30)
        self.assertLess(np.linalg.norm(cpp - python), 1e-6)
        from rokae.control.reference_limit import rotation_from_vector

        turned = _pose(
            xyz=(0.7, -0.2, 0.5),
            rotation=rotation_from_vector(np.array([0.2, -0.4, 0.9])),
        )
        cpp_rot = np.asarray(
            _rokae_cpp.limit_equilibrium_pose(turned, measured, 0.04, 0.25),
            dtype=np.float64,
        )
        python_rot = limit_equilibrium(turned, measured, 0.04, 0.25)
        self.assertLess(np.linalg.norm(cpp_rot - python_rot), 1e-6)


if __name__ == "__main__":
    unittest.main()


def test_tracking_reset_does_not_integrate_the_pause_as_an_ik_step():
    driver = _FakeDriver()
    controller = PositionController(resolve_controller({'control': {'mode': 'pinocchio'}}), driver)
    periods, resets = [], []
    class Solver:
        def step(self, pose, dt):
            periods.append(dt)
            return np.zeros(7)
        def reset(self, joints):
            resets.append(np.asarray(joints).copy())
    controller.ik = Solver()
    controller.state = 'running'
    controller.set_target(EECommand(pose=_pose(), timestamp=1., sequence=0))
    controller.reset_tracking()
    controller.set_target(EECommand(pose=_pose(), timestamp=101., sequence=1))
    assert periods == [.02, .02]
    assert len(resets) == 1
    np.testing.assert_equal(resets[0], driver.get_command_joints())


def test_impedance_invalid_feedback_never_publishes_a_new_equilibrium():
    import pytest

    driver = _FakeDriver()
    controller = create_controller(resolve_controller({'control': {'controller': 'impedance'}}), driver)
    controller.start()
    driver.pose[0, 3] = np.nan
    try:
        with pytest.raises(ValueError):
            controller.set_target(EECommand(pose=_pose(), timestamp=1.0, sequence=1))
        assert driver.targets == []
    finally:
        controller.stop()
