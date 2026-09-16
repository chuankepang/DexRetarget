"""Offline POE kinematics and Cartesian bridge regression tests."""

import copy
from pathlib import Path
import unittest

import numpy as np
import yaml

from kuka_iwaa.cartesian_teleop_bridge import (
    CartesianTeleopBridge,
    IKConfig,
    IiwaKinematics,
    invert_transform,
    pose_distance,
    self_test,
)


ROOT = Path(__file__).resolve().parents[1]


def config():
    with (ROOT / "kuka_iwaa/cartesian_teleop.yaml").open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class PoeKinematicsTest(unittest.TestCase):
    def test_home_configuration_matches_document_M(self):
        cfg = config()
        model = IiwaKinematics(cfg["arms"]["left"], IKConfig.from_mapping(cfg["solver"]))
        expected = np.eye(4)
        expected[2, 3] = 1.332
        np.testing.assert_allclose(model.forward(np.zeros(7)), expected, atol=1e-12)

    def test_fk_ik_round_trip_has_multiple_candidates(self):
        cfg = config()
        model = IiwaKinematics(cfg["arms"]["left"], IKConfig.from_mapping(cfg["solver"]))
        q = np.array([0.2, -0.35, 0.25, -0.55, 0.2, 0.45, -0.15])
        candidates = model.solve_candidates(model.forward(q), q + 0.05)
        self.assertGreaterEqual(len(candidates), 2)
        position, rotation = pose_distance(model.forward(candidates[0].joints), model.forward(q))
        self.assertLessEqual(position, model.config.position_tolerance_m)
        self.assertLessEqual(rotation, model.config.rotation_tolerance_rad)

    def test_left_world_base_matrix_is_rigid_and_invertible(self):
        transform = np.asarray(config()["arms"]["left"]["world_T_base"])
        np.testing.assert_allclose(invert_transform(transform) @ transform, np.eye(4), atol=1e-9)

    def test_live_joint_sample_matches_controller_cartesian_state(self):
        cfg = config()
        arm = cfg["arms"]["left"]
        model = IiwaKinematics(arm, IKConfig.from_mapping(cfg["solver"]))
        joints = np.array([0.0, 0.0, 0.0, -1.6467, 0.0, 0.0, 0.0])
        controller_pose_T_base = np.asarray(arm["controller_pose_T_base"])
        predicted = controller_pose_T_base @ model.forward(joints)
        np.testing.assert_allclose(
            predicted[:3, 3], np.array([0.3892, 0.7060, 0.4446]), atol=1.5e-4
        )
        # Expected rotation reconstructed from the received KUKA ABC sample.
        from kuka_iwaa.cartesian_teleop_bridge import kuka_abc_pose_to_matrix

        received = kuka_abc_pose_to_matrix(
            [389.2, 706.0, 444.6, -1.6688, 0.9913, -2.4397]
        )
        position_error, rotation_error = pose_distance(predicted, received)
        self.assertLess(position_error, 0.00015)
        self.assertLess(rotation_error, 0.00006)

class BridgeContractTest(unittest.TestCase):
    def test_only_left_arm_is_enabled(self):
        bridge = CartesianTeleopBridge(config(), dry_run=True)
        self.assertEqual(set(bridge.runtimes), {"left"})

    def test_unconfirmed_real_output_is_rejected(self):
        cfg = copy.deepcopy(config())
        cfg["arms"]["left"]["model_confirmed"] = False
        with self.assertRaisesRegex(ValueError, "real output blocked"):
            CartesianTeleopBridge(cfg, dry_run=False)

    def test_binary_gripper_command_reaches_low_level_driver_once_per_change(self):
        class FakeRobot:
            def __init__(self):
                self.commands = []

            def set_gripper_binary(self, command, **kwargs):
                self.commands.append((command, kwargs))

        bridge = CartesianTeleopBridge(config(), dry_run=True)
        bridge.connect()
        runtime = bridge.runtimes["left"]
        fake = FakeRobot()
        runtime.robot = fake
        packet = {
            "schema": "dexretarget.dual_iiwa_robotiq.v2",
            "teleop_enabled": True,
            "arm_command_contract": "reference_relative_world_delta.v1",
            "arms": {"left": {"valid": False}},
            "grippers": {"left": {"valid": True, "command": 1}},
        }
        bridge.process_packet(packet)
        bridge.process_packet(packet)
        packet["grippers"]["left"]["command"] = 0
        bridge.process_packet(packet)

        self.assertEqual([item[0] for item in fake.commands], [1, 0])
        self.assertEqual(fake.commands[0][1]["close_position"], 100)
        self.assertEqual(fake.commands[1][1]["open_position"], 30)


if __name__ == "__main__":
    unittest.main()
