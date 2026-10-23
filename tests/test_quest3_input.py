"""Protocol and coordinate tests for the Quest HTS input adapter."""

import unittest

import numpy as np

from integrations.input.quest3 import _HandState, _parse_line, _quat_to_matrix


class Quest3InputTest(unittest.TestCase):
    def test_official_wrist_csv_shape_and_axis_conversion(self) -> None:
        parsed = _parse_line("Right wrist:,1,2,3,0,0,0,1")
        self.assertIsNotNone(parsed)
        side, kind, values = parsed
        self.assertEqual((side, kind), ("right", "wrist"))
        hand = _HandState("right")
        hand.update_wrist(values)
        np.testing.assert_allclose(hand.wrist_position, [3.0, -1.0, 2.0])

    def test_unity_forward_axis_roll_becomes_robot_world_x_axis_roll(self) -> None:
        # Unity/HTS +Z is converted to this project's +X (forward).  Verify
        # quaternion conversion, not just translation, because this is the
        # wrist-roll axis used by the upright ROKAE init.
        angle = np.deg2rad(20.0)
        hand = _HandState("right")
        self.assertTrue(
            hand.update_wrist(
                [0.0, 0.0, 0.0, 0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0)]
            )
        )
        # Rotation is an axial (pseudo-)vector.  The LH->RH reflection keeps
        # the forward axis but reverses the positive rotation sign.
        expected = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(angle), np.sin(angle)],
                [0.0, -np.sin(angle), np.cos(angle)],
            ]
        )
        # ``world_points`` is unrelated here; inspect the converted wrist
        # quaternion through the same matrix helper used by ``get_wrist_pose``.
        np.testing.assert_allclose(_quat_to_matrix(hand.wrist_quat), expected, atol=1e-12)

    def test_malformed_wrist_does_not_refresh_tracking(self) -> None:
        hand = _HandState("left")
        hand.update_wrist([0.0] * 6)
        self.assertIsNone(hand.wrist_last_update)
        hand.update_wrist([0.0] * 8)
        self.assertIsNone(hand.wrist_last_update)

    def test_landmarks_require_exactly_21_xyz_points(self) -> None:
        hand = _HandState("left")
        hand.update_landmarks([0.0] * 60)
        self.assertIsNone(hand.landmarks_last_update)
        hand.update_landmarks(range(63))
        self.assertEqual(hand.landmarks_local.shape, (21, 3))
        self.assertIsNotNone(hand.landmarks_last_update)


if __name__ == "__main__":
    unittest.main()
