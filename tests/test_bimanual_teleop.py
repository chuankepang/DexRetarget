"""Offline tests for dual-arm world TCP targets and binary gripper commands."""

import json
import unittest

import numpy as np

from anydexretarget.teleop import (
    BinaryGripperMapper,
    ReferenceRelativeWorldMapper,
    invert_transform,
    make_transform,
    rotation_vector_to_matrix,
)
from example.teleop_dual_iiwa_robotiq import (
    SideRuntime,
    _summary,
    _update_side,
    build_packet,
)


def open_hand() -> np.ndarray:
    points = np.zeros((21, 3), dtype=np.float64)
    points[0] = [0.0, 0.0, 0.0]
    points[1:5] = [
        [-0.1, 0.1, 0.0],
        [-0.2, 0.1, 0.0],
        [-0.3, 0.1, 0.0],
        [-0.4, 0.1, 0.0],
    ]
    for start, x in zip((5, 9, 13, 17), (-0.15, -0.05, 0.05, 0.15)):
        points[start : start + 4] = [
            [x, 0.2, 0.0],
            [x, 0.4, 0.0],
            [x, 0.6, 0.0],
            [x, 0.8, 0.0],
        ]
    return points


def closed_hand() -> np.ndarray:
    points = open_hand()
    points[1:5] = [
        [-0.1, 0.1, 0.0],
        [-0.2, 0.1, 0.0],
        [-0.2, 0.1, -0.1],
        [-0.1, 0.1, -0.1],
    ]
    for start, x in zip((5, 9, 13, 17), (-0.15, -0.05, 0.05, 0.15)):
        points[start : start + 4] = [
            [x, 0.2, 0.0],
            [x, 0.4, 0.0],
            [x, 0.4, -0.2],
            [x, 0.2, -0.2],
        ]
    return points


def wrist(position, rotation=None, timestamp=1.0) -> dict:
    return {
        "position": np.asarray(position, dtype=np.float64),
        "rotation": np.eye(3) if rotation is None else rotation,
        "timestamp": timestamp,
    }


class ReferenceRelativeWorldMapperTest(unittest.TestCase):
    def test_translation_is_in_quest_world_not_initial_wrist_axes(self) -> None:
        tcp_start = make_transform([0.5, -0.2, 0.7])
        mapper = ReferenceRelativeWorldMapper(
            np.eye(3), tcp_start, low_pass_alpha=1.0, max_translation=1.0
        )
        wrist_rotation = rotation_vector_to_matrix([0.0, 0.0, np.pi / 2.0])
        mapper.set_reference(make_transform([1.0, 2.0, 3.0], wrist_rotation))
        command = mapper.update(
            make_transform([1.1, 2.0, 3.0], wrist_rotation), 1.0
        )
        np.testing.assert_allclose(command.world_translation_delta, [0.1, 0.0, 0.0])
        np.testing.assert_allclose(command.world_target[:3, 3], [0.6, -0.2, 0.7])

    def test_rotation_delta_is_spatial_and_left_multiplies_tcp_start(self) -> None:
        wrist_start_rotation = rotation_vector_to_matrix([0.0, 0.4, 0.0])
        tcp_start_rotation = rotation_vector_to_matrix([0.0, 0.0, -0.3])
        spatial_delta = rotation_vector_to_matrix([0.2, 0.0, 0.0])
        mapper = ReferenceRelativeWorldMapper(
            np.eye(3),
            make_transform(rotation=tcp_start_rotation),
            low_pass_alpha=1.0,
        )
        mapper.set_reference(make_transform(rotation=wrist_start_rotation))
        command = mapper.update(
            make_transform(rotation=spatial_delta @ wrist_start_rotation), 1.0
        )
        np.testing.assert_allclose(
            command.world_target[:3, :3], spatial_delta @ tcp_start_rotation, atol=1e-10
        )

    def test_axis_mapping_scale_and_total_limit(self) -> None:
        axis_map = np.array(
            [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        )
        mapper = ReferenceRelativeWorldMapper(
            axis_map,
            np.eye(4),
            translation_scale=2.0,
            low_pass_alpha=1.0,
            max_translation=0.1,
        )
        mapper.set_reference(np.eye(4))
        command = mapper.update(make_transform([0.1, 0.0, 0.0]), 1.0)
        np.testing.assert_allclose(command.world_translation_delta, [0.0, 0.1, 0.0])


class BinaryGripperMapperTest(unittest.TestCase):
    def test_open_and_closed_joint_angles_map_to_zero_and_one(self) -> None:
        mapper = BinaryGripperMapper(close_threshold=0.62, open_threshold=0.38)
        opened = mapper.update(open_hand())
        closed = mapper.update(closed_hand())
        self.assertEqual(opened.command, 0)
        self.assertLess(opened.closure_score, 0.05)
        self.assertEqual(closed.command, 1)
        self.assertGreater(closed.closure_score, 0.95)

    def test_hysteresis_prevents_threshold_chatter(self) -> None:
        mapper = BinaryGripperMapper(close_threshold=0.6, open_threshold=0.4)
        self.assertEqual(mapper.update_score(0.61), 1)
        self.assertEqual(mapper.update_score(0.50), 1)
        self.assertEqual(mapper.update_score(0.39), 0)
        self.assertEqual(mapper.update_score(0.50), 0)


class OutputContractTest(unittest.TestCase):
    @staticmethod
    def runtime(world_base=None) -> SideRuntime:
        world_base = np.eye(4) if world_base is None else world_base
        base_tcp_start = make_transform([0.5, 0.0, 0.2])
        return SideRuntime(
            mapper=ReferenceRelativeWorldMapper(
                np.eye(3), world_base @ base_tcp_start, low_pass_alpha=1.0
            ),
            gripper=BinaryGripperMapper(),
            base_world=invert_transform(world_base),
            base_frame="base",
            tcp_frame="tcp",
        )

    def test_recenter_preserves_absolute_world_target(self) -> None:
        runtime = self.runtime()
        empty_fingers = {"fingers": None, "landmarks_timestamp": None}
        first = {**empty_fingers, "wrist": wrist([0.0, 0.0, 0.0], timestamp=1.0)}
        moved = {**empty_fingers, "wrist": wrist([0.1, 0.0, 0.0], timestamp=2.0)}
        _update_side(runtime, first, recenter=False, armed=True)
        _update_side(runtime, moved, recenter=False, armed=True)
        before = runtime.mapper.last_world_target
        self.assertTrue(_update_side(runtime, moved, recenter=True, armed=True))
        np.testing.assert_allclose(runtime.mapper.last_world_target, before, atol=1e-12)

    def test_packet_converts_shared_world_target_to_each_base(self) -> None:
        runtimes = {}
        for side, y in (("left", 0.3), ("right", -0.3)):
            world_base = make_transform([0.0, y, 0.0])
            runtime = self.runtime(world_base)
            runtime.base_frame = f"{side}_base"
            runtime.tcp_frame = f"{side}_tcp"
            snapshot = {
                "wrist": wrist([0.0, 0.0, 0.0], timestamp=9.8),
                "fingers": open_hand(),
                "landmarks_timestamp": 9.8,
            }
            _update_side(runtime, snapshot, recenter=False, armed=True)
            snapshot["wrist"] = wrist([0.1, 0.0, 0.0], timestamp=9.9)
            _update_side(runtime, snapshot, recenter=False, armed=True)
            runtimes[side] = runtime

        packet = build_packet(7, runtimes, now=10.0, tracking_timeout=0.2, armed=True)
        left_base = np.asarray(packet["arms"]["left"]["base_T_tcp_target"]).reshape(4, 4)
        left_world = np.asarray(packet["arms"]["left"]["world_T_tcp_target"]).reshape(4, 4)
        np.testing.assert_allclose(left_base[:3, 3], [0.6, 0.0, 0.2])
        np.testing.assert_allclose(left_world[:3, 3], [0.6, 0.3, 0.2])
        self.assertTrue(packet["arms"]["left"]["valid"])
        self.assertEqual(packet["grippers"]["left"]["command"], 0)
        self.assertEqual(packet["schema"], "dexretarget.dual_iiwa_robotiq.v2")
        self.assertEqual(packet["gripper_command_contract"], "binary_position.v1")
        self.assertLess(len(json.dumps(packet, separators=(",", ":")).encode()), 1472)
        summary = _summary(packet, print_world_tcp_target=True)
        self.assertIn("world_p=[", summary)
        self.assertIn("world_qxyzw=[", summary)

    def test_disarmed_packet_holds_target_and_sets_all_valid_false(self) -> None:
        runtimes = {side: self.runtime() for side in ("left", "right")}
        for runtime in runtimes.values():
            _update_side(
                runtime,
                {
                    "wrist": wrist([0.0, 0.0, 0.0], timestamp=9.9),
                    "fingers": open_hand(),
                    "landmarks_timestamp": 9.9,
                },
                recenter=False,
                armed=False,
            )
        packet = build_packet(1, runtimes, 10.0, 0.2, armed=False)
        self.assertFalse(packet["teleop_enabled"])
        for side in ("left", "right"):
            self.assertFalse(packet["arms"][side]["valid"])
            self.assertFalse(packet["grippers"][side]["valid"])
            self.assertEqual(packet["arms"][side]["status"], "disarmed")


if __name__ == "__main__":
    unittest.main()
