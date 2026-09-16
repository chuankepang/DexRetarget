"""Offline framing/codec regression tests for the custom KUKA TCP client."""

import struct
import time
import unittest

from kuka_iwaa.kuka_iiwa import (
    DT_CART_CUR_POS,
    DT_JOINT_CUR_POS,
    FRAME_SEP,
    KukaIiwa,
)


def state_frame(dtype: int, values: list[int]) -> bytes:
    payload = b"".join(struct.pack(">h", value) for value in values)
    return b"_ST" + bytes([dtype]) + payload + b"_EE" + FRAME_SEP


class KukaTcpFramingTest(unittest.TestCase):
    def test_checking_heartbeat_is_recognized(self) -> None:
        robot = KukaIiwa()
        robot._recv_buf = b"CHECKING" + FRAME_SEP + b"CHECKING" + FRAME_SEP
        robot._parse_buffer()
        self.assertEqual(robot._checking_count, 2)
        self.assertEqual(robot._recv_buf, b"")

    def test_frame_split_at_every_byte_is_preserved_and_decoded(self) -> None:
        frame = state_frame(DT_JOINT_CUR_POS, [100, 200, 300, 400, 500, 600, 700])
        for split in range(1, len(frame)):
            robot = KukaIiwa()
            robot._recv_buf += frame[:split]
            robot._parse_buffer()
            self.assertIsNone(robot.get_joints(), f"split={split}")
            robot._recv_buf += frame[split:]
            robot._parse_buffer()
            decoded = robot.get_joints()
            self.assertIsNotNone(decoded, f"split={split}")
            for actual, expected in zip(
                decoded, [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
            ):
                self.assertAlmostEqual(actual, expected, places=12, msg=f"split={split}")

    def test_multiple_frames_in_one_tcp_chunk(self) -> None:
        robot = KukaIiwa()
        robot._recv_buf = (
            state_frame(DT_JOINT_CUR_POS, [0] * 7)
            + state_frame(DT_CART_CUR_POS, [100, 200, 300, 1, 2, 3])
        )
        robot._parse_buffer()
        self.assertEqual(robot.get_joints(), [0.0] * 7)
        for actual, expected in zip(
            robot.get_pose(), [10.0, 20.0, 30.0, 0.0001, 0.0002, 0.0003]
        ):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(robot._recv_buf, b"")

    def test_wrong_payload_size_is_rejected(self) -> None:
        robot = KukaIiwa()
        robot._recv_buf = state_frame(DT_JOINT_CUR_POS, [1, 2])
        robot._parse_buffer()
        self.assertIsNone(robot.get_joints())


class _RecordingSocket:
    def __init__(self) -> None:
        self.frames = []

    def sendall(self, data: bytes) -> None:
        self.frames.append(bytes(data))


class SmartServoStreamTest(unittest.TestCase):
    def test_latest_target_is_refreshed_and_clear_stops_frames(self) -> None:
        robot = KukaIiwa()
        robot._sock = _RecordingSocket()
        robot.rank = 3
        robot.start_smart_servo_stream(rate_hz=100.0, target_timeout_s=0.10)
        try:
            robot.smart_servo([0.1] * 7)
            deadline = time.monotonic() + 0.15
            while robot._servo_sent_count < 4 and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertGreaterEqual(robot._servo_sent_count, 4)

            robot.clear_smart_servo_target()
            # Allow a frame already copied by the sender to finish, then ensure
            # no stale target continues to be emitted.
            time.sleep(0.025)
            sent_after_clear = robot._servo_sent_count
            time.sleep(0.035)
            self.assertEqual(robot._servo_sent_count, sent_after_clear)
        finally:
            robot.stop_smart_servo_stream()

    def test_smart_servo_without_stream_sends_immediately(self) -> None:
        robot = KukaIiwa()
        robot._sock = _RecordingSocket()
        robot.rank = 3
        robot.smart_servo([0.0] * 7)
        self.assertEqual(len(robot._sock.frames), 1)
        self.assertTrue(robot._sock.frames[0].startswith(b"_CM"))


class GripperProtocolTest(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = KukaIiwa()
        self.robot._sock = _RecordingSocket()
        self.robot.rank = 3

    def test_mode_and_position_use_fixed_four_byte_headers(self) -> None:
        self.robot.gripper_set_mode(0)
        self.robot.gripper_move(90)
        self.assertEqual(self.robot._sock.frames[0], b"_CM\x04\x02\x00\x00_EE#")
        self.assertEqual(self.robot._sock.frames[1], b"_CM\x03\x5a\x07\x00_EE#")

    def test_binary_zero_and_one_map_to_open_and_close_positions(self) -> None:
        self.robot.set_gripper_binary(
            0, open_position=30, close_position=100, mode_settle_s=0.0
        )
        self.robot.set_gripper_binary(
            1, open_position=30, close_position=100, mode_settle_s=0.0
        )
        movement_frames = [
            frame for frame in self.robot._sock.frames if frame.startswith(b"_CM\x03")
        ]
        self.assertEqual(movement_frames[0], b"_CM\x03\x1e\x07\x00_EE#")
        self.assertEqual(movement_frames[1], b"_CM\x03\x64\x07\x00_EE#")

    def test_invalid_binary_command_sends_nothing(self) -> None:
        with self.assertRaises(ValueError):
            self.robot.set_gripper_binary(2, mode_settle_s=0.0)
        self.assertEqual(self.robot._sock.frames, [])


if __name__ == "__main__":
    unittest.main()
