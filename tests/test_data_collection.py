"""Acquisition integrity: causal pairing, lifecycle, loss handling and serial wire format."""

import json
import struct
import threading
import time
from pathlib import Path

import cv2
import h5py
import numpy as np
import pytest

from dexretarget.dataset.config import load_config
from dexretarget.dataset.storage import EpisodeWriter, trash_episode, restore_episode
from dexretarget.dataset.processing import validate_episode
from dexretarget.teleop.devices.inspire import InspireSerialOutput
from dexretarget.teleop.devices.rokae import MockRokaeDriver


def fixture_episode(tmp_path, stale=False, paused=False):
    config = load_config()
    config["output_root"] = str(tmp_path)
    config["min_free_disk_gb"] = 0
    config["queue_capacity"] = 2048
    writer = EpisodeWriter(config, "pick up the cup", {"simulated": True}, start=100)
    camera_shape = config["cameras"]
    image = np.zeros((camera_shape["height"], camera_shape["width"], 3), dtype="u1")
    jpeg = cv2.imencode(".jpg", image)[1].tobytes()
    pose = np.eye(4)
    for i in range(32):
        t = 100 + i / 20
        for role in ["wrist", "global"]:
            writer.submit("camera_" + role, t - 0.002, i, {
                "jpeg": jpeg, "depth_png": b"", "capture_t": t - 0.003,
                "device_time_ms": (t - 0.003) * 1000, "clock_offset_s": 0,
                "timestamp_domain": "synthetic", "calibration_json": "{}",
            })
        observed = pose.copy()
        observed[0, 3] = 0.01 * i
        writer.submit("arm_state", t - 0.001, i, {
            "tcp": observed, "qpos": np.zeros(7), "qvel": np.zeros(7),
            "read_start": t - 0.002, "read_end": t - 0.001,
        })
        if not stale or i < 10:
            writer.submit("hand_state", t - 0.001, i, {
                "angle": np.full(6, 100), "force": np.zeros(6), "force_valid": True,
                "read_start": t - 0.002, "read_end": t - 0.001,
            })
        writer.submit("quest", t - 0.001, i, {
            "wrist": pose, "landmarks": np.zeros((21, 3)),
            "wrist_timestamp": t - 0.001, "landmarks_timestamp": t - 0.001,
        })
        writer.submit("control", t - 0.001, i, {"enabled": not (paused and i == 16)})
        target = pose.copy()
        target[0, 3] = 1 + 0.01 * i
        writer.submit("arm_command", t + 0.010, i, {
            "requested_tcp": target, "accepted_tcp": target, "enabled": True,
        })
        writer.submit("hand_command", t + 0.011, i, {"angle": np.full(6, 900), "acknowledged": True})
    return writer.finish(True, stop=101.6)


def test_causal_state_and_accepted_action_remain_distinct(tmp_path):
    path = fixture_episode(tmp_path)
    metadata = validate_episode(path)
    assert metadata["quality"]["valid_frames"] == 32
    with h5py.File(path / "episode.h5") as file:
        samples = file["samples"]
        np.testing.assert_allclose(samples["state"][:, 6:], 0.1)
        np.testing.assert_allclose(samples["action"][:, 6:], 0.9)
        assert samples["state"][0, 0] == 0
        assert samples["action"][0, 0] == 1
        for row, t in enumerate(samples["t"]):
            indices = samples["raw_indices"][row]
            assert file["streams/arm_state/t"][indices[2]] <= t
            assert t <= file["streams/arm_command/t"][indices[6]] < t + 0.05
    assert not any(path.parent.glob(".partial-*"))


def test_stale_feedback_is_flagged_instead_of_filled_from_commands(tmp_path):
    path = fixture_episode(tmp_path, stale=True)
    with h5py.File(path / "episode.h5") as file:
        assert not file["samples/valid"][-1]
        assert file["samples/invalid_reasons"][-1] & 2
        assert len(file["streams/hand_state/t"]) == 10


def test_pause_breaks_continuity(tmp_path):
    path = fixture_episode(tmp_path, paused=True)
    with h5py.File(path / "episode.h5") as file:
        assert not file["samples/valid"][16]
        assert file["samples/invalid_reasons"][16] & 16
        # Neither shorter side may be stitched together into a >20 frame run.
        assert len(file["samples/segments"]) == 0


def test_reversible_deletion_and_path_rejection(tmp_path):
    path = fixture_episode(tmp_path)
    trash = trash_episode(path.parent, path.name)
    assert not path.exists() and trash.is_dir()
    restored = restore_episode(path.parent, trash.name)
    assert restored == path and validate_episode(path)["success"]
    with pytest.raises(ValueError):
        trash_episode(path.parent, "../QA.md")


def test_writer_failure_remains_partial_and_unexportable(tmp_path):
    config = load_config()
    config.update(output_root=str(tmp_path), min_free_disk_gb=0)
    writer = EpisodeWriter(config, "failure", {"simulated": True})
    writer.submit("arm_state", time.monotonic(), 0, {"tcp": np.zeros((2, 2))})
    path = writer.finish()
    assert path.name.startswith(".partial-")
    assert json.loads((path / "metadata.json").read_text())["status"] == "invalid"
    with pytest.raises(ValueError):
        validate_episode(path)


def response(command, address, payload, hand_id=1):
    frame = bytearray([0x90, 0xEB, hand_id, len(payload) + 3, command, address & 255, address >> 8])
    frame.extend(payload)
    frame.append(sum(frame[2:]) & 255)
    return bytes(frame)


class FakePort:
    def __init__(self, bad_checksum=False):
        self.incoming = bytearray()
        self.bad_checksum = bad_checksum
        self.writes = []

    @property
    def in_waiting(self):
        return 1  # Deliberately fragment every response.

    def read(self, count):
        data = bytes(self.incoming[:count])
        del self.incoming[:count]
        return data

    def reset_input_buffer(self):
        self.incoming.clear()

    def write(self, packet):
        self.writes.append(bytes(packet))
        address = packet[5] | packet[6] << 8
        payload = struct.pack("<6h", *range(101, 107)) if packet[4] == 0x11 else b"\x01"
        reply = bytearray(response(packet[4], address, payload))
        if self.bad_checksum:
            reply[-1] ^= 1
        self.incoming.extend(b"noise" + reply)
        return len(packet)


def fake_hand(bad_checksum=False):
    hand = InspireSerialOutput.__new__(InspireSerialOutput)
    hand._port = FakePort(bad_checksum)
    hand._hand_id = 1
    hand._io_lock = threading.RLock()
    hand._feedback_enabled = True
    hand._capture_timeout = 0.005
    return hand


def test_real_feedback_register_and_fragmented_checksum_validation():
    hand = fake_hand()
    result = hand.read_feedback()
    np.testing.assert_equal(result["angle"], np.arange(101, 107))
    assert result["force_valid"]
    assert hand._port.writes[0] == bytes.fromhex("eb900104110a060c32")
    with pytest.raises(TimeoutError):
        fake_hand(True).read_feedback()


def test_write_acknowledgement_and_original_hand_mapping():
    hand = fake_hand()
    result = hand.send(np.zeros(12), [])
    assert result["acknowledged"]
    np.testing.assert_equal(result["angle"], np.full(6, 1000))
    assert hand._port.writes[0][:7] == bytes.fromhex("eb90010f12ce05")


def test_mock_hold_does_not_consume_initial_or_resume_command_sequence():
    driver = MockRokaeDriver(np.eye(4))
    driver.connect()
    driver.start()
    try:
        driver.hold()
        assert driver.set_target_pose(np.eye(4), sequence=0)
        driver.hold()
        assert driver.set_target_pose(np.eye(4), sequence=1)
        assert driver.diagnostics()["buffer"].rejected_sequence == 0
    finally:
        driver.disconnect()


def test_home_hand_angles_use_native_registers_without_retargeting():
    hand = fake_hand()
    target = [100, 200, 300, 400, 500, 600]
    result = hand.send_angles(target)
    assert result['acknowledged']
    assert struct.unpack('<6h', hand._port.writes[-1][7:-1]) == tuple(target)
    with pytest.raises(ValueError):
        hand.send_angles([float('nan')]*6)
    with pytest.raises(ValueError):
        hand.send_angles([1001]*6)
