"""Direct serial protocol driver for Inspire RH56DFX hand."""

import time
import threading
import struct
from contextlib import nullcontext

import numpy as np

from .base import HandOutput

_INSPIRE_CHANNEL_INDICES = [4, 6, 2, 0, 9, 8]
_INSPIRE_CHANNEL_MAX_RAD = [1.47, 1.47, 1.47, 1.47, 0.6, 1.308]
_INSPIRE_CHANNEL_INVERT = [True, True, True, True, True, True]


def _inspire_retarget_to_real(retarget_output: np.ndarray) -> list[int]:
    """Map 12-D Inspire retarget output (rad) to 6 serial control channels."""
    output = np.asarray(retarget_output, dtype=np.float64)
    if output.shape[0] < 12:
        raise ValueError(f"Expected at least 12 Inspire joints, got shape {output.shape}")
    if not np.all(np.isfinite(output)):
        raise ValueError("Invalid Inspire retarget output: contains NaN/Inf")

    result: list[int] = []
    for idx, max_rad, invert in zip(
        _INSPIRE_CHANNEL_INDICES,
        _INSPIRE_CHANNEL_MAX_RAD,
        _INSPIRE_CHANNEL_INVERT,
    ):
        value = float(np.clip(output[idx] / max_rad, 0.0, 1.0))
        if invert:
            value = 1.0 - value
        result.append(int(value * 2000))
    return result


class InspireSerialOutput(HandOutput):
    """Direct serial controller for Inspire RH56DFX hand."""

    def __init__(self, port_name: str, baudrate: int = 115200, hand_id: int = 1):
        try:
            import serial
        except ImportError as exc:
            raise ImportError(
                "pyserial is required for Inspire hand control. "
                "Install it with `pip install pyserial`."
            ) from exc

        self._port = serial.Serial(port_name, baudrate, timeout=0.01)
        self._hand_id = int(hand_id)
        self._port_name = port_name
        self._baudrate = int(baudrate)
        self._io_lock = threading.RLock()
        self._feedback_enabled = False
        self._capture_timeout = 0.025
        print(f"Connected to Inspire hand serial at {port_name} @ {baudrate} baud.")

    @staticmethod
    def _encode_channels(channels: list[int]) -> list[int]:
        if len(channels) != 6:
            raise ValueError(f"Inspire hand expects 6 channels, got {len(channels)}")
        return [int(np.clip(round(ch / 2.0), 0, 1000)) for ch in channels]

    def send(self, qpos, joint_names, *, gate=None, source_timestamp=None):
        channels = _inspire_retarget_to_real(qpos)
        encoded = self._encode_channels(channels)
        return self.send_angles(encoded, gate=gate, source_timestamp=source_timestamp)

    def send_angles(self, angles, *, gate=None, source_timestamp=None):
        """Send native ANGLE_SET registers without applying retargeting twice."""
        values = np.asarray(angles, dtype=float)
        if values.shape != (6,) or not np.all(np.isfinite(values)) or np.any((values < 0) | (values > 1000)):
            raise ValueError("Expected six finite native hand angles in 0..1000")
        encoded = np.rint(values).astype(int).tolist()
        packet = bytearray([0xEB, 0x90, self._hand_id, 0x0F, 0x12, 0xCE, 0x05])
        for angle in encoded:
            packet.append(angle & 0xFF)
            packet.append((angle >> 8) & 0xFF)
        checksum = sum(packet[2:2 + 0x0F + 3])
        packet.append(checksum & 0xFF)
        with self._io_lock:
            # Wait for serial ownership without blocking arm publication. Only
            # the write is serialized with pause/home; ACK waiting is not.
            with gate.lock if gate is not None else nullcontext():
                if gate is not None and (not gate.enabled or gate.tracking_held):
                    return None
                timestamp = time.monotonic()
                if source_timestamp is not None:
                    from ..tracking import fresh_timestamp
                    if not fresh_timestamp(source_timestamp, timestamp, .20):
                        return None
                written = self._port.write(packet)
                if written != len(packet):
                    raise IOError("Incomplete Inspire command write")
            if self._feedback_enabled:
                # A transmitted target is not called accepted until the hand
                # returns the matching write acknowledgement.
                response = self._read_packet(0x12, 1486)
                if len(response) != 1 or response[0] != 1:
                    raise IOError("Inspire rejected ANGLE_SET")
                acknowledged = True
            else:
                # Exit on the complete ACK, not an extra serial silence timeout.
                # Legacy no-capture mode still tolerates an absent ACK and does
                # not label the target as acknowledged in the dataset contract.
                try:
                    self._read_packet(0x12, 1486)
                except TimeoutError:
                    pass
                acknowledged = False
        return {"angle": np.asarray(encoded, dtype=np.int16), "timestamp": timestamp,
                "acknowledged": acknowledged}

    def enable_feedback(self, timeout=0.025):
        """Enable validated transactions, sharing the existing serial owner."""
        if not 0 < timeout <= 0.1:
            raise ValueError("Feedback timeout must be in (0, 0.1]")
        with self._io_lock:
            self._capture_timeout = float(timeout)
            self._port.reset_input_buffer()
            self._feedback_enabled = True

    def _read_packet(self, command, address):
        """Parse fragmented/noisy frames; check header, ID, command, address, checksum."""
        deadline = time.monotonic() + self._capture_timeout
        buffer = bytearray()
        while time.monotonic() < deadline:
            buffer.extend(self._port.read(self._port.in_waiting or 1))
            while True:
                header = buffer.find(b"\x90\xeb")
                if header < 0:
                    buffer[:] = buffer[-1:]
                    break
                if header:
                    del buffer[:header]
                if len(buffer) < 4:
                    break
                length = buffer[3] + 5
                if not 8 <= length <= 128:
                    del buffer[0]
                    continue
                if len(buffer) < length:
                    break
                frame = bytes(buffer[:length])
                del buffer[:length]
                if (sum(frame[2:-1]) & 0xFF) != frame[-1]:
                    continue
                if frame[2] != self._hand_id or frame[4] != command:
                    continue
                if frame[5] | (frame[6] << 8) != address:
                    continue
                return frame[7:-1]
        raise TimeoutError(f"No valid Inspire response for command {command:#x}, register {address}")

    def _read_register(self, address, count):
        packet = bytearray([0xEB, 0x90, self._hand_id, 4, 0x11, address & 0xFF, address >> 8, count])
        packet.append(sum(packet[2:]) & 0xFF)
        self._port.reset_input_buffer()
        if self._port.write(packet) != len(packet):
            raise IOError("Incomplete Inspire read request")
        payload = self._read_packet(0x11, address)
        if len(payload) != count:
            raise IOError("Unexpected Inspire register response length")
        return payload

    def read_feedback(self, read_force=True):
        """Read actual ANGLE_ACT, never substitute the previous command.

        Register 1546: six 0..1000 angle values, channel order little/ring/
        middle/index/thumb bend/thumb rotation. Force remains in vendor units.
        """
        if not self._feedback_enabled:
            raise RuntimeError("Call enable_feedback before reading capture state")
        with self._io_lock:
            begin = time.monotonic()
            angles = np.asarray(struct.unpack("<6h", self._read_register(1546, 12)), dtype=np.int16)
            if np.any((angles < 0) | (angles > 1000)):
                raise ValueError("Inspire actual angle outside 0..1000")
            forces = np.full(6, -1, dtype=np.int16)
            force_valid = False
            if read_force:
                try:
                    forces = np.asarray(struct.unpack("<6h", self._read_register(1582, 12)), dtype=np.int16)
                    force_valid = True
                except (IOError, TimeoutError):
                    pass
            return {"angle": angles, "force": forces, "force_valid": force_valid,
                    "read_start": begin, "read_end": time.monotonic()}

    def close(self):
        with self._io_lock:
            if self._port.is_open:
                self._port.close()
                print(f"Closed Inspire hand serial at {self._port_name} @ {self._baudrate} baud.")


__all__ = ["InspireSerialOutput", "_inspire_retarget_to_real"]
