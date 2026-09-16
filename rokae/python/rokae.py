"""Public Python wrapper for the vendored ROKAE C++ driver.

Build once with ``bash rokae/build.sh``.  This module deliberately has no
dependency on xCoreSDK-Python.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np

try:
    from . import _rokae_cpp
except ImportError as exc:  # pragma: no cover - exercised by a missing-build install
    raise ImportError(
        "ROKAE C++ bridge is not built. Run `bash rokae/build.sh` from the "
        "DexRetarget repository root."
    ) from exc


SDK_VERSION = _rokae_cpp.SDK_VERSION
RealtimeConfig = _rokae_cpp.RealtimeConfig
monotonic_time = _rokae_cpp.monotonic_time


class Rokae:
    """Context-manageable Python facade; all robot operations execute in C++."""

    def __init__(
        self,
        robot_ip: str = "192.168.0.160",
        local_ip: str = "192.168.0.100",
        robot_type: str = "xmate-er-pro-7",
        realtime_config: RealtimeConfig | None = None,
    ) -> None:
        self._driver = _rokae_cpp.RokaeDriver(
            robot_ip,
            local_ip,
            robot_type,
            realtime_config or RealtimeConfig(),
        )

    def __enter__(self) -> "Rokae":
        self.connect()
        return self

    def __exit__(self, *_: Any) -> None:
        self.disconnect()

    def connect(self) -> None:
        self._driver.connect()

    def disconnect(self) -> None:
        self._driver.disconnect()

    def get_state(self) -> dict[str, Any]:
        return dict(self._driver.get_state())

    def get_joint_positions(self) -> np.ndarray:
        return np.asarray(self._driver.get_joint_positions(), dtype=np.float64)

    def get_tcp_pose(self) -> np.ndarray:
        return np.asarray(self._driver.get_tcp_pose(), dtype=np.float64)

    def get_base_frame(self) -> np.ndarray:
        return np.asarray(self._driver.get_base_frame(), dtype=np.float64)

    def calculate_fk(self, joints: Sequence[float]) -> np.ndarray:
        return np.asarray(self._driver.calculate_fk(list(joints)), dtype=np.float64)

    def move_joint(
        self,
        target: Sequence[float],
        speed: int = 50,
        timeout: float = 60.0,
        max_joint_delta: float = 0.25,
        power_on: bool = True,
    ) -> None:
        self._driver.move_joint(
            list(target), speed, timeout, max_joint_delta, power_on
        )

    def move_cartesian(
        self,
        target: np.ndarray,
        speed: int = 20,
        timeout: float = 60.0,
        max_translation_delta: float = 0.05,
        max_rotation_delta: float = 0.20,
        power_on: bool = True,
    ) -> None:
        self._driver.move_cartesian(
            np.asarray(target, dtype=np.float64),
            speed,
            timeout,
            max_translation_delta,
            max_rotation_delta,
            power_on,
        )

    def move_to_init(
        self,
        target: Sequence[float],
        speed: int = 50,
        timeout: float = 60.0,
        max_joint_delta: float = 1.2,
        power_on: bool = True,
    ) -> None:
        self._driver.move_to_init(
            list(target), speed, timeout, max_joint_delta, power_on
        )

    def start_realtime_cartesian(self, power_on: bool = True) -> None:
        self._driver.start_realtime_cartesian(power_on)

    def set_target_pose(
        self, target: np.ndarray, source_timestamp: float, sequence: int
    ) -> bool:
        return bool(
            self._driver.set_target_pose(
                np.asarray(target, dtype=np.float64), source_timestamp, sequence
            )
        )

    def get_command_pose(self) -> np.ndarray:
        return np.asarray(self._driver.get_command_pose(), dtype=np.float64)

    def hold(self) -> None:
        self._driver.hold()

    def stop_realtime(self) -> None:
        self._driver.stop_realtime()

    def stop(self) -> None:
        self._driver.stop()

    def diagnostics(self) -> dict[str, Any]:
        return dict(self._driver.diagnostics())


def module_file() -> Path:
    return Path(_rokae_cpp.__file__).resolve()


__all__ = [
    "Rokae",
    "RealtimeConfig",
    "SDK_VERSION",
    "module_file",
    "monotonic_time",
]
