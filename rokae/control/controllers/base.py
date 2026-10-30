"""Controller lifecycle shared by position and impedance routes."""

from __future__ import annotations

import threading
from typing import Protocol

import numpy as np

from rokae.control.command import EECommand, PathSegment
from rokae.control.controllers.spec import ControllerSpec


class ArmBackend(Protocol):
    def start(self, power_on: bool = True, control_mode: str = "cartesian", kinematics: object = None) -> None: ...

    def set_target_pose(self, pose: np.ndarray, *, source_timestamp: float | None = None, sequence: int | None = None) -> bool: ...

    def set_target_joints(self, joints, *, source_timestamp: float | None = None, sequence: int | None = None) -> bool: ...

    def get_command_pose(self) -> np.ndarray: ...

    def get_command_joints(self) -> np.ndarray: ...

    def get_tcp_pose(self) -> np.ndarray: ...

    def get_joint_positions(self): ...

    def hold(self) -> None: ...

    def stop(self) -> None: ...

    def move_cartesian(self, pose: np.ndarray, *, speed: int, timeout: float, max_translation_delta: float, max_rotation_delta: float, power_on: bool) -> None: ...


class ControllerBase:
    def __init__(self, spec: ControllerSpec, driver: ArmBackend) -> None:
        self.spec = spec
        self.driver = driver
        self.state = "idle"
        self._lock = threading.Lock()
        self.last_accepted_pose = None
        self.timing = None

    @property
    def kind(self) -> str:
        return self.spec.kind

    def start(self, *, power_on: bool = True) -> None:
        raise NotImplementedError

    def set_target(self, command: EECommand) -> bool:
        raise NotImplementedError

    def command_pose(self) -> np.ndarray:
        raise NotImplementedError

    def hold(self) -> None:
        if self.state != "running":
            return
        self.driver.hold()

    def stop(self) -> None:
        if self.state == "idle":
            self.state = "stopped"
            return
        self.driver.stop()
        self.state = "stopped"

    def reset_tracking(self) -> None:
        return None

    def follow(self, segments: list[PathSegment], *, power_on: bool = True) -> None:
        raise NotImplementedError

    def _require_running(self) -> None:
        if self.state != "running":
            raise RuntimeError(f"controller is {self.state}; call start() first")
