"""Unified end-effector command.

Frames:
  base    robot base. +X forward, +Y left, +Z up. Teleop and tasks use this.
  world   same axis directions as base on the current upright cell.
  tcp     controller TCP. Open fingertips are 0.18 m along flange +Z.
  quest   Quest wrist, mapped into base before an EECommand is built.

Units: metres, radians, seconds from time.monotonic, Newtons, Nm.
Pose is a 4x4 row-major base_T_tcp matrix. Quaternion order is XYZW.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class EECommand:
    """Desired end-effector reference. The controller decides how to track it."""

    pose: np.ndarray
    timestamp: float
    sequence: int
    source: str = "task"
    gripper: float | None = None

    def __post_init__(self) -> None:
        self.pose = np.asarray(self.pose, dtype=np.float64)
        if self.pose.shape != (4, 4) or not np.all(np.isfinite(self.pose)):
            raise ValueError("EECommand.pose must be a finite 4x4 matrix")
        if not np.isfinite(self.timestamp):
            raise ValueError("EECommand.timestamp must be finite")


@dataclass
class PathSegment:
    """One Cartesian waypoint shared by position and impedance execution."""

    name: str
    pose: np.ndarray
    speed: int = 100
    max_translation_delta: float = 0.40
    max_rotation_delta: float = 1.20
    dwell_s: float = 0.0
    timeout_s: float = 60.0
