"""Reference-relative world TCP commands and binary gripper mapping.

This module contains no robot SDK dependency.  It converts Quest wrist poses
and MediaPipe-compatible hand landmarks into a small command contract that a
robot-specific process can consume.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional

import numpy as np

from .pose import (
    interpolate_rotation,
    make_transform,
    matrix_to_rotation_vector,
    rotation_vector_to_matrix,
    validate_rotation,
    validate_transform,
)


@dataclass(frozen=True)
class WorldPoseCommand:
    """An absolute world TCP target generated from reference-relative VR motion."""

    timestamp: float
    reference_id: int
    world_translation_delta: np.ndarray
    world_rotation_delta_vector: np.ndarray
    world_target: np.ndarray


class ReferenceRelativeWorldMapper:
    """Map Quest wrist motion into an absolute world-frame TCP target.

    Translation and rotation deltas are spatial/world-relative:

    ``dp_world = R_world_quest @ (p_quest_now - p_quest_reference)``

    ``dR_world = R_world_quest @ (R_now @ R_reference.T) @ R_world_quest.T``

    The resulting target is anchored at ``T_world_tcp_reference``.  This keeps
    both KUKA arms moving along the same world axes even when their base frames
    or initial TCP orientations differ.
    """

    def __init__(
        self,
        quest_to_world_rotation: np.ndarray,
        world_tcp_reference: np.ndarray,
        translation_scale: float = 1.0,
        rotation_scale: float = 1.0,
        low_pass_alpha: float = 0.35,
        max_translation: float = 0.50,
        max_rotation: float = np.pi,
    ) -> None:
        if not np.isfinite(translation_scale) or translation_scale <= 0.0:
            raise ValueError("translation_scale must be positive")
        if not np.isfinite(rotation_scale) or rotation_scale < 0.0:
            raise ValueError("rotation_scale cannot be negative")
        if not 0.0 < low_pass_alpha <= 1.0:
            raise ValueError("low_pass_alpha must be in (0, 1]")
        if not np.isfinite(max_translation) or max_translation <= 0.0:
            raise ValueError("max_translation must be positive")
        if not np.isfinite(max_rotation) or not 0.0 < max_rotation <= np.pi:
            raise ValueError("max_rotation must be in (0, pi]")

        self.quest_to_world_rotation = validate_rotation(
            quest_to_world_rotation, "quest_to_world_rotation"
        )
        self.translation_scale = float(translation_scale)
        self.rotation_scale = float(rotation_scale)
        self.low_pass_alpha = float(low_pass_alpha)
        self.max_translation = float(max_translation)
        self.max_rotation = float(max_rotation)
        self._world_tcp_reference = validate_transform(
            world_tcp_reference, "world_tcp_reference"
        )
        self._quest_reference: Optional[np.ndarray] = None
        self._last_world_target = self._world_tcp_reference.copy()
        self._reference_id = 0

    @property
    def referenced(self) -> bool:
        return self._quest_reference is not None

    @property
    def reference_id(self) -> int:
        return self._reference_id

    @property
    def world_tcp_reference(self) -> np.ndarray:
        return self._world_tcp_reference.copy()

    @property
    def last_world_target(self) -> np.ndarray:
        return self._last_world_target.copy()

    def set_reference(
        self,
        quest_pose: np.ndarray,
        world_tcp_reference: Optional[np.ndarray] = None,
    ) -> None:
        """Capture a wrist reference, optionally preserving a held world target."""
        self._quest_reference = validate_transform(quest_pose, "quest_pose")
        if world_tcp_reference is not None:
            self._world_tcp_reference = validate_transform(
                world_tcp_reference, "world_tcp_reference"
            )
        self._last_world_target = self._world_tcp_reference.copy()
        self._reference_id += 1

    @staticmethod
    def _clip_vector(vector: np.ndarray, maximum: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= maximum:
            return vector
        return vector * (maximum / norm)

    def update(self, quest_pose: np.ndarray, timestamp: float) -> WorldPoseCommand:
        """Return one filtered and bounded absolute world-frame TCP target."""
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        current = validate_transform(quest_pose, "quest_pose")
        if self._quest_reference is None:
            self.set_reference(current)

        assert self._quest_reference is not None
        change = self.quest_to_world_rotation

        world_translation_delta = self.translation_scale * change @ (
            current[:3, 3] - self._quest_reference[:3, 3]
        )
        world_translation_delta = self._clip_vector(
            world_translation_delta, self.max_translation
        )
        raw_position = (
            self._world_tcp_reference[:3, 3] + world_translation_delta
        )

        quest_world_delta_rotation = (
            current[:3, :3] @ self._quest_reference[:3, :3].T
        )
        world_delta_rotation = (
            change @ quest_world_delta_rotation @ change.T
        )
        world_delta_rotvec = self.rotation_scale * matrix_to_rotation_vector(
            world_delta_rotation
        )
        world_delta_rotvec = self._clip_vector(
            world_delta_rotvec, self.max_rotation
        )
        raw_rotation = (
            rotation_vector_to_matrix(world_delta_rotvec)
            @ self._world_tcp_reference[:3, :3]
        )

        alpha = self.low_pass_alpha
        filtered_position = self._last_world_target[:3, 3] + alpha * (
            raw_position - self._last_world_target[:3, 3]
        )
        filtered_rotation = interpolate_rotation(
            self._last_world_target[:3, :3], raw_rotation, alpha
        )
        self._last_world_target = make_transform(
            filtered_position, filtered_rotation
        )
        filtered_world_rotation_delta = (
            filtered_rotation @ self._world_tcp_reference[:3, :3].T
        )
        return WorldPoseCommand(
            timestamp=float(timestamp),
            reference_id=self._reference_id,
            world_translation_delta=(
                filtered_position - self._world_tcp_reference[:3, 3]
            ),
            world_rotation_delta_vector=matrix_to_rotation_vector(
                filtered_world_rotation_delta
            ),
            world_target=self._last_world_target.copy(),
        )


@dataclass(frozen=True)
class BinaryGripperCommand:
    """Binary Robotiq command plus continuous diagnostics used to derive it."""

    command: int
    closure_score: float
    finger_curl: Mapping[str, float]
    joint_curl: Mapping[str, tuple[float, float]]


class BinaryGripperMapper:
    """Map 21 hand landmarks to ``0=open`` / ``1=close`` with hysteresis.

    Curl is computed from PIP/IP and DIP joint bend angles, so the result is
    invariant to hand position, wrist orientation, and hand size.  The four
    non-thumb fingers dominate the aggregate score; the thumb contributes a
    smaller term because thumb tracking and natural grasp styles vary more.
    """

    _CHAINS = {
        "thumb": (1, 2, 3, 4),
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "little": (17, 18, 19, 20),
    }
    _WEIGHTS = {
        "thumb": 0.10,
        "index": 0.225,
        "middle": 0.225,
        "ring": 0.225,
        "little": 0.225,
    }

    def __init__(
        self,
        close_threshold: float = 0.62,
        open_threshold: float = 0.38,
        initial_command: int = 0,
    ) -> None:
        if not 0.0 <= open_threshold < close_threshold <= 1.0:
            raise ValueError("thresholds must satisfy 0 <= open < close <= 1")
        if initial_command not in (0, 1):
            raise ValueError("initial_command must be 0 or 1")
        self.close_threshold = float(close_threshold)
        self.open_threshold = float(open_threshold)
        self._command = int(initial_command)

    @property
    def command(self) -> int:
        return self._command

    @staticmethod
    def _joint_curl(
        previous: np.ndarray, joint: np.ndarray, following: np.ndarray
    ) -> float:
        incoming = previous - joint
        outgoing = following - joint
        denominator = float(np.linalg.norm(incoming) * np.linalg.norm(outgoing))
        if denominator < 1e-10:
            raise ValueError("degenerate hand landmarks")
        cosine = float(np.clip(np.dot(incoming, outgoing) / denominator, -1.0, 1.0))
        internal_angle = float(np.arccos(cosine))
        # Straight is pi (0 curl); a 90-degree or tighter bend is full curl.
        return float(np.clip((np.pi - internal_angle) / (0.5 * np.pi), 0.0, 1.0))

    def update_score(self, score: float) -> int:
        """Apply the binary Schmitt trigger to a precomputed closure score."""
        if not np.isfinite(score) or not 0.0 <= score <= 1.0:
            raise ValueError("closure score must be finite and in [0, 1]")
        if self._command == 0 and score >= self.close_threshold:
            self._command = 1
        elif self._command == 1 and score <= self.open_threshold:
            self._command = 0
        return self._command

    def update(self, landmarks: np.ndarray) -> BinaryGripperCommand:
        points = np.asarray(landmarks, dtype=np.float64)
        if points.shape != (21, 3) or not np.all(np.isfinite(points)):
            raise ValueError("landmarks must be a finite (21, 3) array")
        if np.allclose(points, 0.0):
            raise ValueError("landmarks contain no tracked hand")

        finger_curl: dict[str, float] = {}
        joint_curl: dict[str, tuple[float, float]] = {}
        for name, (mcp, pip, dip, tip) in self._CHAINS.items():
            proximal = self._joint_curl(points[mcp], points[pip], points[dip])
            distal = self._joint_curl(points[pip], points[dip], points[tip])
            joint_curl[name] = (proximal, distal)
            finger_curl[name] = 0.5 * (proximal + distal)

        score = float(
            sum(self._WEIGHTS[name] * finger_curl[name] for name in self._CHAINS)
        )
        command = self.update_score(score)
        return BinaryGripperCommand(command, score, finger_curl, joint_curl)
