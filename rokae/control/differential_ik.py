"""Damped differential IK with redundancy resolution for xMate ER Pro 7.

The task-space step is damped least squares. Damping rises when the smallest
singular value drops, which is the usual way to stay finite at a singularity
instead of pretending the singularity is gone. The nullspace holds the
high-elbow commissioning posture, pushes J2/J4/J6 away from zero, and backs
off the joint limits. Those secondary terms follow the same split used by
libfranka joint-position examples and pink posture tasks.
"""

from __future__ import annotations

import numpy as np

from .model import XmateErPro7


def _clip_norm(value: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    if norm <= limit or norm < 1e-15:
        return value
    return value * (limit / norm)


class DifferentialIK:
    """Integrate a Cartesian target into a continuous joint command."""

    def __init__(self, robot: XmateErPro7, joints: np.ndarray) -> None:
        self.robot = robot
        self.gains = robot.ik_gains
        self.q = robot.clip(joints)

    def reset(self, joints: np.ndarray) -> None:
        self.q = self.robot.clip(joints)

    def step(self, base_T_tcp: np.ndarray, dt: float) -> np.ndarray:
        """One integration step. ``dt`` is the time since the previous target."""
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("IK dt must be positive")
        period = float(np.clip(dt, 1e-3, 0.05))
        error = self.robot.local_error(self.q, base_T_tcp)
        error[:3] = _clip_norm(error[:3], self.gains.max_translation_speed * period)
        error[3:] = _clip_norm(error[3:], self.gains.max_angular_speed * period)
        jacobian = self.robot.jacobian(self.q)
        singular = np.linalg.svd(jacobian, compute_uv=False)
        sigma_min = float(singular[-1])
        damping = self.gains.damping
        if sigma_min < self.gains.singular_sigma:
            ratio = 1.0 - sigma_min / self.gains.singular_sigma
            damping = self.gains.damping + (self.gains.damping_max - self.gains.damping) * ratio * ratio
        gram = jacobian @ jacobian.T
        damped = gram + (damping * damping) * np.eye(6, dtype=np.float64)
        primary = jacobian.T @ np.linalg.solve(damped, error)
        nullspace = np.eye(self.robot.dof) - jacobian.T @ np.linalg.solve(damped, jacobian)
        secondary = self._secondary_velocity()
        secondary = _clip_norm(secondary, self.gains.null_max_speed)
        delta = primary + (nullspace @ secondary) * period
        joint_cap = self.robot.max_velocity * period
        delta = np.clip(delta, -joint_cap, joint_cap)
        self.q = self.robot.clip(self.q + delta)
        return self.q.copy()

    def _secondary_velocity(self) -> np.ndarray:
        q = self.q
        posture = self.gains.posture_gain * (self.robot.neutral - q)
        margin = self.gains.limit_margin
        lower_gap = q - self.robot.position_min
        upper_gap = self.robot.position_max - q
        limit = np.zeros(self.robot.dof, dtype=np.float64)
        near_low = lower_gap < margin
        near_high = upper_gap < margin
        limit = np.where(near_low, (margin - lower_gap) / margin, limit)
        limit = np.where(near_high, -(margin - upper_gap) / margin, limit)
        singularity = np.zeros(self.robot.dof, dtype=np.float64)
        epsilon = 0.08
        for index in self.robot.singular_joints:
            value = float(q[index])
            if abs(value) < 1e-3:
                reference = float(self.robot.neutral[index])
                singularity[index] = math_sign(reference if abs(reference) > 1e-3 else 1.0)
                continue
            denominator = (value * value + epsilon * epsilon) ** 2
            singularity[index] = 2.0 * value * epsilon * epsilon / denominator
        return posture + self.gains.limit_gain * limit + self.gains.singularity_gain * singularity


def math_sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0
