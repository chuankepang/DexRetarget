"""Damped differential IK with redundancy resolution for xMate ER Pro 7.

The task-space step is damped least squares. Damping rises when the smallest
singular value drops, which is the usual way to stay finite at a singularity
instead of pretending the singularity is gone. The secondary task uses the
geometric kernel and an explicit posture reference. Whole-vector scaling
preserves task directions when effective joint bounds become active.
"""

from __future__ import annotations

import numpy as np

from .model import XmateErPro7


def _clip_norm(value: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    if norm <= limit or norm < 1e-15:
        return value
    return value * (limit / norm)


def _bounded_scale(delta: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Largest feasible uniform scale, assuming zero is inside the box."""
    ratios = np.ones(delta.size)
    positive, negative = delta > 0, delta < 0
    ratios[positive] = upper[positive] / delta[positive]
    ratios[negative] = lower[negative] / delta[negative]
    return delta * float(np.clip(np.min(ratios), 0., 1.))


class DifferentialIK:
    """Integrate a Cartesian target into a continuous joint command."""

    def __init__(self, robot: XmateErPro7, joints: np.ndarray) -> None:
        self.robot = robot
        self.gains = robot.ik_gains
        self.q = robot.require_within_limits(joints).copy()
        self.posture_reference = self._reference(self.q)

    def _reference(self, joints: np.ndarray) -> np.ndarray:
        return (joints.copy() if self.gains.posture_reference == "initial"
                else self.robot.clip(self.robot.neutral))

    def reset(self, joints: np.ndarray) -> None:
        self.q = self.robot.require_within_limits(joints).copy()
        # e/c and tracking recovery create a new reference without changing
        # the Quest-to-tool mapping or the task's home configuration.
        self.posture_reference = self._reference(self.q)

    def step(self, base_T_tcp: np.ndarray, dt: float) -> np.ndarray:
        """One integration step. ``dt`` is the time since the previous target."""
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("IK dt must be positive")
        period = float(np.clip(dt, 1e-3, 0.05))
        error, jacobian = self.robot.error_and_jacobian(self.q, base_T_tcp)
        error[:3] = _clip_norm(error[:3], self.gains.max_translation_speed * period)
        error[3:] = _clip_norm(error[3:], self.gains.max_angular_speed * period)
        u, singular, vh = np.linalg.svd(jacobian, full_matrices=True)
        sigma_min = float(singular[-1])
        damping = self.gains.damping
        if sigma_min < self.gains.singular_sigma:
            ratio = 1.0 - sigma_min / self.gains.singular_sigma
            damping = self.gains.damping + (self.gains.damping_max - self.gains.damping) * ratio * ratio
        # Reuse the SVD needed for the kernel: algebraically identical DLS,
        # without a second dense factorization or another kinematic traversal.
        primary = vh[:jacobian.shape[0]].T @ (
            (singular / (singular * singular + damping * damping)) * (u.T @ error))
        # Keep only the structural redundancy (7-6), even near rank loss.
        # DLS is still used for the primary task; it is not a kernel projector.
        basis = vh[jacobian.shape[0]:].T
        secondary = self._secondary_velocity(sigma_min)
        secondary = _clip_norm(secondary, self.gains.null_max_speed)
        null_delta = (basis @ (basis.T @ secondary)) * period
        # Fade secondary motion near rank loss, where the kernel may change
        # rapidly. This does not claim to remove geometric singularities.
        null_delta *= min(1., sigma_min / self.gains.singular_sigma) ** 2
        joint_cap = self.robot.max_velocity * period
        lower = np.maximum(-joint_cap, self.robot.position_min - self.q)
        upper = np.minimum(joint_cap, self.robot.position_max - self.q)
        primary = _bounded_scale(primary, lower, upper)
        null_delta = _bounded_scale(null_delta, lower-primary, upper-primary)
        self.q = self.robot.clip(self.q + primary + null_delta)
        return self.q.copy()

    def _secondary_velocity(self, sigma_min: float | None = None) -> np.ndarray:
        q = self.q
        posture = self.gains.posture_gain * (self.posture_reference - q)
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
        if sigma_min is None:
            sigma_min = self.robot.smallest_singular_value(q)
        # The bend heuristic acts only near an actual Jacobian singularity.
        # A well-conditioned stationary hand must not drift toward an old pose.
        for index in self.robot.singular_joints if sigma_min < self.gains.singular_sigma else ():
            value = float(q[index])
            if abs(value) < 1e-3:
                reference = float(self.posture_reference[index])
                singularity[index] = math_sign(reference if abs(reference) > 1e-3 else 1.0)
                continue
            denominator = (value * value + epsilon * epsilon) ** 2
            singularity[index] = 2.0 * value * epsilon * epsilon / denominator
        singular_weight = max(0., 1. - sigma_min / self.gains.singular_sigma) ** 2
        return posture + self.gains.limit_gain * limit + self.gains.singularity_gain * singular_weight * singularity


def math_sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0
