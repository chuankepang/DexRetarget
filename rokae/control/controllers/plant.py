"""Offline spring-damper used only to check impedance plumbing.

F = K * position_error - D * velocity, with the spring pulling toward the
equilibrium. This is not the xCore Cartesian impedance law and it is not
evidence that the robot controller is correct.
"""

from __future__ import annotations

import numpy as np

from rokae.control.reference_limit import rotation_from_vector, rotation_vector


class CompliantPlant:
    def __init__(
        self,
        pose: np.ndarray,
        stiffness: np.ndarray,
        damping: np.ndarray,
        mass: float = 1.0,
    ) -> None:
        self.pose = np.array(pose, dtype=np.float64, copy=True)
        self.equilibrium = self.pose.copy()
        self.stiffness = np.asarray(stiffness, dtype=np.float64)
        self.damping = np.asarray(damping, dtype=np.float64)
        self.mass = float(mass)
        self.linear_velocity = np.zeros(3, dtype=np.float64)
        self.angular_velocity = np.zeros(3, dtype=np.float64)
        self.last_wrench = np.zeros(6, dtype=np.float64)

    def set_equilibrium(self, pose: np.ndarray) -> None:
        self.equilibrium = np.array(pose, dtype=np.float64, copy=True)

    def step(self, dt: float) -> np.ndarray:
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("plant dt must be > 0")
        position_error = self.equilibrium[:3, 3] - self.pose[:3, 3]
        force = self.stiffness[:3] * position_error - self.damping[:3] * self.linear_velocity
        self.linear_velocity = self.linear_velocity + force / self.mass * dt
        self.pose[:3, 3] = self.pose[:3, 3] + self.linear_velocity * dt
        relative = self.pose[:3, :3].T @ self.equilibrium[:3, :3]
        rotation_error = rotation_vector(relative)
        torque = (
            self.stiffness[3:] * rotation_error
            - self.damping[3:] * self.angular_velocity
        )
        self.angular_velocity = self.angular_velocity + torque / self.mass * dt
        self.pose[:3, :3] = self.pose[:3, :3] @ rotation_from_vector(
            self.angular_velocity * dt
        )
        self.last_wrench = np.concatenate([force, torque])
        return self.last_wrench.copy()
